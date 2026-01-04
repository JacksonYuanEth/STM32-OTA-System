import serial
import time
import sys
import os

# === 配置区 ===
SERIAL_PORT = 'COM9'      # 树莓派上通常是 '/dev/ttyUSB0' 或 '/dev/ttyAMA0'
BAUD_RATE = 115200

# 固件路径
FIRMWARE_NEW = 'App_v2.0.bin'  # 想要升级的新固件
FIRMWARE_OLD = 'App_v1.0.bin'  # 备份的稳定旧固件

# 协议常量
CMD_JUMP_BOOT = 0x10
CMD_ERASE     = 0x20
CMD_WRITE     = 0x21
CMD_RESET     = 0x30
HEARTBEAT_MSG = b"APP_"      # APP 启动后必须发送这个心跳包
VERIFY_TIMEOUT = 10            # 验证等待时间 (秒)
# =============

def calc_checksum(data):
    return sum(data) & 0xFF

def send_packet(ser, cmd, payload=[]):
    """发送协议包并等待ACK"""
    length = 1 + len(payload)
    frame = [0xAA, 0x55, length, cmd] + payload
    checksum = calc_checksum(frame)
    frame.append(checksum)
    
    ser.write(bytearray(frame))
    
    # 简单接收 ACK (AA 55 02 CMD STATUS)
    # 实际项目中建议增加超时重试
    try:
        resp = ser.read(5)
        if len(resp) == 5 and resp[0]==0xAA and resp[1]==0x55:
            if resp[3] == cmd and resp[4] == 0x00:
                return True
    except:
        pass
    return False

def wait_for_bootloader_ready(ser):
    """等待 Bootloader 发送 Ready 信号 (0x55)"""
    print("   Waiting for Bootloader Ready (0x55)...")
    start_t = time.time()
    while time.time() - start_t < 5:
        if ser.in_waiting:
            c = ser.read(1)
            if c == b'\x55':
                print("   Bootloader is READY!")
                return True
    print("   Timeout waiting for Bootloader.")
    return False

def flash_firmware(ser, file_path):
    """执行完整的烧录流程：读取文件 -> 擦除 -> 写入 -> 重启"""
    print(f"----------------------------------------")
    print(f"Start Flashing: {file_path}")
    
    # 1. 读取文件
    try:
        with open(file_path, 'rb') as f:
            bin_data = f.read()
        print(f"   File loaded: {len(bin_data)} bytes")
    except FileNotFoundError:
        print("   Error: Firmware file not found!")
        return False

    # 2. 请求进入 Bootloader
    # 如果已经在 Bootloader 中，这一步可能无响应，忽略即可
    print(">> Step 1: Requesting Bootloader...")
    send_packet(ser, CMD_JUMP_BOOT, [])
    time.sleep(0.5) # 给一点时间重启
    
    # 清空缓冲区，等待 Bootloader 上线
    ser.reset_input_buffer()
    
    # 尝试捕捉 Ready 信号 (如果板子刚重启)
    if not wait_for_bootloader_ready(ser):
        # 如果没收到 Ready，可能是已经在 Bootloader 里了，尝试直接发指令
        print("   Assuming Bootloader is active, trying to erase...")

    # 3. 擦除
    print(">> Step 2: Erasing Flash...")
    # 这里可以增加重试机制
    retry = 3
    while retry > 0:
        if send_packet(ser, CMD_ERASE, []):
            print("   Erase OK!")
            break
        else:
            print("   Erase No Response, retrying...")
            time.sleep(1)
            retry -= 1
    if retry == 0:
        print("   Erase FAILED! Board might be stuck.")
        return False

    # 4. 写入
    print(">> Step 3: Flashing Data...")
    CHUNK_SIZE = 128
    total_chunks = (len(bin_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
    
    for i in range(total_chunks):
        offset = i * CHUNK_SIZE
        chunk = list(bin_data[offset : offset + CHUNK_SIZE])
        
        offset_bytes = [
            (offset >> 24) & 0xFF,
            (offset >> 16) & 0xFF,
            (offset >> 8) & 0xFF,
            (offset) & 0xFF
        ]
        
        if not send_packet(ser, CMD_WRITE, offset_bytes + chunk):
            print(f"   Write Error at chunk {i}")
            return False
        
        # 打印进度条
        progress = (i + 1) / total_chunks * 100
        print(f"\r   Progress: {progress:.1f}%", end='')
    print("\n   Write Complete!")

    # 5. 重启运行 APP
    print(">> Step 4: Resetting to APP...")
    send_packet(ser, CMD_RESET, [])
    return True

def verify_app_running(ser):
    """监听串口，看是否收到 APP 发来的心跳包"""
    print(f">> Verifying APP logic (Timeout: {VERIFY_TIMEOUT}s)...")
    ser.timeout = 0.1 # 设置非阻塞读
    start_time = time.time()
    
    buffer = b""
    while time.time() - start_time < VERIFY_TIMEOUT:
        if ser.in_waiting:
            buffer += ser.read(ser.in_waiting)
            if HEARTBEAT_MSG in buffer:
                print(f"   [SUCCESS] Heartbeat '{HEARTBEAT_MSG}' detected!")
                return True
        time.sleep(0.1)
    
    print("   [FAIL] No Heartbeat received!")
    return False

def main():
    try:
        # 打开串口
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
        print(f"Opened {SERIAL_PORT} successfully.")
    except Exception as e:
        print(f"Serial Error: {e}")
        return

    # === 尝试 1: 刷入新固件 ===
    print("\n========== ATTEMPT 1: UPDATING TO V2.0 ==========")
    if flash_firmware(ser, FIRMWARE_NEW):
        # 刷写过程成功，现在等待 APP 启动并验证
        # 清空缓冲区，避免读到旧数据
        ser.reset_input_buffer()
        time.sleep(1) # 等待 STM32 初始化
        
        if verify_app_running(ser):
            print("\n========== UPDATE SUCCESSFUL! ==========")
            ser.close()
            return
        else:
            print("\n!!!!!!!!!! UPDATE FAILED (APP CRASHED) !!!!!!!!!!")
    else:
        print("\n!!!!!!!!!! FLASHING FAILED !!!!!!!!!!")

    # === 尝试 2: 失败回滚 ===
    print("\n========== ROLLBACK INITIATED: RESTORING V1.0 ==========")
    print(">> Attempting to rescue the board...")
    
    # 关键点：如果 APP 挂了，可能无法响应 Jump 指令。
    # 1. 树莓派可以尝试疯狂发送 Jump 指令 (0x10)
    # 2. 或者提示用户手动复位 (如果有 GPIO 连接 RESET 引脚，这里代码控制拉低复位)
    
    # 这里模拟死循环发送 Jump 指令抢占 Bootloader
    # 实际 Bootloader 应该设计为：上电后等待几百毫秒再跳转 APP，给树莓派机会
    for _ in range(5):
        ser.write(bytearray([0xAA, 0x55, 0x01, 0x10, 0x10])) # 盲发
        time.sleep(0.2)
    
    if flash_firmware(ser, FIRMWARE_OLD):
        print("\n========== ROLLBACK SUCCESSFUL ==========")
        print("System restored to V1.0. Please check V2.0 code.")
    else:
        print("\n========== CRITICAL ERROR: BOARD MIGHT BE BRICKED ==========")
        print("Manual intervention required.")

    ser.close()

if __name__ == '__main__':
    main()