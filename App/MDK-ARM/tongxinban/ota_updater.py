import serial
import time
import struct
import sys

# === 配置区 ===
SERIAL_PORT = 'COM3'   # 你的 USB 转 TTL 串口号
BAUD_RATE = 115200
FIRMWARE_FILE = 'App.bin' # 你的固件文件
# =============

def calc_checksum(data):
    return sum(data) & 0xFF

def send_packet(ser, cmd, payload=[]):
    # 构造帧: [AA 55] [LEN] [CMD] [PAYLOAD] [CS]
    # LEN = 1 (CMD) + len(PAYLOAD)
    length = 1 + len(payload)
    frame = [0xAA, 0x55, length, cmd] + payload
    checksum = calc_checksum(frame)
    frame.append(checksum)
    
    ser.write(bytearray(frame))
    
    # 等待 ACK: [AA 55 02 CMD STATUS]
    resp = ser.read(5)
    if len(resp) == 5 and resp[0]==0xAA and resp[1]==0x55:
        if resp[3] == cmd and resp[4] == 0x00:
            return True
    return False

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        print(f"Connected to {SERIAL_PORT}")
    except:
        print("Error opening serial port!")
        return

    # 1. 读取固件
    try:
        with open(FIRMWARE_FILE, 'rb') as f:
            bin_data = f.read()
        print(f"Firmware loaded: {len(bin_data)} bytes")
    except:
        print("File not found!")
        return

    # 2. 发送 [跳转 Bootloader] 指令
    # 注意：如果板子已经在 Bootloader 里了，这步可能会超时，可以忽略
    print(">> Step 1: Requesting Bootloader...")
    send_packet(ser, 0x10, []) 
    time.sleep(1.5) # 等待 STM32 重启

    # 清空串口缓冲区，准备接收 Bootloader 的 Ready 信号
    ser.reset_input_buffer()
    
    # 3. 发送 [擦除] 指令
    print(">> Step 2: Erasing Flash...")
    if send_packet(ser, 0x20, []):
        print("   Erase OK!")
    else:
        print("   Erase Failed or Timeout! (Is board in Bootloader?)")
        # 这里可以选择退出，或者强行继续尝试
        # return 

    # 4. 发送 [写入] 指令 (分包发送)
    print(">> Step 3: Flashing Firmware...")
    CHUNK_SIZE = 256 # 每次发 256 字节
    total_chunks = (len(bin_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
    
    for i in range(total_chunks):
        offset = i * CHUNK_SIZE
        chunk = list(bin_data[offset : offset + CHUNK_SIZE])
        
        # 构造 Payload: [Offset 4bytes] + [Data]
        offset_bytes = [
            (offset >> 24) & 0xFF,
            (offset >> 16) & 0xFF,
            (offset >> 8) & 0xFF,
            (offset) & 0xFF
        ]
        
        if send_packet(ser, 0x21, offset_bytes + chunk):
            print(f"   Writing chunk {i+1}/{total_chunks}... OK")
        else:
            print(f"   Writing chunk {i+1} Failed!")
            return

    # 5. 发送 [重启] 指令
    print(">> Step 4: Resetting Board...")
    send_packet(ser, 0x30, [])
    print(">> Update Complete! APP should be running.")

    ser.close()

if __name__ == '__main__':
    main()