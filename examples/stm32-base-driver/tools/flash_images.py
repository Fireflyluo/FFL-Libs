# -*- coding: utf-8 -*-
"""
经 COM 口把 img_bin/*.bin 写入板载 W25Q（需已烧录支持 PROTO 的固件）。

协议（无校验和，分块）：
  主机 -> 固件 ASCII 行：
    P\n          进入编程
    W addr len\n 写 len 字节（随后发 raw）
    R addr len\n 读 len 字节（固件回 raw）
    E addr\n     4KB 扇区擦除
    Q\n          退出编程

用法：
  python tools/flash_images.py COM4
"""
import sys
import time
import os

try:
    import serial
except ImportError:
    print("need pyserial: pip install pyserial")
    sys.exit(1)

BASE = os.path.join(os.path.dirname(__file__), "..")
BIN = os.path.join(BASE, "img_bin")
IMGS = [
    ("img0.bin", 0x000000),
    ("img1.bin", 0x010000),
    ("img2.bin", 0x020000),
]
BLOCK = 256

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    ser = serial.Serial(port_name, 115200, timeout=2)
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(b"P\n")
    time.sleep(0.05)

    for fn, addr in IMGS:
        path = os.path.join(BIN, fn)
        data = open(path, "rb").read()
        print("write", fn, "addr=0x%06X len=%d" % (addr, len(data)))
        # 擦除
        nsec = (len(data) + 4095) // 4096
        for s in range(nsec):
            ser.write(("E %d\n" % (addr + s * 4096)).encode())
            time.sleep(0.02)
        # 写
        off = 0
        while off < len(data):
            chunk = data[off:off + BLOCK]
            ser.write(("W %d %d\n" % (addr + off, len(chunk))).encode())
            ser.write(chunk)
            off += len(chunk)
            if off % 4096 == 0:
                print("  ...", off)
        # 抽检
        ser.write(("R %d 64\n" % addr).encode())
        back = ser.read(64)
        if back != data[:64]:
            print("VERIFY FAIL head", back[:16].hex(), "vs", data[:16].hex())
        else:
            print("  head OK")
    ser.write(b"Q\n")
    ser.close()
    print("done")

if __name__ == "__main__":
    main()
