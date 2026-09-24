# -*- coding: utf-8 -*-
"""将 test_img 三张图转为 160x128 RGB565 二进制（供经串口写入 W25Q）。"""
from PIL import Image
import os

base = r"D:\MCU\0.fireflyluo-Embedded-Libs-main\examples\stm32-base-driver"
out_dir = os.path.join(base, "img_bin")
os.makedirs(out_dir, exist_ok=True)
W, H = 160, 128

def to_rgb565(path):
    im = Image.open(path).convert("RGB").resize((W, H), Image.LANCZOS)
    px = im.load()
    buf = bytearray()
    for y in range(H):
        for x in range(W):
            r, g, b = px[x, y]
            c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            buf.append((c >> 8) & 0xFF)
            buf.append(c & 0xFF)
    return bytes(buf)

for i, name in enumerate(["test.bmp", "test2.jpg", "test3.jpg"]):
    src = os.path.join(base, "test_img", name)
    data = to_rgb565(src)
    dst = os.path.join(out_dir, "img%d.bin" % i)
    with open(dst, "wb") as f:
        f.write(data)
    print(name, "->", dst, len(data))
