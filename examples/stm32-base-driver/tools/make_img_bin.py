# -*- coding: utf-8 -*-
"""test_img/*.bmp → img_bin/img{0,1,2}.bin  320x240 RGB565 大端"""
from pathlib import Path
from PIL import Image

src = Path(r"D:\MCU\0.fireflyluo-Embedded-Libs-main\examples\stm32-base-driver\test_img")
dst = Path(r"D:\MCU\0.fireflyluo-Embedded-Libs-main\examples\stm32-base-driver\img_bin")
dst.mkdir(parents=True, exist_ok=True)

names = ["1.bmp", "2.bmp", "3.bmp"]
for i, name in enumerate(names):
    im = Image.open(src / name).convert("RGB")
    if im.size != (320, 240):
        im = im.resize((320, 240), Image.LANCZOS)
    raw = im.tobytes()  # RGB888
    out = bytearray()
    for p in range(0, len(raw), 3):
        r, g, b = raw[p], raw[p + 1], raw[p + 2]
        c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        out.append((c >> 8) & 0xFF)  # big-endian
        out.append(c & 0xFF)
    path = dst / f"img{i}.bin"
    path.write_bytes(out)
    print(path.name, len(out), "bytes")
