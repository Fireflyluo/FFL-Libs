# -*- coding: utf-8 -*-
"""将 test_img 下三张图转为 80x64 RGB565 C 数组（片内源），供写入外部 Flash。"""
from PIL import Image
import os

base = r"D:\MCU\0.fireflyluo-Embedded-Libs-main\examples\stm32-base-driver"
srcs = [
    ("test.bmp", "img0"),
    ("test2.jpg", "img1"),
    ("test3.jpg", "img2"),
]
W, H = 80, 64

def rgb565(im):
    im = im.convert("RGB").resize((W, H), Image.BILINEAR)
    px = im.load()
    data = bytearray()
    for y in range(H):
        for x in range(W):
            r, g, b = px[x, y]
            c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            data.append((c >> 8) & 0xFF)
            data.append(c & 0xFF)
    return data

hdr = """#ifndef {NAME}_H
#define {NAME}_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {{
#endif
#define {NAME}_W 80
#define {NAME}_H 64
#define {NAME}_BYTES ({NAME}_W * {NAME}_H * 2u)
extern const uint8_t {name}_rgb565[{NAME}_BYTES];
#ifdef __cplusplus
}}
#endif
#endif
"""

for fn, name in srcs:
    path = os.path.join(base, "test_img", fn)
    im = Image.open(path)
    data = rgb565(im)
    NAME = name.upper()
    out_h = os.path.join(base, "app", name + ".h")
    out_c = os.path.join(base, "app", name + ".c")
    with open(out_h, "w", encoding="utf-8") as f:
        f.write(hdr.format(NAME=NAME, name=name))
    with open(out_c, "w", encoding="utf-8") as f:
        f.write("/* %s -> 80x64 RGB565 */\n#include \"%s.h\"\n\n" % (fn, name))
        f.write("const uint8_t %s_rgb565[%s_BYTES] = {\n" % (name, NAME))
        for i in range(0, len(data), 16):
            f.write("  " + ", ".join("0x%02X" % b for b in data[i:i+16]) + ",\n")
        f.write("};\n")
    print(name, "bytes", len(data), "c", os.path.getsize(out_c))
