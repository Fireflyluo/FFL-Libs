# -*- coding: utf-8 -*-
"""已停用：图像写入仅支持 USB CDC 分包协议。"""
import sys

print("UART image upload is unsupported. Use tools/usb_upload_img.py instead.", file=sys.stderr)
sys.exit(2)
