# -*- coding: utf-8 -*-
"""通过 CherryUSB CDC 分包写入 W25Q 图像（每包写后 ACK）。"""
import sys
import time
from pathlib import Path

import serial

ROOT = Path(r"D:\MCU\0.fireflyluo-Embedded-Libs-main\examples\stm32-base-driver")
BIN = ROOT / "img_bin"

MAGIC_REQ = b"IP"
MAGIC_ACK = b"IA"
VERSION = 1
BEGIN = 1
DATA = 2
END = 3
ABORT = 4
ACK = 0x80
DATA_MAX = 48


def find_cdc(prefer=None):
    import serial.tools.list_ports as lp

    ports = list(lp.comports())
    names = [port.device for port in ports]
    print("ports:", [(p.device, p.description) for p in ports])
    if prefer and prefer in names:
        return prefer
    for port in ports:
        hw = " ".join((port.hwid or "", port.description or ""))
        if "FF55:5711" in hw or "5711" in hw:
            return port.device
    # 不用调试器 VCP（PowerWriter / WCH / CMSIS-DAP C251）
    for port in ports:
        desc = " ".join((port.description or "", port.hwid or ""))
        if "PowerWriter" in desc or "WCH" in desc or "C251" in desc or "CZ_2023" in desc:
            continue
        if port.device.startswith("COM") and port.device not in ("COM15", "COM16"):
            return port.device
    return None


def crc16_ccitt(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_packet(kind, sequence, image_index, offset, payload=b""):
    if len(payload) > DATA_MAX:
        raise ValueError("payload exceeds 48 bytes")
    header = bytearray(MAGIC_REQ)
    header.extend((VERSION, kind))
    header.extend(sequence.to_bytes(2, "little"))
    header.extend((image_index, len(payload)))
    header.extend(offset.to_bytes(4, "little"))
    frame = bytes(header) + payload
    return frame + crc16_ccitt(frame).to_bytes(2, "little")


def read_exact(port, count, timeout):
    deadline = time.monotonic() + timeout
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        chunk = port.read(count - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def send_packet(port, packet, sequence, timeout=5, retries=3):
    for attempt in range(1, retries + 1):
        port.write(packet)
        port.flush()
        reply = read_exact(port, 12, timeout)
        if len(reply) != 12:
            print(f"  timeout seq={sequence} attempt={attempt}")
            continue
        if reply[:2] != MAGIC_ACK or reply[2] != VERSION:
            print(f"  invalid reply seq={sequence}: {reply.hex()}")
            continue
        ack_sequence = int.from_bytes(reply[4:6], "little")
        status = reply[6]
        committed = int.from_bytes(reply[8:12], "little")
        if ack_sequence != sequence:
            print(f"  stale reply seq={ack_sequence}, expected={sequence}")
            continue
        if reply[3] == ACK and status == 0:
            return committed
        if status == 3:
            # STATUS_STATE：设备会话不一致，发 ABORT 后重来
            print(f"  STATE nack seq={sequence}, send ABORT")
            try:
                port.write(make_packet(ABORT, sequence, 0, 0, b""))
                port.flush()
                port.read(64)
            except Exception:
                pass
            raise RuntimeError(f"NACK seq={sequence} status={status} committed=0x{committed:X}")
        raise RuntimeError(
            f"NACK seq={sequence} status={status} committed=0x{committed:X}"
        )
    raise TimeoutError(f"ACK timeout seq={sequence}")


def upload(port_name):
    port = serial.Serial(port_name, 115200, timeout=0.2, write_timeout=30)
    port.dtr = True
    port.rts = False
    time.sleep(0.3)
    port.reset_input_buffer()
    print("open", port_name)
    try:
        for image_index in range(3):
            data = (BIN / f"img{image_index}.bin").read_bytes()
            print(f"== img{image_index} {len(data)} ==")
            port.reset_input_buffer()
            try:
                port.write(make_packet(ABORT, 0, image_index, 0, b""))
                port.flush()
                time.sleep(0.05)
                port.reset_input_buffer()
            except Exception:
                pass
            sequence = 0
            committed = send_packet(
                port,
                make_packet(BEGIN, sequence, image_index, 0, len(data).to_bytes(4, "little")),
                sequence,
                timeout=45,
            )
            if committed != 0:
                raise RuntimeError(f"BEGIN committed={committed}")
            sequence += 1
            offset = 0
            while offset < len(data):
                size = min(DATA_MAX, len(data) - offset)
                committed = send_packet(
                    port,
                    make_packet(DATA, sequence, image_index, offset, data[offset:offset + size]),
                    sequence,
                )
                offset += size
                if committed != offset:
                    raise RuntimeError(
                        f"DATA seq={sequence} committed={committed}, expected={offset}"
                    )
                sequence = (sequence + 1) & 0xFFFF
                if offset % 32768 == 0:
                    print(f"  {offset}/{len(data)}")
            committed = send_packet(
                port, make_packet(END, sequence, image_index, offset), sequence, timeout=15
            )
            if committed != len(data):
                raise RuntimeError(f"END committed={committed}, expected={len(data)}")
            print(f"  img{image_index} OK")
    finally:
        port.close()
    print("all images written")
    return 0


if __name__ == "__main__":
    preferred = sys.argv[1] if len(sys.argv) > 1 else None
    selected = find_cdc(preferred)
    if not selected or selected == "COM9":
        print("NO_USB_CDC (FF55:5711) — 请**拔插 USB 线**，等出现 COM6 后：")
        print("  python tools\\usb_upload_img.py COM6")
        sys.exit(1)
    try:
        sys.exit(upload(selected))
    except (RuntimeError, TimeoutError, OSError) as exc:
        print(f"UPLOAD_FAIL: {exc}")
        print("若 USB 异常：拔插 USB 后重试 python tools\\usb_upload_img.py COM6")
        sys.exit(2)
