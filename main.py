"""2026 工科赛视觉程序，单文件版。

把本文件上传到 K230D 后，在 CanMV IDE 直接运行。
唯一入口：main()
"""

import os
import sys
import time
import gc
import image
from media.sensor import Sensor, CAM_CHN_ID_0
from media.display import Display
from media.media import MediaManager
from machine import UART, FPIOA


# ========== 雨轩优先修改这里 ==========
# LOCAL：单独调试；REMOTE：接 F407 后再启用。
RUN_MODE = "LOCAL"

# PREVIEW / QR / BOMB / TARGET / RESCUE / IDLE
MODE = "PREVIEW"

# 单项调试用：第一位=排爆颜色，第二位=靶颜色，第三位=救援物形状。
# 1/2/3 分别是 红/绿/蓝；救援物 1/2/3 是 圆柱/圆台/腰鼓。
TASK = "211"

SENSOR_ID = 2
WIDTH = 640
HEIGHT = 480
DISPLAY_MODE = "VIRT"  # VIRT=IDE图传，LCD=3.1寸屏，NONE=无显示。
HMIRROR = False
VFLIP = False

# 首次运行保持 False；接 F407 后核对实际 GPIO 才填写并改 True。
UART_ENABLED = False
UART_TX_GPIO = None  # 芯片 GPIO 编号，不是排针序号。
UART_RX_GPIO = None
UART_BAUDRATE = 115200

# LAB 颜色阈值起点，必须在真实道具和现场光线下调。
COLOR_LAB = {
    1: (15, 75, 20, 80, 0, 65),       # 红
    2: (15, 80, -75, -15, -15, 65),   # 绿
    3: (10, 70, -25, 30, -75, -15),   # 蓝
}
WHITE_LAB = (55, 100, -15, 15, -15, 15)

ROI = (0, 0, WIDTH, HEIGHT)  # 检测范围；云台姿态固定后可缩小，排除背景。
MIN_PIXELS = 150
MIN_AREA = 200
STABLE_FRAMES = 3
STABLE_RADIUS = 12
MAX_FRAME_GAP_MS = 400
SETTLE_MS = 400
SEND_INTERVAL_MS = 100
LINK_TIMEOUT_MS = 1500
CIRCLE_THRESHOLD = 2000

# 白色救援物从侧面观察时的轮廓规则参数。
CYLINDER_MAX_SPREAD = 1.10
CONE_MIN_TAPER = 1.20
CONE_MAX_TAPER = 2.20
DRUM_MIN_BULGE = 1.12


def crc16(data):
    """CRC16/CCITT-FALSE，避免串口乱码触发指令。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_packet(*fields):
    """串口帧：$V1,正文*CRC\\r\\n。"""
    body = ",".join(["V1"] + [str(value) for value in fields]).encode("ascii")
    return b"$" + body + ("*%04X\r\n" % crc16(body)).encode("ascii")


def read_packets(buffer, data, now_ms):
    """处理半包、粘包、噪声；半包 500 ms 未完成就丢弃。"""
    result = []
    if buffer["data"] and time.ticks_diff(now_ms, buffer["time"]) > 500:
        buffer["data"] = bytearray()
    for byte in data:
        if byte == 36:  # $
            buffer["data"] = bytearray(b"$")
            buffer["time"] = now_ms
        elif buffer["data"]:
            buffer["data"].append(byte)
            if len(buffer["data"]) > 160:
                buffer["data"] = bytearray()
            elif byte == 10:
                raw = bytes(buffer["data"])
                buffer["data"] = bytearray()
                try:
                    body, check = raw[1:].rstrip(b"\r\n").rsplit(b"*", 1)
                    if len(check) == 4 and crc16(body) == int(check, 16):
                        fields = body.decode("ascii").split(",")
                        if fields[0] == "V1":
                            result.append(fields[1:])
                except (ValueError, UnicodeError):
                    pass
    return result


def parse_task(text):
    if not isinstance(text, str) or len(text) != 3 or any(char not in "123" for char in text):
        return None
    return (int(text[0]), int(text[1]), int(text[2]))


def classify_shape(widths):
    """五段横截面宽度：1圆柱，2圆台，3腰鼓，0不确定。"""
    if len(widths) != 5 or min(widths) <= 0:
        return 0
    top, upper, middle, lower, bottom = widths
    if max(widths) / min(widths) <= CYLINDER_MAX_SPREAD:
        return 1
    taper = bottom / top
    if (CONE_MIN_TAPER <= taper <= CONE_MAX_TAPER
            and upper >= top * 0.97 and middle >= upper * 0.97
            and lower >= middle * 0.97 and bottom >= lower * 0.97):
        return 2
    if (middle / max(top, bottom) >= DRUM_MIN_BULGE
            and upper > top and lower > bottom and middle >= upper and middle >= lower):
        return 3
    return 0


def single_blob(img, threshold):
    """只接受一个完整色块，避免多目标时任意选一个。"""
    blobs = img.find_blobs([threshold], roi=ROI, pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_AREA, merge=False, x_stride=2, y_stride=1)
    if len(blobs) != 1:
        return None
    blob = blobs[0]
    if (blob.x() <= ROI[0] or blob.y() <= ROI[1]
            or blob.x() + blob.w() >= ROI[0] + ROI[2]
            or blob.y() + blob.h() >= ROI[1] + ROI[3]):
        return None
    return blob


def detect_qr(img):
    candidates = []
    for code in img.find_qrcodes(roi=ROI):
        task = parse_task(code.payload())
        if task:
            x, y, w, h = code.rect()
            candidates.append((task, x + w // 2, y + h // 2, w, h))
    return candidates[0] if len(candidates) == 1 else None


def detect_ball(img, color_id):
    blob = single_blob(img, COLOR_LAB[color_id])
    if blob is None:
        return None
    w, h = blob.w(), blob.h()
    if 0.45 <= w / h <= 1.55 and blob.pixels() / (w * h) >= 0.25:
        return (color_id, blob.cx(), blob.cy(), w, h)
    return None


def detect_target(img, color_id):
    """彩色外环筛选，再取靶内同心圆的中心。"""
    blob = single_blob(img, COLOR_LAB[color_id])
    if blob is None:
        return None
    w, h = blob.w(), blob.h()
    if not 0.60 <= w / h <= 1.55:
        return None
    centers = []
    for circle in img.find_circles(roi=blob.rect(), threshold=CIRCLE_THRESHOLD):
        if (0.07 * min(w, h) <= circle.r() <= 0.60 * max(w, h)
                and abs(circle.x() - (blob.x() + w / 2)) <= 0.20 * w
                and abs(circle.y() - (blob.y() + h / 2)) <= 0.20 * h):
            centers.append((circle.x(), circle.y()))
    if not centers:
        return None
    xs, ys = sorted(p[0] for p in centers), sorted(p[1] for p in centers)
    if xs[-1] - xs[0] > 0.12 * w or ys[-1] - ys[0] > 0.12 * h:
        return None
    return (color_id, xs[len(xs) // 2], ys[len(ys) // 2], w, h)


def row_width(mask, x, y, width):
    """在一行二值图里找最长白段。"""
    runs, start = [], None
    for px in range(x, x + width):
        white = mask.get_pixel(px, y)[0] > 127
        if white and start is None:
            start = px
        elif not white and start is not None:
            runs.append((px - start, start))
            start = None
    if start is not None:
        runs.append((x + width - start, start))
    if not runs:
        return 0, 0
    runs.sort(reverse=True)
    if len(runs) > 1 and runs[1][0] > runs[0][0] * 0.35:
        return 0, 0
    length, left = runs[0]
    return length, left + length / 2


def detect_rescue(img, shape_id):
    """仅适合白色救援物的侧视轮廓；返回 目标、五段宽度。"""
    blob = single_blob(img, WHITE_LAB)
    if blob is None or blob.w() < 18 or blob.h() < 25:
        return None, None
    mask = img.copy()
    mask.binary([WHITE_LAB])
    widths, centers = [], []
    for ratio in (0.20, 0.35, 0.50, 0.65, 0.80):
        y = blob.y() + int((blob.h() - 1) * ratio)
        values = [row_width(mask, blob.x(), y + offset, blob.w()) for offset in (-1, 0, 1)]
        values.sort()
        widths.append(values[1][0])
        centers.append(values[1][1])
    del mask
    if min(widths) < 8 or max(centers) - min(centers) > blob.w() * 0.18:
        return None, widths
    found_shape = classify_shape(widths)
    if found_shape != shape_id:
        return None, widths
    return (found_shape, blob.cx(), blob.cy(), blob.w(), blob.h()), widths


def stable(previous, current):
    return (previous is not None and previous[0] == current[0]
            and (previous[1] - current[1]) ** 2 + (previous[2] - current[2]) ** 2
            <= STABLE_RADIUS ** 2)


def show(canvas, raw, mode, status, target, task, fps):
    canvas.clear()
    canvas.draw_image(raw, 0, 0)
    if target:
        _, x, y, w, h = target
        canvas.draw_rectangle(x - w // 2, y - h // 2, w, h, color=(0, 255, 0))
        canvas.draw_cross(x, y, color=(0, 255, 0), size=10)
    canvas.draw_cross(WIDTH // 2, HEIGHT // 2, color=(255, 0, 0), size=10)
    code = "".join(str(value) for value in task) if task else "---"
    canvas.draw_string_advanced(4, 4, 20, "%s  %s" % (mode, status), color=(255, 255, 0))
    canvas.draw_string_advanced(4, 30, 18, "task:%s fps:%.1f" % (code, fps), color=(255, 255, 0))
    Display.show_image(canvas)


def main():
    """唯一入口。LOCAL 改顶部 MODE；REMOTE 由 F407 发 CMD 来切换。"""
    sensor = uart = canvas = None
    media_started = display_started = False
    try:
        allowed_modes = ("IDLE", "PREVIEW", "QR", "BOMB", "TARGET", "RESCUE")
        if RUN_MODE not in ("LOCAL", "REMOTE") or MODE not in allowed_modes:
            raise ValueError("RUN_MODE 或 MODE 配置错误")
        if WIDTH % 16 or WIDTH > 640 or HEIGHT > 480:
            raise ValueError("初版使用 640×480 以内且宽度为 16 倍数")
        if RUN_MODE == "LOCAL" and MODE in ("BOMB", "TARGET", "RESCUE") and parse_task(TASK) is None:
            raise ValueError("TASK 需要三位 1/2/3，例如 211")
        if UART_ENABLED and (not isinstance(UART_TX_GPIO, int) or not isinstance(UART_RX_GPIO, int)):
            raise ValueError("先核对 UART GPIO 再打开 UART_ENABLED")

        os.exitpoint(os.EXITPOINT_ENABLE)
        print("K230:", os.uname())
        print("先用 LOCAL + PREVIEW 确认实时画面。")

        if UART_ENABLED:
            fpioa = FPIOA()
            fpioa.set_function(UART_TX_GPIO, FPIOA.UART2_TXD)
            fpioa.set_function(UART_RX_GPIO, FPIOA.UART2_RXD)
            uart = UART(UART.UART2, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS,
                        parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE, timeout=0)

        sensor = Sensor(id=SENSOR_ID, fps=30)
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        sensor.set_hmirror(HMIRROR)
        sensor.set_vflip(VFLIP)

        if DISPLAY_MODE != "NONE":
            display_started = True
            if DISPLAY_MODE == "VIRT":
                Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, fps=30)
            elif DISPLAY_MODE == "LCD":
                Display.init(Display.ST7701, width=800, height=480, to_ide=True)
            else:
                raise ValueError("DISPLAY_MODE 必须是 VIRT/LCD/NONE")
            canvas = image.Image(WIDTH, HEIGHT, image.ARGB8888)

        media_started = True
        MediaManager.init()
        sensor.run()

        mode = MODE if RUN_MODE == "LOCAL" else "IDLE"
        task = parse_task(TASK) if RUN_MODE == "LOCAL" else None
        rid, seq = 0, 0
        now = time.ticks_ms()
        settle_at = last_contact = last_frame = last_send = last_log = now
        buffer = {"data": bytearray(), "time": now}
        previous, count = None, 0

        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # F407 指令：$V1,CMD,请求号,QR/BOMB/TARGET/RESCUE/IDLE/RESET*CRC\r\n
            if uart is not None:
                data = uart.read(256)
                if data:
                    for fields in read_packets(buffer, data, now):
                        if len(fields) == 3 and fields[0] == "CMD":
                            try:
                                new_rid, new_mode = int(fields[1]), fields[2]
                                if not 0 <= new_rid <= 65535 or new_mode not in allowed_modes + ("RESET",):
                                    raise ValueError
                                if new_mode in ("BOMB", "TARGET", "RESCUE") and task is None:
                                    uart.write(make_packet("ERR", new_rid, "NO_TASK"))
                                    continue
                                rid = new_rid
                                mode = "IDLE" if new_mode == "RESET" else new_mode
                                if new_mode == "RESET":
                                    task = None
                                settle_at = last_contact = now
                                previous, count = None, 0
                                uart.write(make_packet("ACK", rid, new_mode, "OK"))
                            except ValueError:
                                uart.write(make_packet("ERR", 0, "BAD_COMMAND"))
                        elif len(fields) == 3 and fields[0] == "PING" and fields[2] == "KEEP":
                            try:
                                if int(fields[1]) == rid:
                                    last_contact = now
                                    uart.write(make_packet("PONG", rid, "OK"))
                            except ValueError:
                                pass

            if RUN_MODE == "REMOTE" and time.ticks_diff(now, last_contact) > LINK_TIMEOUT_MS:
                mode, previous, count = "IDLE", None, 0

            raw = sensor.snapshot(chn=CAM_CHN_ID_0)
            gap = time.ticks_diff(now, last_frame)
            last_frame = now
            target, widths = None, None
            status = "IDLE" if mode == "IDLE" else "PREVIEW"

            if mode in ("QR", "BOMB", "TARGET", "RESCUE"):
                if time.ticks_diff(now, settle_at) < SETTLE_MS:
                    status = "SETTLING"
                elif mode == "QR":
                    target, status = detect_qr(raw), "NONE"
                    if target:
                        status = "FOUND"
                elif task is None:
                    status = "NO_TASK"
                elif mode == "BOMB":
                    target, status = detect_ball(raw, task[0]), "NONE"
                    if target:
                        status = "FOUND"
                elif mode == "TARGET":
                    target, status = detect_target(raw, task[1]), "NONE"
                    if target:
                        status = "FOUND"
                else:
                    target, widths = detect_rescue(raw, task[2])
                    status = "FOUND" if target else "UNKNOWN"

            if target and gap <= MAX_FRAME_GAP_MS:
                count = count + 1 if stable(previous, target) else 1
                previous = target
                if count >= STABLE_FRAMES:
                    status = "VALID"
                    if mode == "QR":
                        task = target[0]
                        target = (0, target[1], target[2], target[3], target[4])
            else:
                previous, count = None, 0
                if gap > MAX_FRAME_GAP_MS:
                    status = "SLOW"

            seq = (seq + 1) & 0xFFFF
            if status == "VALID" and target:
                kind, x, y, w, h = target
                dx, dy = x - WIDTH // 2, y - HEIGHT // 2
            else:
                kind, x, y, w, h, dx, dy = 0, -1, -1, 0, 0, 0, 0
            code = "".join(str(value) for value in task) if task else "000"
            report = ("OBS", rid, seq, "R" if RUN_MODE == "REMOTE" else "L", mode, status,
                      kind, x, y, w, h, dx, dy, code, count, WIDTH, HEIGHT)

            if time.ticks_diff(now, last_send) >= SEND_INTERVAL_MS:
                if uart is not None:
                    uart.write(make_packet(*report))
                last_send = now
            if time.ticks_diff(now, last_log) >= 500:
                print(make_packet(*report).decode().strip())
                if widths:
                    print("白色物体五段宽度:", widths)
                last_log = now
            if canvas is not None:
                show(canvas, raw, mode, status, target, task, 1000 / max(1, gap))
            del raw
            if seq % 30 == 0:
                gc.collect()
            time.sleep_ms(1)

    except KeyboardInterrupt:
        print("用户停止")
    except Exception as error:
        print("程序异常:", error)
        if hasattr(sys, "print_exception"):
            sys.print_exception(error)
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except Exception:
                pass
        if display_started:
            try:
                Display.deinit()
            except Exception:
                pass
        if media_started:
            try:
                MediaManager.deinit()
            except Exception:
                pass
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass


if __name__ == "__main__":
    main()
