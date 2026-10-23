# -*- coding: utf-8 -*-
"""K230D/GC2093 line tracking over the project's proven text UART protocol.

F407 -> K230D::

    MODE,TRACK\r\n

K230D -> F407::

    ACK,TRACK\r\n
    TRACK,<seq>,<dx>,0,<quality>\r\n
    TRACK,<seq>,LOST\r\n

This file intentionally keeps ``uart_link_test.py`` unchanged as the known-good
transport probe.  It reuses that probe's UART2/GPIO/text framing and adds only
the narrow-ROI line detector derived from the passed line-follow reference.
"""

import gc
import os
import sys
import time

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0, Sensor


# Camera/display baseline from this project's main.py.
SENSOR_ID = 2
WIDTH = 320
HEIGHT = 240
DISPLAY_MODE = "LCD"  # LCD=3.1-inch display + IDE preview; VIRT=IDE only; NONE=headless.
HMIRROR = False
VFLIP = False

# Known-good physical/text link from uart_link_test.py.
UART_TX_GPIO = 11
UART_RX_GPIO = 12
UART_BAUDRATE = 115200
SEND_INTERVAL_MS = 100

# Line detector. BAND is only a starting point and must be measured on-site.
BAND = (36, 60, -3, 35, -36, 25)
# Use a wider lower-middle band so a line remains measurable at the normal
# mounting height.  The old 20-pixel band plus a 1000-pixel minimum only
# accepted a line when the camera was very close to the floor.
BAND_Y = HEIGHT // 2
BAND_H = HEIGHT // 3
ROI = (0, BAND_Y, WIDTH, BAND_H)
MIN_PIXELS = 120
MIN_AREA = 160
FILTER_LEN = 5
JUMP_LIMIT = WIDTH // 8
STABLE_SPREAD = WIDTH // 32
LOST_RESET_FRAMES = 3


def configure_uart():
    """Create the same UART2 link that already passed the transport test."""
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_GPIO, FPIOA.UART2_RXD)
    return UART(UART.UART2, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE, timeout=0)


def send_line(uart, text):
    """Send the canonical CRLF-terminated text frame."""
    uart.write((text + "\r\n").encode("ascii"))


def consume_mode_commands(uart, line_buffer, data):
    """Consume fragmented/newline-joined input; return (buffer, reset_session)."""
    reset_session = False
    for byte in data:
        if byte == 10:  # \n
            try:
                text = bytes(line_buffer).decode("ascii").rstrip("\r\n")
            except UnicodeError:
                text = ""
            line_buffer = bytearray()
            if text == "MODE,TRACK":
                send_line(uart, "ACK,TRACK")
                reset_session = True
        elif byte == 13:  # CR in the canonical CRLF terminator
            pass
        elif 32 <= byte <= 126 and len(line_buffer) < 47:
            line_buffer.append(byte)
        else:
            line_buffer = bytearray()
    return line_buffer, reset_session


def reset_detector(state):
    state["history"] = []
    state["last_detected_x"] = None
    state["lost_count"] = 0


def detect_line(raw, state):
    """Return a report dictionary; never reuses an old dx on a bad frame."""
    blobs = raw.find_blobs([BAND], roi=ROI, pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_AREA, merge=False,
                           x_stride=2, y_stride=1)
    # Match the passed algorithm: reject candidates touching left/right image edge.
    blobs = [blob for blob in blobs
             if blob.x() > 0 and blob.x() + blob.w() < WIDTH]

    selected = None
    jumped = False
    if blobs:
        selected = max(blobs, key=lambda blob: blob.w() * blob.h())
        center_x = selected.cx()
        jumped = (state["last_detected_x"] is not None and
                  abs(center_x - state["last_detected_x"]) > JUMP_LIMIT)
        # The reference follows the newest detection even when rejecting one jump,
        # otherwise every later frame can remain far from a stale baseline.
        state["last_detected_x"] = center_x
    else:
        center_x = None

    if center_x is None or jumped:
        state["history"] = []
        state["lost_count"] += 1
        if state["lost_count"] >= LOST_RESET_FRAMES:
            state["last_detected_x"] = None
        return {
            "status": "LOST", "dx": 0, "quality": 0, "stable": 0,
            "filtered_x": None, "blob": selected, "jumped": jumped,
        }

    state["lost_count"] = 0
    state["history"].append(center_x)
    if len(state["history"]) > FILTER_LEN:
        state["history"].pop(0)
    filtered_x = sum(state["history"]) / float(len(state["history"]))
    stable = (len(state["history"]) == FILTER_LEN and
              max(state["history"]) - min(state["history"]) <= STABLE_SPREAD)
    dx = int(round((filtered_x - WIDTH / 2.0) * (1280.0 / WIDTH)))
    return {
        "status": "VALID", "dx": dx, "quality": 100 if stable else 50,
        "stable": 1 if stable else 0, "filtered_x": filtered_x,
        "blob": selected, "jumped": False,
    }


def draw_preview(raw, report, active, sequence, fps):
    # Draw directly on the captured RGB565 frame.  An ARGB overlay canvas can
    # remain transparent on ST7701 and hide the camera image while still
    # showing its rectangles/text.
    raw.draw_rectangle(ROI[0], ROI[1], ROI[2], ROI[3], color=(255, 255, 255))
    raw.draw_line(WIDTH // 2, 0, WIDTH // 2, HEIGHT - 1, color=(255, 0, 0))
    blob = report["blob"]
    if blob is not None:
        color = (255, 255, 0) if report["jumped"] else (0, 255, 0)
        raw.draw_rectangle(blob.x(), blob.y(), blob.w(), blob.h(), color=color)
    if report["filtered_x"] is not None:
        raw.draw_cross(int(round(report["filtered_x"])),
                       ROI[1] + ROI[3] // 2, color=(0, 255, 0), size=8)
    link = "ACTIVE" if active else "WAIT MODE,TRACK"
    raw.draw_string_advanced(4, 4, 16, "%s %s" % (link, report["status"]),
                             color=(255, 255, 0))
    raw.draw_string_advanced(
        4, 24, 14, "seq:%d dx:%d q:%d fps:%.1f" %
        (sequence, report["dx"], report["quality"], fps),
        color=(255, 255, 0))
    Display.show_image(raw)


def main():
    sensor = uart = None
    media_started = display_started = False
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        uart = configure_uart()

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
                raise ValueError("DISPLAY_MODE must be VIRT/LCD/NONE")

        media_started = True
        MediaManager.init()
        sensor.run()

        line_buffer = bytearray()
        active = False
        sequence = 0
        frame_count = 0
        detector = {"history": [], "last_detected_x": None, "lost_count": 0}
        last_send = time.ticks_ms()
        last_frame = last_send
        report = {"status": "LOST", "dx": 0, "quality": 0, "stable": 0,
                  "filtered_x": None, "blob": None, "jumped": False}
        print("TRACK text protocol ready; waiting for MODE,TRACK")

        while True:
            os.exitpoint()
            now = time.ticks_ms()
            data = uart.read(64)
            if data:
                line_buffer, reset_session = consume_mode_commands(
                    uart, line_buffer, data)
                if reset_session:
                    active = True
                    sequence = 0
                    reset_detector(detector)
                    last_send = now - SEND_INTERVAL_MS

            raw = sensor.snapshot(chn=CAM_CHN_ID_0)
            frame_count += 1
            frame_gap = time.ticks_diff(now, last_frame)
            last_frame = now
            report = detect_line(raw, detector)

            if active and time.ticks_diff(now, last_send) >= SEND_INTERVAL_MS:
                sequence = (sequence + 1) & 0xFFFF
                if report["status"] == "VALID":
                    send_line(uart, "TRACK,%d,%d,0,%d" %
                              (sequence, report["dx"], report["quality"]))
                else:
                    send_line(uart, "TRACK,%d,LOST" % sequence)
                print("%d,%s,%s,q%d,s%d" %
                      (sequence, report["status"],
                       str(report["dx"]) if report["status"] == "VALID" else "NA",
                       report["quality"], report["stable"]))
                last_send = now

            if DISPLAY_MODE != "NONE":
                draw_preview(raw, report, active, sequence,
                             1000.0 / max(1, frame_gap))
            del raw
            if frame_count % 30 == 0:
                gc.collect()
            time.sleep_ms(1)

    except KeyboardInterrupt:
        print("stopped")
    except Exception as error:
        print("fatal:", error)
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
