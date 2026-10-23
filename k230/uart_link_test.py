"""K230D <-> F407 UART text-link smoke test.

Upload and run this file alone in CanMV before joining it with camera code.
It accepts the agreed 115200/8N1 text commands and returns deterministic
LOST/NONE observations, so transport is proven separately from recognition.
"""

import time
from machine import UART, FPIOA


# K230D UART2 reference pins.  These must match the physical K230D pins wired
# to F407 PD6 (K230 TX) and PD5 (K230 RX); do not connect any 5 V signal.
UART_TX_GPIO = 11
UART_RX_GPIO = 12
UART_BAUDRATE = 115200
SEND_INTERVAL_MS = 100
BOOT_TX_PROBE_INTERVAL_MS = 500


def configure_uart():
    """Create the 3.3 V UART2 link used by the F407 visual-link test."""
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_GPIO, FPIOA.UART2_RXD)
    return UART(UART.UART2, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE, timeout=0)


def send_line(uart, text):
    """Send the project's canonical CRLF-terminated text frame."""
    uart.write((text + "\r\n").encode("ascii"))


def handle_line(uart, line):
    """Apply one MODE command and return the active mode, or None if invalid."""
    fields = line.split(",")
    if len(fields) != 2 or fields[0] != "MODE":
        return None
    if fields[1] == "TRACK":
        send_line(uart, "ACK,TRACK")
        return "TRACK"
    if fields[1] == "RECOG":
        send_line(uart, "ACK,RECOG")
        return "RECOG"
    return None


def main():
    uart = configure_uart()
    line = bytearray()
    mode = None
    sequence = 0
    last_report = time.ticks_ms()
    last_boot_probe = last_report
    print("UART link test ready: MODE,TRACK or MODE,RECOG")

    while True:
        data = uart.read(64)
        if data:
            for byte in data:
                if byte == 10:  # newline
                    try:
                        text = bytes(line).decode("ascii").rstrip("\r\n")
                    except UnicodeError:
                        text = ""
                    line = bytearray()
                    selected = handle_line(uart, text)
                    if selected is not None:
                        mode = selected
                        last_report = time.ticks_ms() - SEND_INTERVAL_MS
                elif byte == 13:  # CR in the canonical CRLF terminator
                    pass
                elif 32 <= byte <= 126 and len(line) < 47:
                    line.append(byte)
                else:
                    line = bytearray()

        now = time.ticks_ms()
        # A one-way physical-link probe: it proves K230 TX -> F407 PD6 even
        # when F407 TX -> K230 RX is not yet working.  This exists only in
        # the standalone link-test script, never in the recognition program.
        if mode is None and time.ticks_diff(now, last_boot_probe) >= BOOT_TX_PROBE_INTERVAL_MS:
            send_line(uart, "RECOG,0,NONE")
            last_boot_probe = now
        if mode is not None and time.ticks_diff(now, last_report) >= SEND_INTERVAL_MS:
            sequence = (sequence + 1) & 0xFFFF
            if mode == "TRACK":
                send_line(uart, "TRACK,%d,LOST" % sequence)
            else:
                send_line(uart, "RECOG,%d,NONE" % sequence)
            last_report = now
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
