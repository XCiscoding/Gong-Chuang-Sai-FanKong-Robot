#!/usr/bin/env python3
"""Static gate for K230 scripts that speak the F407 TEXT TRACK protocol.

It deliberately checks the narrow transport contract, not camera logic.  A
passing result means the source uses the frozen pins, 115200/8N1, CRLF sender,
MODE gate, ACK and TRACK/LOST forms.  It is not a physical-link result.
"""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import sys


REQUIRED_SNIPPETS = {
    "FPIOA UART2 TX mapping": "fpioa.set_function(UART_TX_GPIO, FPIOA.UART2_TXD)",
    "FPIOA UART2 RX mapping": "fpioa.set_function(UART_RX_GPIO, FPIOA.UART2_RXD)",
    "UART2 instance": "UART(UART.UART2",
    "8 data bits": "bits=UART.EIGHTBITS",
    "no parity": "parity=UART.PARITY_NONE",
    "one stop bit": "stop=UART.STOPBITS_ONE",
    "exact TRACK mode": 'text == "MODE,TRACK"',
    "TRACK acknowledgement": 'send_line(uart, "ACK,TRACK")',
    "valid TRACK frame": 'send_line(uart, "TRACK,%d,%d,0,%d"',
    "LOST TRACK frame": 'send_line(uart, "TRACK,%d,LOST"',
}


class UartWriteVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.function_stack: list[str] = []
        self.direct_writes: list[int] = []

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self.function_stack.append(node.name)
        self.generic_visit(node)
        self.function_stack.pop()

    def visit_Call(self, node: ast.Call) -> None:
        if isinstance(node.func, ast.Attribute) and node.func.attr == "write":
            if not self.function_stack or self.function_stack[-1] != "send_line":
                self.direct_writes.append(node.lineno)
        self.generic_visit(node)


def check_source(path: Path) -> list[str]:
    failures: list[str] = []
    try:
        source = path.read_text(encoding="utf-8")
    except OSError as error:
        return [f"cannot read: {error}"]

    for name, expected in REQUIRED_SNIPPETS.items():
        if expected not in source:
            failures.append(f"missing {name}: {expected}")

    for name, expected in (
        ("GPIO11 TX", "UART_TX_GPIO = 11"),
        ("GPIO12 RX", "UART_RX_GPIO = 12"),
        ("115200 baud", "UART_BAUDRATE = 115200"),
        ("CRLF sender", 'text + "\\r\\n"'),
    ):
        if expected not in source:
            failures.append(f"missing {name}: {expected}")

    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError as error:
        return failures + [f"Python syntax error: {error.msg} at line {error.lineno}"]

    visitor = UartWriteVisitor()
    visitor.visit(tree)
    if visitor.direct_writes:
        lines = ", ".join(str(line) for line in visitor.direct_writes)
        failures.append(f"uart.write outside send_line at line(s): {lines}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", action="append", required=True,
                        help="K230 TRACK script to inspect; repeat for each source")
    args = parser.parse_args()

    failed = False
    for raw_path in args.source:
        path = Path(raw_path)
        failures = check_source(path)
        if failures:
            failed = True
            print(f"FAIL k230_track_protocol source={path}")
            for failure in failures:
                print(f"  - {failure}")
        else:
            print(f"PASS k230_track_protocol source={path}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
