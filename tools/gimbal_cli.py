#!/usr/bin/env python3
"""Interactive and scripted USART2 console for the Armtrack gimbal."""

from __future__ import annotations

import argparse
import sys
import threading
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: python -m pip install -r requirements.txt") from exc

from gimbal_protocol import command_tick, describe_line


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{value:02X}" for value in data)


class Console:
    def __init__(self, port: serial.Serial, show_hex: bool) -> None:
        self.port = port
        self.show_hex = show_hex
        self.stop_event = threading.Event()
        self.buffer = bytearray()
        self.thread = threading.Thread(target=self._reader, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def send(self, command: str) -> None:
        raw = (command.strip() + "\r\n").encode("ascii")
        self.port.write(raw)
        self.port.flush()
        print(f"[TX] {command.strip()}", flush=True)

    def _reader(self) -> None:
        while not self.stop_event.is_set():
            try:
                data = self.port.read(self.port.in_waiting or 1)
            except (OSError, serial.SerialException) as exc:
                print(f"[SERIAL ERROR] {exc}", file=sys.stderr, flush=True)
                return
            if not data:
                continue
            if self.show_hex:
                print(f"[RX HEX] {hex_bytes(data)}", flush=True)
            self.buffer.extend(data)
            while True:
                indexes = [self.buffer.find(b"\r"), self.buffer.find(b"\n")]
                indexes = [index for index in indexes if index >= 0]
                if not indexes:
                    break
                index = min(indexes)
                line = bytes(self.buffer[:index]).decode("ascii", errors="replace")
                del self.buffer[:index + 1]
                while self.buffer[:1] in (b"\r", b"\n"):
                    del self.buffer[:1]
                if line:
                    print(f"[RX] {describe_line(line)}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM34")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--command", action="append", default=[])
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--no-hex", action="store_true")
    parser.add_argument("--heartbeat", action="store_true", help="send tick heartbeat once per second")
    args = parser.parse_args()
    if args.baud <= 0 or args.duration < 0:
        parser.error("--baud must be positive and --duration must be non-negative")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=0.5) as port:
            console = Console(port, not args.no_hex)
            console.start()
            try:
                for command in args.command:
                    console.send(command)
                    time.sleep(0.05)
                if args.duration > 0:
                    deadline = time.monotonic() + args.duration
                    sequence = 0
                    while time.monotonic() < deadline:
                        if args.heartbeat:
                            sequence += 1
                            console.send(command_tick(sequence))
                        time.sleep(1.0 if args.heartbeat else min(0.1, max(0.0, deadline - time.monotonic())))
                elif not args.command or sys.stdin.isatty():
                    print("Type commands; Ctrl+C exits.")
                    while not console.stop_event.is_set():
                        try:
                            command = input("gimbal> ")
                        except EOFError:
                            break
                        if command.strip():
                            console.send(command)
            except KeyboardInterrupt:
                pass
            finally:
                console.stop()
    except (OSError, serial.SerialException) as exc:
        print(f"Unable to open {args.port}: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
