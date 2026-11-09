#!/usr/bin/env python3
"""
Talk to the boat's CM7 serial console: the simulator or the real board, same commands.

  python3 sim/tools/board.py                          the simulator (Renode, port 3456)
  python3 sim/tools/board.py /dev/tty.usbmodem1203    the real NUCLEO board over its USB cable (115200 baud)
  python3 sim/tools/board.py --quiet                  hide the sensor printouts, show only console replies

Type a command and press Enter:  tail 30 | rudder -10 | center | sweep tail | sweep rudder | status | help
Quit with Ctrl-C. (Needs CM7_CONSOLE_ENABLE in the firmware; the simulator build has it on.)
"""
import argparse
import os
import select
import socket
import sys
import termios
import tty


def open_serial(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return (lambda b: os.write(fd, b)), (lambda: os.read(fd, 4096)), fd


def open_sim(port):
    s = socket.create_connection(("127.0.0.1", port), timeout=3)
    s.setblocking(False)
    return s.sendall, (lambda: s.recv(4096)), s.fileno()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("device", nargs="?", help="serial device of the real board (default: the simulator)")
    ap.add_argument("--port", type=int, default=3456, help="simulator console port")
    ap.add_argument("--quiet", action="store_true", help="only show [CONSOLE] replies and servo moves")
    args = ap.parse_args()
    try:
        write, read, fd = open_serial(args.device) if args.device else open_sim(args.port)
    except OSError as e:
        where = args.device or f"the simulator on port {args.port}"
        sys.exit(f"Could not open {where}: {e}\n(Simulator: start the dashboard and click 'Run my firmware'.)")
    print(f"Connected to {'the board at ' + args.device if args.device else 'the simulated board'}. "
          "Commands: tail <deg>, rudder <deg>, center, sweep tail|rudder, status, help. Ctrl-C to quit.")
    buf = b""
    try:
        while True:
            r, _, _ = select.select([sys.stdin, fd], [], [])
            if sys.stdin in r:
                line = sys.stdin.readline()
                if not line:
                    break
                write(line.rstrip("\n").encode() + b"\r")
            if fd in r:
                try:
                    data = read()
                except (BlockingIOError, InterruptedError):
                    continue
                if not data:
                    print("\n(connection closed)")
                    break
                buf += data
                *lines, buf = buf.split(b"\n")
                for l in lines:
                    text = l.decode(errors="replace").rstrip("\r")
                    if not args.quiet or "[CONSOLE]" in text or "Setting Servo" in text:
                        print(text)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
