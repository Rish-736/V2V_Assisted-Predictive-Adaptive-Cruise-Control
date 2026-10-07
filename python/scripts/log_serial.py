#!/usr/bin/env python3
"""
log_serial.py -- capture a node's serial output to a file.

Used for two things:
  * capturing step-response data for sysid_fit.py
  * recording a telemetry run so it can be replayed into the dashboard
    later (invaluable: if the hardware misbehaves on review day you can
    still show a real recorded run)

    # list ports
    python python/scripts/log_serial.py --list

    # capture a step-response run
    python python/scripts/log_serial.py --out data/step.csv --duration 45

    # capture telemetry from both nodes at once
    python python/scripts/log_serial.py --port COM5 --out data/follow.csv &
    python python/scripts/log_serial.py --port COM6 --out data/lead.csv
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from v2vacc.serial_link import list_ports, autodetect_port  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", type=Path, default=None, help="output file")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many seconds (0 = until Ctrl-C)")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--quiet", action="store_true", help="do not echo to the terminal")
    args = ap.parse_args()

    ports = list_ports()
    if args.list:
        if not ports:
            print("no serial ports found (is pyserial installed? is the board plugged in?)")
        for dev, desc in ports:
            print(f"  {dev:<12} {desc}")
        return 0

    try:
        import serial
    except ImportError:
        print("pyserial is required:  pip install -r python/requirements.txt",
              file=sys.stderr)
        return 2

    port = args.port or autodetect_port()
    if port is None:
        print("no serial port found. Plug in a board, or pass --port.", file=sys.stderr)
        return 2
    print(f"# port     : {port} @ {args.baud}")

    fh = None
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        fh = args.out.open("w", encoding="utf-8", newline="")
        print(f"# writing  : {args.out}")
    if args.duration:
        print(f"# duration : {args.duration:g} s")
    print("# Ctrl-C to stop")
    print("-" * 60)

    n, t0 = 0, time.monotonic()
    try:
        with serial.Serial(port, args.baud, timeout=0.5) as ser:
            time.sleep(0.2)
            ser.reset_input_buffer()
            while True:
                if args.duration and (time.monotonic() - t0) >= args.duration:
                    break
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                n += 1
                if fh:
                    fh.write(line + "\n")
                    if n % 100 == 0:
                        fh.flush()
                if not args.quiet:
                    print(line)
    except KeyboardInterrupt:
        print("\n# stopped by user")
    except Exception as exc:
        print(f"\n# serial error: {exc}", file=sys.stderr)
        return 1
    finally:
        if fh:
            fh.close()

    dt = time.monotonic() - t0
    print(f"# captured {n} lines in {dt:.1f} s ({n/max(dt, 1e-6):.0f} lines/s)")
    if args.out:
        print(f"# saved -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
