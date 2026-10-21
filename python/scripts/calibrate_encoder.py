#!/usr/bin/env python3
"""
calibrate_encoder.py -- work out the virtual-drivetrain scale factor.

WHY THIS EXISTS
---------------
A bare DC motor on the bench has no gearbox and no wheel, so "speed in m/s"
has to be defined by a scale factor. We model a plausible drivetrain -- a
reduction onto a 65 mm wheel -- and convert measured shaft pulses into a
vehicle speed through it.

This is a UNIT CHOICE, not a fudge. The shaft speed is genuinely measured;
this just maps it onto a vehicle speed range where a 1.5 s time headway and
a 0.8 m gap mean something. Say exactly that if asked.

HOW TO USE IT
-------------
 1. Wire the sensor, set ENC_PPR in config.h to the number of black sectors
    on your disc, and flash firmware/tools/step_response.
 2. Capture a run:
       python python/scripts/log_serial.py --port COM5 --out data/step.csv --duration 45
 3. Either let this script read the file:
       python python/scripts/calibrate_encoder.py --csv data/step.csv
    or pass the peak pulse rate you saw by hand:
       python python/scripts/calibrate_encoder.py --pulses-per-sec 850

It prints the ENC_GEAR_RATIO to paste into config.h, and tells you whether
your sector count gives enough resolution to control on.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def peak_pulse_rate(csv_path: Path) -> tuple[float, float]:
    """Return (peak pulses/sec, the duty it occurred at) from a step capture.

    Uses the 95th percentile rather than the single maximum, because one
    spurious double-count would otherwise set the entire scale factor.
    """
    rows = []
    header = None
    for line in csv_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if header is None:
            if parts[0].lower().startswith("t_ms"):
                header = [p.lower() for p in parts]
                continue
            header = ["t_ms", "duty", "speed_mps", "enc_count",
                      "pulses_per_sec", "tag"]
        rec = dict(zip(header, parts))
        try:
            pps = float(rec["pulses_per_sec"])
            duty = float(rec["duty"])
        except (KeyError, ValueError):
            continue
        if duty > 0.5:                       # only the high-duty segments
            rows.append((pps, duty))
    if not rows:
        raise SystemExit(
            "No pulses_per_sec column found.\n"
            "Re-flash firmware/tools/step_response (it logs that column now),\n"
            "or pass the rate by hand with --pulses-per-sec.")
    rows.sort(key=lambda r: r[0])
    pick = rows[int(0.95 * (len(rows) - 1))]
    return pick


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--csv", type=Path, help="step-response capture")
    src.add_argument("--pulses-per-sec", type=float,
                     help="peak pulse rate at full duty, measured by hand")
    ap.add_argument("--ppr", type=float, default=12.0,
                    help="black sectors on the disc (= ENC_PPR), default 12")
    ap.add_argument("--wheel", type=float, default=0.065,
                    help="virtual wheel diameter [m], default 0.065")
    ap.add_argument("--v-max", type=float, default=0.60,
                    help="vehicle speed full duty should map to [m/s]")
    ap.add_argument("--ctrl-hz", type=float, default=50.0)
    ap.add_argument("--window-ticks", type=int, default=3,
                    help="ENC_SPEED_WINDOW_TICKS from config.h")
    args = ap.parse_args()

    if args.csv:
        pps, duty = peak_pulse_rate(args.csv)
        print(f"read {args.csv}")
        print(f"  peak pulse rate (95th pct) : {pps:.1f} pulses/s at duty {duty:.2f}")
        if duty < 0.95:
            pps = pps / duty                 # extrapolate to full duty
            print(f"  extrapolated to full duty  : {pps:.1f} pulses/s")
    else:
        pps = args.pulses_per_sec
        print(f"using the rate you supplied: {pps:.1f} pulses/s at full duty")

    rev_per_s = pps / args.ppr
    rpm = rev_per_s * 60.0
    print()
    print(f"  sectors on the disc        : {args.ppr:.0f}")
    print(f"  shaft speed                : {rev_per_s:.1f} rev/s  ({rpm:.0f} rpm)")

    # --- resolution check -------------------------------------------
    window_s = args.window_ticks / args.ctrl_hz
    per_window = pps * window_s
    quant_pct = 100.0 / per_window if per_window > 0 else 999.0
    print()
    print(f"  measurement window         : {window_s*1000:.0f} ms "
          f"({args.window_ticks} ticks at {args.ctrl_hz:.0f} Hz)")
    print(f"  pulses per window          : {per_window:.1f}")
    print(f"  speed quantisation         : {quant_pct:.1f}% of full speed")
    if per_window >= 15:
        print("  -> good. Plenty of resolution to control on.")
    elif per_window >= 8:
        print("  -> usable, but marginal. Add more sectors to the disc, or")
        print("     raise ENC_SPEED_WINDOW_TICKS to 4-5 (costs a little lag).")
    else:
        print("  -> TOO COARSE. The controller would be regulating noise.")
        need = math.ceil(15.0 / (rev_per_s * window_s))
        print(f"     Use at least {need:.0f} sectors, or raise the window to "
              f"{math.ceil(15.0/(pps/args.ctrl_hz)):.0f} ticks.")

    # --- the scale factor --------------------------------------------
    #  v_max = (pps / (PPR * GEAR)) * pi * D   ->  solve for GEAR
    circ = math.pi * args.wheel
    gear = (pps * circ) / (args.ppr * args.v_max)
    print()
    print("=" * 62)
    print("PASTE INTO firmware/common/config.h SECTION 10")
    print("=" * 62)
    print(f"  #define ENC_PPR                {args.ppr:.1f}f")
    print(f"  #define ENC_GEAR_RATIO         {gear:.1f}f")
    print(f"  #define WHEEL_DIAMETER_M       {args.wheel:.3f}f")
    print()
    print("then:  python tools/sync_common.py")
    print()
    print(f"With these, full duty reads as {args.v_max:.2f} m/s, and one pulse")
    print(f"is {circ/(args.ppr*gear)*1000:.3f} mm of virtual wheel travel.")

    print()
    print("SANITY CHECK after flashing: spin the motor at full duty and")
    print(f"confirm the telemetry reports close to {args.v_max:.2f} m/s. If it")
    print("reads about half, you are counting one edge per sector instead of")
    print("two (or vice versa) -- halve or double ENC_PPR to match.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
