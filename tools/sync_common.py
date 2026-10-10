#!/usr/bin/env python3
"""
sync_common.py -- propagate firmware/common/*.h into each Arduino sketch folder.

WHY THIS EXISTS
---------------
The Arduino IDE copies a sketch folder into a temporary build directory
before compiling, which breaks relative includes like "../common/config.h".
The portable fix is for each sketch folder to hold its own copy of the
shared headers.

To avoid the classic "edited the wrong copy" bug, firmware/common/ is the
single source of truth and the copies are marked GENERATED. Run this after
touching anything in firmware/common/.

    python tools/sync_common.py            # copy
    python tools/sync_common.py --check    # verify copies are current (CI)

(If you use PlatformIO instead, see platformio.ini -- it points at
firmware/common/ directly and this script is unnecessary.)
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMON = ROOT / "firmware" / "common"

# which shared headers each sketch folder needs
TARGETS = {
    ROOT / "firmware" / "lead_node": [
        "config.h", "v2v_protocol.h", "filters.h", "pid.h", "vehicle_io.h",
        "scenario.h", "hmi.h",
    ],
    ROOT / "firmware" / "follow_node": [
        "config.h", "v2v_protocol.h", "filters.h", "pid.h", "vehicle_io.h",
        "scenario.h", "hmi.h",
    ],
    ROOT / "firmware" / "tools" / "step_response": [
        "config.h", "filters.h",
    ],
}

BANNER = (
    "// >>> GENERATED FILE -- DO NOT EDIT <<<\n"
    "// Source of truth: firmware/common/{name}\n"
    "// Regenerate with: python tools/sync_common.py\n"
)


def rendered(src: Path) -> str:
    return BANNER.format(name=src.name) + src.read_text(encoding="utf-8")


def digest(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if any copy is stale (do not write)")
    args = ap.parse_args()

    if not COMMON.is_dir():
        print(f"error: {COMMON} not found", file=sys.stderr)
        return 2

    stale: list[Path] = []
    written = 0

    for sketch_dir, names in TARGETS.items():
        sketch_dir.mkdir(parents=True, exist_ok=True)
        for name in names:
            src = COMMON / name
            if not src.is_file():
                print(f"error: missing {src}", file=sys.stderr)
                return 2

            dst = sketch_dir / name
            want = rendered(src)
            have = dst.read_text(encoding="utf-8") if dst.is_file() else None

            if have is not None and digest(have) == digest(want):
                continue

            rel = dst.relative_to(ROOT)
            if args.check:
                stale.append(rel)
            else:
                dst.write_text(want, encoding="utf-8")
                print(f"  updated  {rel}")
                written += 1

    if args.check:
        if stale:
            print("stale copies (run: python tools/sync_common.py):")
            for p in stale:
                print(f"  {p}")
            return 1
        print("all sketch copies are up to date")
        return 0

    print(f"sync complete -- {written} file(s) updated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
