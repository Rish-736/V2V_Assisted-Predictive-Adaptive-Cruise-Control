"""
telemetry.py -- parse the serial telemetry emitted by the two nodes.

WIRE FORMAT
-----------
Plain CSV, one record per line, first field is a single-letter record
type. Chosen over a binary protocol on purpose: you can debug the whole
system with nothing but the Arduino serial monitor, which matters when
something breaks the night before a review.

  Follower:
    F,t_ms,d,d_des,closing,ttc,v,v_tgt,v_lead,duty,mode,link,brake,
      p,i,dterm,pkt_good,pkt_lost,pkt_bad

  Lead:
    L,t_ms,v,v_tgt,accel,duty,braking,hazard,seq,tx_ok,tx_fail

Lines beginning with '#' are human-readable banner/comment lines and are
returned as CommentRecord so the dashboard can show them without
choking.
"""

from __future__ import annotations

import csv
import io
from dataclasses import dataclass, asdict, fields
from pathlib import Path

__all__ = [
    "FollowerRecord", "LeadRecord", "CommentRecord",
    "parse_line", "parse_file", "records_to_csv",
    "MODE_NAMES", "LINK_NAMES",
]

MODE_NAMES = ["STANDBY", "CRUISE", "FOLLOW", "PREDICT", "EMERG", "FAULT"]
LINK_NAMES = ["OK", "DEGRADED", "LOST"]


# ---------------------------------------------------------------------
@dataclass
class FollowerRecord:
    t_ms: int
    d: float            # filtered gap [m]
    d_des: float        # desired gap from the time-gap policy [m]
    closing: float      # closing rate [m/s], positive = gap shrinking
    ttc: float          # time to contact [s]
    v: float            # own speed [m/s]
    v_tgt: float        # outer-loop speed setpoint [m/s]
    v_lead: float       # lead speed (V2V, or estimated if link lost)
    duty: float         # commanded duty [-1, 1]
    mode: int
    link: int
    brake: int
    p: float
    i: float
    dterm: float
    pkt_good: int
    pkt_lost: int
    pkt_bad: int

    @property
    def t(self) -> float:
        return self.t_ms / 1000.0

    @property
    def mode_name(self) -> str:
        return MODE_NAMES[self.mode] if 0 <= self.mode < len(MODE_NAMES) else "?"

    @property
    def link_name(self) -> str:
        return LINK_NAMES[self.link] if 0 <= self.link < len(LINK_NAMES) else "?"

    @property
    def gap_error(self) -> float:
        return self.d - self.d_des

    @property
    def loss_pct(self) -> float:
        total = self.pkt_good + self.pkt_lost
        return 100.0 * self.pkt_lost / total if total else 0.0


@dataclass
class LeadRecord:
    t_ms: int
    v: float
    v_tgt: float
    accel: float
    duty: float
    braking: int
    hazard: int
    seq: int
    tx_ok: int
    tx_fail: int

    @property
    def t(self) -> float:
        return self.t_ms / 1000.0

    @property
    def tx_fail_pct(self) -> float:
        total = self.tx_ok + self.tx_fail
        return 100.0 * self.tx_fail / total if total else 0.0


@dataclass
class CommentRecord:
    text: str


# ---------------------------------------------------------------------
_INT_FIELDS = {"t_ms", "mode", "link", "brake", "pkt_good", "pkt_lost",
               "pkt_bad", "braking", "hazard", "seq", "tx_ok", "tx_fail"}


def _build(cls, parts: list[str]):
    names = [f.name for f in fields(cls)]
    if len(parts) < len(names):
        return None
    kwargs = {}
    for name, raw in zip(names, parts):
        try:
            kwargs[name] = int(float(raw)) if name in _INT_FIELDS else float(raw)
        except ValueError:
            return None
    return cls(**kwargs)


def parse_line(line: str):
    """Parse one telemetry line. Returns a record, or None if unparseable.

    Deliberately forgiving: a half-written line from a board that reset
    mid-print should be dropped, not crash a dashboard that has been
    running for twenty minutes.
    """
    line = line.strip()
    if not line:
        return None
    if line.startswith("#"):
        return CommentRecord(line.lstrip("# ").rstrip())

    parts = line.split(",")
    tag = parts[0].strip().upper()
    if tag == "F":
        return _build(FollowerRecord, parts[1:])
    if tag == "L":
        return _build(LeadRecord, parts[1:])
    return None


def parse_file(path) -> tuple[list[FollowerRecord], list[LeadRecord]]:
    """Read a captured log. Returns (follower_records, lead_records)."""
    follow: list[FollowerRecord] = []
    lead: list[LeadRecord] = []
    for raw in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        rec = parse_line(raw)
        if isinstance(rec, FollowerRecord):
            follow.append(rec)
        elif isinstance(rec, LeadRecord):
            lead.append(rec)
    return follow, lead


def records_to_csv(records: list, path=None) -> str:
    """Write records out as a proper headed CSV, for MATLAB / Excel / the report."""
    if not records:
        return ""
    names = [f.name for f in fields(records[0])]
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=names, lineterminator="\n")
    w.writeheader()
    for r in records:
        w.writerow(asdict(r))
    text = buf.getvalue()
    if path is not None:
        Path(path).write_text(text, encoding="utf-8")
    return text
