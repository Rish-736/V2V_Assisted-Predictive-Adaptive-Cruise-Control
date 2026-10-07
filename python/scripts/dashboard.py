#!/usr/bin/env python3
"""
dashboard.py -- live telemetry dashboard with side-by-side digital twin.

THREE SOURCES, so this works at every stage of the project:

    --demo              run the closed-loop twin as a live source.
                        No hardware, no serial port, no boards. Use this
                        to build and rehearse the demo today.

    --replay FILE       replay a recorded telemetry log in real time.
                        Record a good run once and you can always show
                        it, even if the hardware sulks on review day.

    --port COM5         the real thing: live serial from the follower.

WHAT IS PLOTTED
---------------
  1. GAP        measured gap vs the desired time-gap. The controller is
                working when these two converge.
  2. SPEED      lead (via V2V), follower measured, follower setpoint,
                and the DIGITAL TWIN's independent prediction.
  3. DUTY + closing rate
  4. MODE + LINK state bands, so you can see the state machine switch
     into PREDICT the instant the V2V braking flag arrives.

THE TWIN TRACE IS THE POINT
---------------------------
The twin is fed the duty the hardware COMMANDED and nothing else. It is
never given the measured speed. So the gap between the red twin trace
and the blue measured trace is pure model error -- friction, battery
sag, wheel slip, unmodelled delay. That divergence is the thing to
discuss in the report, and it is only meaningful because the twin never
sees the answer.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from v2vacc import OpenLoopTwin, PlantParams  # noqa: E402
from v2vacc.serial_link import SerialTelemetry, ReplaySource, autodetect_port, list_ports  # noqa: E402
from v2vacc.telemetry import FollowerRecord, LeadRecord, MODE_NAMES, LINK_NAMES  # noqa: E402


# =====================================================================
#  A live source backed by the closed-loop twin, so the dashboard can
#  be developed and demonstrated with nothing attached.
# =====================================================================
class DemoSource:
    def __init__(self, params: PlantParams | None = None, dt: float = 0.02,
                 telem_hz: float = 25.0, v2v: bool = True,
                 outage: tuple[float, float] | None = None):
        from v2vacc import ClosedLoopTwin
        self._twin = ClosedLoopTwin(dt=dt, params=params or PlantParams(),
                                    v2v_enabled=v2v)
        self._res = self._twin.run(600.0, initial_gap=1.0, v2v_outage=outage)
        self._stride = max(1, int(round((1.0 / telem_hz) / dt)))
        self._i = 0
        self._t0 = None
        self.banner = ["DEMO MODE -- closed-loop twin as the data source",
                       "no hardware attached; the control laws are the real ones"]
        self.name = "demo"

        class _S:
            connected = True
            parsed = 0
            dropped = 0
            errors = 0
        self.stats = _S()

    def start(self):
        self._t0 = time.monotonic()
        return self

    def stop(self):
        pass

    def __enter__(self):
        return self.start()

    def __exit__(self, *a):
        pass

    @property
    def alive(self) -> bool:
        return True

    @property
    def age(self) -> float:
        return 0.0

    def drain(self, limit: int = 5000) -> list:
        if self._t0 is None:
            return []
        now = (time.monotonic() - self._t0)
        r = self._res
        out = []
        while self._i < len(r.t) and len(out) < limit:
            if r.t[self._i] > now:
                break
            i = self._i
            self._i += self._stride
            out.append(FollowerRecord(
                t_ms=int(r.t[i] * 1000), d=r.gap[i], d_des=r.gap_des[i],
                closing=r.closing[i], ttc=r.ttc[i], v=r.v_follow[i],
                v_tgt=r.v_target[i], v_lead=r.v_lead[i], duty=r.duty[i],
                mode=r.mode[i], link=r.link[i], brake=r.braking[i],
                p=0.0, i=0.0, dterm=0.0,
                pkt_good=i, pkt_lost=0, pkt_bad=0))
            self.stats.parsed += 1
        return out


# =====================================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", default=None, help="serial port of the FOLLOWER")
    src.add_argument("--replay", type=Path, default=None, help="recorded log to replay")
    src.add_argument("--demo", action="store_true",
                     help="generate live data from the closed-loop twin")
    ap.add_argument("--lead-port", default=None,
                    help="optional serial port of the LEAD node")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--params", type=Path, default=None,
                    help="plant_params.json -- the twin's model")
    ap.add_argument("--window", type=float, default=20.0, help="x-axis span [s]")
    ap.add_argument("--no-twin", action="store_true", help="hide the twin trace")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--record", type=Path, default=None,
                    help="also append raw records to this CSV")
    ap.add_argument("--demo-no-v2v", action="store_true",
                    help="demo mode with the V2V link disabled, for comparison")
    args = ap.parse_args()

    if args.list:
        for dev, desc in list_ports():
            print(f"  {dev:<12} {desc}")
        return 0

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        print("matplotlib is required:  pip install -r python/requirements.txt",
              file=sys.stderr)
        return 2

    params = PlantParams.from_json(args.params) if args.params else PlantParams()

    # ---- pick the source --------------------------------------------
    if args.demo or (not args.port and not args.replay):
        if not args.demo:
            print("no --port or --replay given, falling back to --demo")
        source = DemoSource(params, v2v=not args.demo_no_v2v)
        title_src = "DEMO (closed-loop twin)"
    elif args.replay:
        source = ReplaySource(args.replay, speed=1.0, loop=True)
        title_src = f"REPLAY {args.replay.name}"
    else:
        port = args.port or autodetect_port()
        if port is None:
            print("no serial port found", file=sys.stderr)
            return 2
        source = SerialTelemetry(port, args.baud, name="follow")
        title_src = f"LIVE {port}"

    lead_source = None
    if args.lead_port:
        lead_source = SerialTelemetry(args.lead_port, args.baud, name="lead")

    twin = OpenLoopTwin(params, dt=0.04)

    # ---- rolling buffers --------------------------------------------
    N = 4000
    buf = {k: deque(maxlen=N) for k in
           ("t", "d", "d_des", "v", "v_tgt", "v_lead", "duty", "closing",
            "mode", "link", "brake", "twin_v", "ttc")}
    counters = {"good": 0, "lost": 0, "bad": 0, "last_t": None, "n": 0}

    rec_fh = None
    if args.record:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        rec_fh = args.record.open("w", encoding="utf-8", newline="")
        rec_fh.write("t_ms,d,d_des,closing,ttc,v,v_tgt,v_lead,duty,mode,link,"
                     "brake,twin_v\n")

    # ---- figure ------------------------------------------------------
    plt.style.use("default")
    fig, axes = plt.subplots(4, 1, figsize=(12, 9.5), sharex=True,
                             gridspec_kw={"height_ratios": [3, 3, 2, 1.3]})
    fig.canvas.manager.set_window_title("V2V Predictive ACC -- live dashboard")

    ax_gap, ax_spd, ax_duty, ax_mode = axes

    l_gap, = ax_gap.plot([], [], lw=2.0, color="#1f77b4", label="measured gap")
    l_des, = ax_gap.plot([], [], lw=1.6, ls="--", color="#ff7f0e",
                         label="desired gap (time-gap policy)")
    ax_gap.set_ylabel("gap [m]")
    ax_gap.grid(alpha=0.3)
    ax_gap.legend(loc="upper left", fontsize=8)

    l_vl, = ax_spd.plot([], [], lw=1.6, color="#2ca02c", label="lead (V2V)")
    l_v, = ax_spd.plot([], [], lw=2.0, color="#1f77b4", label="follower measured")
    l_vt, = ax_spd.plot([], [], lw=1.2, ls="--", color="#ff7f0e",
                        label="follower setpoint")
    l_tw, = ax_spd.plot([], [], lw=1.8, ls=":", color="#d62728",
                        label="DIGITAL TWIN prediction")
    ax_spd.set_ylabel("speed [m/s]")
    ax_spd.grid(alpha=0.3)
    ax_spd.legend(loc="upper left", fontsize=8, ncol=2)

    l_duty, = ax_duty.plot([], [], lw=1.6, color="#9467bd", label="duty")
    l_clos, = ax_duty.plot([], [], lw=1.4, color="#8c564b", label="closing rate [m/s]")
    ax_duty.axhline(0, color="k", lw=0.8)
    ax_duty.set_ylabel("duty / closing")
    ax_duty.grid(alpha=0.3)
    ax_duty.legend(loc="upper left", fontsize=8)

    l_mode, = ax_mode.step([], [], where="post", lw=1.8, color="#17becf")
    ax_mode.set_ylabel("mode")
    ax_mode.set_xlabel("time [s]")
    ax_mode.set_yticks(range(len(MODE_NAMES)))
    ax_mode.set_yticklabels(MODE_NAMES, fontsize=7)
    ax_mode.set_ylim(-0.5, len(MODE_NAMES) - 0.5)
    ax_mode.grid(alpha=0.3)

    status = fig.text(0.01, 0.975, "", fontsize=9, family="monospace", va="top")
    fig.suptitle(f"V2V-Assisted Predictive ACC  --  {title_src}",
                 fontsize=13, y=0.995)

    brake_spans: list = []

    # ---- animation ---------------------------------------------------
    def update(_frame):
        for rec in source.drain():
            if isinstance(rec, LeadRecord):
                continue
            if not isinstance(rec, FollowerRecord):
                continue

            t = rec.t
            dt = (t - counters["last_t"]) if counters["last_t"] is not None else 0.04
            counters["last_t"] = t
            counters["n"] += 1

            # ---- THE TWIN: fed the COMMANDED duty only. Never rec.v.
            tw = twin.step(rec.duty, dt=dt if 0.001 < dt < 0.5 else None)

            buf["t"].append(t)
            buf["d"].append(rec.d)
            buf["d_des"].append(rec.d_des)
            buf["v"].append(rec.v)
            buf["v_tgt"].append(rec.v_tgt)
            buf["v_lead"].append(rec.v_lead)
            buf["duty"].append(rec.duty)
            buf["closing"].append(rec.closing)
            buf["mode"].append(rec.mode)
            buf["link"].append(rec.link)
            buf["brake"].append(rec.brake)
            buf["ttc"].append(rec.ttc)
            buf["twin_v"].append(tw)

            counters["good"] = rec.pkt_good
            counters["lost"] = rec.pkt_lost
            counters["bad"] = rec.pkt_bad

            if rec_fh:
                rec_fh.write(f"{rec.t_ms},{rec.d:.4f},{rec.d_des:.4f},"
                             f"{rec.closing:.4f},{rec.ttc:.3f},{rec.v:.4f},"
                             f"{rec.v_tgt:.4f},{rec.v_lead:.4f},{rec.duty:.4f},"
                             f"{rec.mode},{rec.link},{rec.brake},{tw:.4f}\n")

        if not buf["t"]:
            status.set_text(f"waiting for data from {title_src} ...")
            return ()

        t = list(buf["t"])
        l_gap.set_data(t, list(buf["d"]))
        l_des.set_data(t, list(buf["d_des"]))
        l_vl.set_data(t, list(buf["v_lead"]))
        l_v.set_data(t, list(buf["v"]))
        l_vt.set_data(t, list(buf["v_tgt"]))
        l_tw.set_data([], []) if args.no_twin else l_tw.set_data(t, list(buf["twin_v"]))
        l_duty.set_data(t, list(buf["duty"]))
        l_clos.set_data(t, list(buf["closing"]))
        l_mode.set_data(t, list(buf["mode"]))

        t1 = t[-1]
        t0 = max(0.0, t1 - args.window)
        for ax in axes:
            ax.set_xlim(t0, max(t1, t0 + args.window * 0.25))

        vis_d = [x for ti, x in zip(t, buf["d"]) if ti >= t0]
        vis_dd = [x for ti, x in zip(t, buf["d_des"]) if ti >= t0]
        if vis_d:
            hi = max(max(vis_d), max(vis_dd) if vis_dd else 0) * 1.2 + 0.05
            ax_gap.set_ylim(0, hi)
        vis_v = [x for ti, x in zip(t, buf["v"]) if ti >= t0]
        vis_vl = [x for ti, x in zip(t, buf["v_lead"]) if ti >= t0]
        vis_tw = [x for ti, x in zip(t, buf["twin_v"]) if ti >= t0]
        if vis_v:
            hi = max(max(vis_v), max(vis_vl or [0]), max(vis_tw or [0])) * 1.25 + 0.03
            ax_spd.set_ylim(-0.02, hi)
        ax_duty.set_ylim(-1.15, 1.15)

        # shade the braking episodes
        for sp in brake_spans:
            sp.remove()
        brake_spans.clear()
        in_brake, start = False, None
        for ti, b in zip(t, buf["brake"]):
            if b and not in_brake:
                in_brake, start = True, ti
            elif not b and in_brake:
                in_brake = False
                if start >= t0:
                    brake_spans.append(
                        ax_gap.axvspan(start, ti, color="red", alpha=0.10, zorder=0))
        if in_brake and start is not None:
            brake_spans.append(
                ax_gap.axvspan(start, t[-1], color="red", alpha=0.10, zorder=0))

        # ---- status line: model error is the headline number
        from v2vacc import compare
        n_cmp = min(len(buf["v"]), len(buf["twin_v"]))
        tail = min(n_cmp, 500)
        met = compare(list(buf["v"])[-tail:], list(buf["twin_v"])[-tail:]) if tail > 10 else None

        mode_i = buf["mode"][-1]
        link_i = buf["link"][-1]
        total = counters["good"] + counters["lost"]
        loss = 100.0 * counters["lost"] / total if total else 0.0

        lines = [
            f"t={t[-1]:7.2f}s   mode={MODE_NAMES[mode_i]:<8} "
            f"link={LINK_NAMES[link_i]:<9} {'BRAKING' if buf['brake'][-1] else '       '}",
            f"gap={buf['d'][-1]*100:6.1f}cm (want {buf['d_des'][-1]*100:5.1f}cm, "
            f"err {(buf['d'][-1]-buf['d_des'][-1])*100:+6.1f}cm)   "
            f"ttc={buf['ttc'][-1]:5.2f}s  closing={buf['closing'][-1]*100:+6.1f}cm/s",
            f"v={buf['v'][-1]:5.3f}  lead={buf['v_lead'][-1]:5.3f}  "
            f"set={buf['v_tgt'][-1]:5.3f}  duty={buf['duty'][-1]:+5.2f}   "
            f"V2V pkts={counters['good']} lost={counters['lost']} "
            f"bad={counters['bad']} ({loss:.1f}% loss)",
        ]
        if met and not args.no_twin:
            lines.append(f"TWIN vs REAL (last {met.n} samples): "
                         f"fit={met.nrmse_fit_pct:5.1f}%  RMSE={met.rmse:.4f} m/s  "
                         f"max|e|={met.max_err:.4f} m/s")
        status.set_text("\n".join(lines))
        return ()

    with source:
        if lead_source:
            lead_source.start()
        ani = FuncAnimation(fig, update, interval=50, blit=False,
                            cache_frame_data=False)
        fig.tight_layout(rect=(0, 0, 1, 0.90))
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            if lead_source:
                lead_source.stop()
            if rec_fh:
                rec_fh.close()
                print(f"recorded -> {args.record}")
    # keep a reference so the animation is not garbage collected early
    del ani
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
