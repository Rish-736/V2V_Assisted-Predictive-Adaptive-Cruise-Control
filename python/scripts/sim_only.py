#!/usr/bin/env python3
"""
sim_only.py -- run the closed-loop digital twin with no hardware at all.

This is the offline sandbox. Use it to tune gains, to explore scenarios
that are tedious or dangerous to stage on the floor, and above all to
produce THE headline result of the project:

    the same controller, the same plant, the same scenario,
    run once WITH the V2V link and once WITHOUT it.

If the V2V feedforward is doing what we claim, the no-V2V run brakes
later, gets closer, and in a hard-stop scenario makes contact where the
V2V run does not.

EXAMPLES
--------
    # headline comparison + plot
    python python/scripts/sim_only.py --compare --plot

    # just the nominal run, save traces for the report
    python python/scripts/sim_only.py --csv data/sim_nominal.csv

    # degradation study: force the link down from t=22s to t=30s,
    # right across the emergency stop
    python python/scripts/sim_only.py --outage 22 30 --plot

    # link-quality sweep: how much packet loss can we tolerate?
    python python/scripts/sim_only.py --sweep-loss

    # use the identified plant instead of the nominal guess
    python python/scripts/sim_only.py --params data/plant_params.json --compare
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from v2vacc import ClosedLoopTwin, PlantParams, Gains, GapPolicy, BrakePolicy  # noqa: E402
from v2vacc.telemetry import MODE_NAMES  # noqa: E402


# ---------------------------------------------------------------------
def summarise(name: str, res) -> dict:
    """Pull out the numbers that go in the report table."""
    # Reaction latency is measured around the EMERGENCY STOP at t=24 s,
    # the sharpest braking event in the scenario.
    lat_hard = res.reaction_latency(after=23.0)
    lat_soft = res.reaction_latency(after=11.0)
    resp_hard = res.response_latency(after=23.0)
    mag_hard = res.response_magnitude(after=23.0, window=0.5)
    mag_soft = res.response_magnitude(after=11.0, window=0.5)
    resp_soft = res.response_latency(after=11.0)

    first_brake_i = next((i for i, b in enumerate(res.braking) if b), None)
    t_brake = res.t[first_brake_i] if first_brake_i is not None else None

    # steady-state gap tracking error over the first cruise stretch
    window = [(g, gd) for t, g, gd in zip(res.t, res.gap, res.gap_des)
              if 8.0 <= t <= 11.5]
    ss_err = (sum(abs(g - gd) for g, gd in window) / len(window)) if window else float("nan")

    # how close did we get relative to the gap we were asking for?
    ratio = [g / gd for g, gd in zip(res.gap, res.gap_des) if gd > 0.01]

    return {
        "name": name,
        "min_gap_cm": res.min_gap * 100.0,
        "collided": res.collided,
        "t_first_brake": t_brake,
        "lat_soft_ms": lat_soft * 1000.0 if lat_soft is not None else None,
        "lat_hard_ms": lat_hard * 1000.0 if lat_hard is not None else None,
        "resp_soft_ms": resp_soft * 1000.0 if resp_soft is not None else None,
        "resp_hard_ms": resp_hard * 1000.0 if resp_hard is not None else None,
        "mag_soft": mag_soft,
        "mag_hard": mag_hard,
        "ss_gap_err_cm": ss_err * 100.0,
        "min_gap_ratio": min(ratio) if ratio else float("nan"),
        "mean_gap_cm": (sum(res.gap) / len(res.gap) * 100.0) if res.gap else 0.0,
    }


def print_summary(rows: list[dict]) -> None:
    print()
    def fmt(v, none="--"):
        return f"{v:.0f} ms" if v is not None else none

    print(f"{'run':<26} {'min gap':>9} {'mean gap':>9} {'contact':>8} "
          f"{'slow@soft':>10} {'brake@soft':>11} {'slow@hard':>10} {'brake@hard':>11}")
    print("-" * 100)
    for r in rows:
        print(f"{r['name']:<26} {r['min_gap_cm']:>8.1f}cm {r['mean_gap_cm']:>8.1f}cm "
              f"{('YES' if r['collided'] else 'no'):>8} "
              f"{fmt(r['resp_soft_ms']):>10} {fmt(r['lat_soft_ms'], 'never'):>11} "
              f"{fmt(r['resp_hard_ms']):>10} {fmt(r['lat_hard_ms'], 'never'):>11}")
    print()
    print("slow@  = lag until the follower STARTS EASING OFF (speed setpoint falls)")
    print("brake@ = lag until the follower ENGAGES THE BRAKING LAYER")
    print("soft   = the lead's gentle slowdown at t=12 s")
    print("hard   = the lead's emergency stop at t=24 s")
    print("'never' means that event never tripped the discrete braking layer --")
    print("the controller handled it through the speed loop alone.")
    print()


# ---------------------------------------------------------------------
def do_plot(runs: list[tuple[str, object]], out_path: Path | None) -> None:
    try:
        import matplotlib
        if out_path is not None:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -- skipping plot "
              "(pip install -r python/requirements.txt)")
        return

    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    colours = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]

    for (label, res), c in zip(runs, colours):
        axes[0].plot(res.t, res.gap, color=c, label=f"{label}: actual gap")
        axes[0].plot(res.t, res.gap_des, color=c, ls=":", alpha=0.55,
                     label=f"{label}: desired gap")
        axes[1].plot(res.t, res.v_follow, color=c, label=f"{label}: follower")
        axes[2].plot(res.t, res.duty, color=c, label=label)
        axes[3].step(res.t, res.mode, color=c, where="post", label=label)

    # the lead's speed is identical across runs -- draw it once
    if runs:
        axes[1].plot(runs[0][1].t, runs[0][1].v_lead, "k--", lw=1.4,
                     label="lead (V2V truth)")

    axes[0].axhline(0.0, color="k", lw=0.8)
    axes[0].axhspan(-0.05, 0.0, color="red", alpha=0.18)
    axes[0].set_ylabel("gap [m]")
    axes[0].set_title("V2V-Assisted Predictive ACC -- closed-loop digital twin")
    axes[0].legend(fontsize=7, ncol=2, loc="upper right")
    axes[0].grid(alpha=0.3)

    axes[1].set_ylabel("speed [m/s]")
    axes[1].legend(fontsize=7, ncol=2, loc="upper right")
    axes[1].grid(alpha=0.3)

    axes[2].set_ylabel("duty [-1..1]")
    axes[2].axhline(0.0, color="k", lw=0.8)
    axes[2].legend(fontsize=7)
    axes[2].grid(alpha=0.3)

    axes[3].set_ylabel("mode")
    axes[3].set_yticks(range(len(MODE_NAMES)))
    axes[3].set_yticklabels(MODE_NAMES, fontsize=7)
    axes[3].set_xlabel("time [s]")
    axes[3].legend(fontsize=7)
    axes[3].grid(alpha=0.3)

    fig.tight_layout()
    if out_path is not None:
        fig.savefig(out_path, dpi=150)
        print(f"plot saved -> {out_path}")
    else:
        plt.show()


# ---------------------------------------------------------------------
def sweep_loss(args, params) -> None:
    print("\npacket-loss sweep (does the controller stay safe as the radio degrades?)")
    print(f"{'loss %':>8} {'min gap':>10} {'contact':>9}")
    print("-" * 30)
    for loss in (0.0, 0.1, 0.2, 0.4, 0.6, 0.8, 0.95, 1.0):
        twin = ClosedLoopTwin(dt=args.dt, params=params,
                              v2v_enabled=loss < 1.0,
                              v2v_loss_rate=loss,
                              v2v_latency_s=args.latency)
        res = twin.run(args.duration, initial_gap=args.gap)
        print(f"{loss*100:>7.0f}% {res.min_gap*100:>9.1f}cm "
              f"{('YES' if res.collided else 'no'):>9}")
    print("\nInterpretation: the TTC and gap-floor layers are what keep the")
    print("min gap positive as V2V degrades. V2V improves the margin; it is")
    print("not load-bearing for basic safety. That separation is deliberate.")


# ---------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=45.0, help="seconds")
    ap.add_argument("--dt", type=float, default=0.02, help="timestep [s]")
    ap.add_argument("--gap", type=float, default=1.00, help="initial gap [m]")
    ap.add_argument("--params", type=Path, default=None,
                    help="plant_params.json from sysid_fit.py")
    ap.add_argument("--compare", action="store_true",
                    help="run with AND without V2V and tabulate the difference")
    ap.add_argument("--fair", action="store_true",
                    help="hold the time gap EQUAL in both runs. Without this, "
                         "the sensor-only run correctly falls back to the wider "
                         "degraded headway, so the two runs are not following at "
                         "the same distance and only the latency columns are "
                         "comparable. Use --fair to isolate the effect of the "
                         "V2V feedforward alone.")
    ap.add_argument("--outage", nargs=2, type=float, metavar=("T0", "T1"),
                    default=None, help="force a V2V outage over this window")
    ap.add_argument("--latency", type=float, default=0.0,
                    help="V2V one-way latency [s]")
    ap.add_argument("--loss", type=float, default=0.0,
                    help="V2V packet loss rate 0..1")
    ap.add_argument("--sweep-loss", action="store_true",
                    help="sweep packet loss and report the safety margin")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--save-plot", type=Path, default=None)
    ap.add_argument("--csv", type=Path, default=None,
                    help="write the nominal run's traces to CSV")
    # gain overrides, for quick manual tuning
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--ki", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--tgap", type=float, default=None)
    ap.add_argument("--kgap", type=float, default=None)
    args = ap.parse_args()

    params = PlantParams.from_json(args.params) if args.params else PlantParams()
    print(f"plant: {params.summary()}")

    gains = Gains()
    if args.kp is not None:
        gains.kp = args.kp
    if args.ki is not None:
        gains.ki = args.ki
    if args.kd is not None:
        gains.kd = args.kd
    print(f"gains: kp={gains.kp} ki={gains.ki} kd={gains.kd}")

    gp = GapPolicy()
    if args.tgap is not None:
        gp.t_gap = args.tgap
    if args.kgap is not None:
        gp.k_gap = args.kgap
    print(f"policy: Th={gp.t_gap}s d0={gp.d_standstill}m Kgap={gp.k_gap}")

    if args.sweep_loss:
        sweep_loss(args, params)
        return 0

    outage = tuple(args.outage) if args.outage else None

    def build(v2v: bool) -> ClosedLoopTwin:
        policy = GapPolicy(**vars(gp))
        if args.fair:
            # same headway whether or not the link is up -- isolates the
            # feedforward effect from the degraded-mode safety margin
            policy.t_gap_degraded = policy.t_gap
        return ClosedLoopTwin(dt=args.dt, params=params, gains=Gains(**vars(gains)),
                              gap_policy=policy,
                              brake_policy=BrakePolicy(),
                              v2v_enabled=v2v, v2v_latency_s=args.latency,
                              v2v_loss_rate=args.loss)

    runs: list[tuple[str, object]] = []
    rows: list[dict] = []

    res_v2v = build(True).run(args.duration, initial_gap=args.gap, v2v_outage=outage)
    label = "with V2V" + (f" (outage {outage[0]:g}-{outage[1]:g}s)" if outage else "")
    runs.append((label, res_v2v))
    rows.append(summarise(label, res_v2v))

    if args.compare:
        res_plain = build(False).run(args.duration, initial_gap=args.gap)
        runs.append(("sensor only", res_plain))
        rows.append(summarise("sensor only", res_plain))

    print_summary(rows)

    if args.compare and len(rows) == 2:
        a, b = rows[0], rows[1]
        print("INTERPRETATION")
        print("-" * 92)

        if args.fair:
            print(f"Same {gp.t_gap:.2f} s headway in both runs, so this isolates the")
            print("V2V feedforward term from the degraded-mode margin.")
            d = a["min_gap_cm"] - b["min_gap_cm"]
            print(f"  minimum clearance:  {d:+.1f} cm in favour of V2V")
        else:
            print("NOTE: the sensor-only run has fallen back to the degraded")
            print(f"      headway ({gp.t_gap_degraded:.2f} s vs {gp.t_gap:.2f} s), so it deliberately")
            print("      follows further back. Compare the LATENCY columns here;")
            print("      re-run with --fair for an equal-headway clearance comparison.")
            print(f"  mean following distance: {a['mean_gap_cm']:.1f} cm with V2V "
                  f"vs {b['mean_gap_cm']:.1f} cm without")
            print("  -> V2V buys a TIGHTER GAP AT EQUAL SAFETY, which is exactly the")
            print("     CACC result in the literature (0.6 s CACC vs 1.5 s ACC headway)")

        for tag, key in (("gentle slowdown", "resp_soft_ms"),
                         ("emergency stop", "resp_hard_ms")):
            av, bv = a[key], b[key]
            if av is not None and bv is not None:
                print(f"  starts slowing after {tag}: {av:.0f} ms with V2V vs "
                      f"{bv:.0f} ms without  ({bv - av:+.0f} ms earlier)")
        for tag, key in (("gentle slowdown", "lat_soft_ms"),
                         ("emergency stop", "lat_hard_ms")):
            av, bv = a[key], b[key]
            if av is not None and bv is not None:
                print(f"  engages braking after {tag}: {av:.0f} ms with V2V vs "
                      f"{bv:.0f} ms without  ({bv - av:+.0f} ms earlier)")
            elif av is not None and bv is None:
                print(f"  engages braking after {tag}: {av:.0f} ms with V2V; "
                      f"sensor-only never trips the braking layer (it eases off "
                      f"through the speed loop instead)")

        for tag, key in (("gentle slowdown", "mag_soft"),
                         ("emergency stop", "mag_hard")):
            av, bv = a[key], b[key]
            if av is not None and bv is not None:
                extra = (av / bv) if abs(bv) > 1e-4 else float("inf")
                print(f"  speed shed in the first 0.5 s after the {tag}: "
                      f"{av*100:.1f} cm/s with V2V vs {bv*100:.1f} cm/s without"
                      + (f"  ({extra:.1f}x stronger)" if extra != float("inf") else ""))

        if b["collided"] and not a["collided"]:
            print("  CONTACT: sensor-only collides, V2V does not. Headline result.")
        elif not a["collided"] and not b["collided"]:
            print("  Neither run makes contact. Expected at scale-model speeds:")
            print("  this car stops in ~5 cm but follows at ~60 cm, so it is an")
            print("  order of magnitude over-provisioned compared with a real")
            print("  vehicle. Report the LATENCY and CLEARANCE margins, not")
            print("  collision counts -- see docs/CONTROL_DESIGN.md.")
        print()

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        res_v2v.to_csv(args.csv)
        print(f"traces saved -> {args.csv}")

    if args.plot or args.save_plot:
        do_plot(runs, args.save_plot)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
