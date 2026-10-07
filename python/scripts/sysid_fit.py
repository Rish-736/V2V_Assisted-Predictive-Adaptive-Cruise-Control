#!/usr/bin/env python3
"""
sysid_fit.py -- fit a motor transfer function to measured step-response data.

This is the single point where real hardware data enters the model. Run it
ONCE (per motor), offline, on data captured with
firmware/tools/step_response. Everything downstream -- PID gains, root
locus, Bode, lead-lag design, the digital twin -- is built on its output.

    python python/scripts/sysid_fit.py data/step.csv --plot --out data/plant_params.json

INPUT
-----
CSV with columns:  t_ms, duty, speed_mps, enc_count, tag
(exactly what step_response.ino prints). Comment lines starting with '#'
are ignored.

WHAT IT FITS
------------
                     K
    V(s)/U(s) = ----------- * exp(-L*s)
                 tau*s + 1

Two estimators are run and reported side by side:

  1. GRAPHICAL  -- the classic by-hand method. K from the steady state,
     L from where the response leaves zero, tau from the 63.2% point.
     Reported because it is what the examiner can verify off the plot
     with a ruler, and because a numerical fit that disagrees wildly
     with it usually means the data is bad, not that the fit is clever.

  2. LEAST-SQUARES -- scipy.optimize over (K, tau, L) against the whole
     curve. More accurate; falls back to a coarse grid search if scipy
     is not installed, so the script always works.

PER-LEVEL FITTING
-----------------
Each step amplitude is fitted separately. A geared DC motor is NOT
linear: static friction means K and tau drift with operating point. The
script reports the spread and picks the level nearest the cruise duty as
the nominal model, then states the valid range honestly. A single
lumped fit across all levels would hide exactly the nonlinearity that
makes the real hardware miss the predicted settling time.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


# ---------------------------------------------------------------------
def load_csv(path: Path) -> list[dict]:
    rows = []
    header = None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if header is None:
            if parts[0].lower().startswith("t_ms"):
                header = [p.lower() for p in parts]
                continue
            header = ["t_ms", "duty", "speed_mps", "enc_count", "tag"]
        if len(parts) < 3:
            continue
        rec = dict(zip(header, parts))
        try:
            rows.append({
                "t": float(rec["t_ms"]) / 1000.0,
                "duty": float(rec["duty"]),
                "v": float(rec["speed_mps"]),
                "tag": rec.get("tag", ""),
            })
        except (ValueError, KeyError):
            continue
    return rows


def split_segments(rows: list[dict]) -> list[dict]:
    """Group consecutive samples that share a duty level into segments."""
    segs: list[dict] = []
    cur: dict | None = None
    for r in rows:
        if cur is None or abs(r["duty"] - cur["duty"]) > 1e-6:
            cur = {"duty": r["duty"], "t": [], "v": [], "tag": r["tag"]}
            segs.append(cur)
        cur["t"].append(r["t"])
        cur["v"].append(r["v"])
    return [s for s in segs if len(s["t"]) > 10]


# ---------------------------------------------------------------------
def smooth(v: list[float], n: int = 5) -> list[float]:
    if n < 2 or len(v) < n:
        return list(v)
    out, half = [], n // 2
    for i in range(len(v)):
        lo, hi = max(0, i - half), min(len(v), i + half + 1)
        out.append(sum(v[lo:hi]) / (hi - lo))
    return out


def graphical_fit(t: list[float], v: list[float], duty: float) -> dict | None:
    """Classic K / L / tau read off the curve."""
    if duty <= 1e-6 or len(t) < 20:
        return None
    vs = smooth(v, 7)
    t0 = t[0]

    # steady state = mean of the last 25% of the segment
    tail = vs[int(len(vs) * 0.75):]
    v_ss = sum(tail) / len(tail)
    if v_ss <= 1e-4:
        return None

    # dead time: first sample that clears 5% of the final value
    thr5 = 0.05 * v_ss
    L = 0.0
    for ti, vi in zip(t, vs):
        if vi >= thr5:
            L = ti - t0
            break

    # time constant: 63.2% point, measured from the end of the dead time
    thr63 = 0.632 * v_ss
    tau = None
    for ti, vi in zip(t, vs):
        if vi >= thr63:
            tau = (ti - t0) - L
            break
    if tau is None or tau <= 1e-4:
        return None

    return {"K": v_ss / duty, "tau": tau, "delay": L, "v_ss": v_ss}


def simulate(t: list[float], duty: float, K: float, tau: float, L: float) -> list[float]:
    """Analytic first-order step response with dead time."""
    t0 = t[0]
    return [0.0 if (ti - t0) < L else K * duty * (1.0 - math.exp(-((ti - t0) - L) / tau))
            for ti in t]


def sse(t, v, duty, K, tau, L) -> float:
    pred = simulate(t, duty, K, tau, L)
    return sum((a - b) ** 2 for a, b in zip(v, pred))


def lsq_fit(t: list[float], v: list[float], duty: float, seed: dict) -> dict:
    """Refine the graphical estimate. scipy if available, grid search if not."""
    K0, tau0, L0 = seed["K"], seed["tau"], seed["delay"]
    try:
        from scipy.optimize import minimize  # type: ignore

        def cost(p):
            K, tau, L = p
            if tau <= 1e-3 or L < 0 or K <= 0:
                return 1e9
            return sse(t, v, duty, K, tau, L)

        r = minimize(cost, [K0, tau0, L0], method="Nelder-Mead",
                     options={"xatol": 1e-5, "fatol": 1e-10, "maxiter": 4000})
        K, tau, L = r.x
        method = "scipy Nelder-Mead"
    except ImportError:
        best, K, tau, L = sse(t, v, duty, K0, tau0, L0), K0, tau0, L0
        for dK in [0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15]:
            for dT in [0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.4]:
                for dL in [0.0, 0.5, 0.75, 1.0, 1.25, 1.5]:
                    c = sse(t, v, duty, K0 * dK, tau0 * dT, L0 * dL)
                    if c < best:
                        best, K, tau, L = c, K0 * dK, tau0 * dT, L0 * dL
        method = "grid search (install scipy for a better fit)"

    pred = simulate(t, duty, K, tau, L)
    mean_v = sum(v) / len(v)
    ss_tot = sum((x - mean_v) ** 2 for x in v)
    ss_res = sum((a - b) ** 2 for a, b in zip(v, pred))
    r2 = 1 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    fit_pct = 100 * (1 - math.sqrt(ss_res) / math.sqrt(ss_tot)) if ss_tot > 1e-12 else 0.0

    return {"K": K, "tau": tau, "delay": L, "r2": r2,
            "fit_pct": fit_pct, "method": method}


def fit_static_curve(levels: list[dict]) -> tuple[float, float]:
    """Fit the steady-state curve  v_ss = m*duty + c  across all levels.

    Returns (slope m, dead-zone duty).

    WHY THIS MATTERS -- and it is easy to get wrong:
    The per-level K reported above is the APPARENT gain, v_ss/duty. It
    already has the dead-zone loss baked into it, and it therefore varies
    with operating point (that is the 49%-of-mean spread in the table).

    The twin and the firmware simulator apply the dead-zone SEPARATELY,
    as an explicit nonlinearity, and then multiply by K. Feeding them an
    apparent K would subtract the dead-zone twice and make the model
    predict a speed ~20% low at mid duty.

    The true dead-zone-corrected gain is the SLOPE of the static curve:
        v_ss = K*(duty - dz)/(1 - dz)  =>  m = K/(1 - dz)
        so    K = m*(1 - dz)
    and the dead-zone is the x-intercept, -c/m.
    """
    pts = [(lv["duty"], lv["v_ss"]) for lv in levels if lv.get("v_ss")]
    if len(pts) < 2:
        return (pts[0][1] / pts[0][0] if pts else 0.0), 0.0
    n = len(pts)
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return 0.0, 0.0
    m = (n * sxy - sx * sy) / den
    c = (sy - m * sx) / n
    dz = max(0.0, min(0.9, -c / m)) if abs(m) > 1e-9 else 0.0
    return m, dz


# ---------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="step-response capture")
    ap.add_argument("--out", type=Path, default=Path("data/plant_params.json"))
    ap.add_argument("--nominal-duty", type=float, default=0.60,
                    help="operating point the nominal model should match")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--save-plot", type=Path, default=None)
    args = ap.parse_args()

    if not args.csv.is_file():
        print(f"error: {args.csv} not found", file=sys.stderr)
        print("Capture it first with firmware/tools/step_response, e.g.:")
        print("  python python/scripts/log_serial.py --port COM5 "
              "--out data/step.csv --duration 45")
        return 2

    rows = load_csv(args.csv)
    if not rows:
        print("error: no usable rows", file=sys.stderr)
        return 2
    print(f"loaded {len(rows)} samples over {rows[-1]['t'] - rows[0]['t']:.1f} s")

    segs = [s for s in split_segments(rows) if s["duty"] > 1e-6]
    if not segs:
        print("error: no rising-step segments found "
              "(every sample has duty = 0?)", file=sys.stderr)
        return 2

    print(f"\n{'duty':>6} {'K':>9} {'tau':>8} {'L':>8} {'fit%':>7} {'R2':>7}  method")
    print("-" * 74)

    levels = []
    for seg in segs:
        g = graphical_fit(seg["t"], seg["v"], seg["duty"])
        if g is None:
            print(f"{seg['duty']:>6.2f}   -- no usable step (dead-zone or too short)")
            continue
        f = lsq_fit(seg["t"], seg["v"], seg["duty"], g)
        f["duty"] = seg["duty"]
        f["v_ss"] = g["v_ss"]
        f["graphical"] = g
        levels.append(f)
        print(f"{f['duty']:>6.2f} {f['K']:>9.4f} {f['tau']:>8.4f} {f['delay']:>8.4f} "
              f"{f['fit_pct']:>6.1f}% {f['r2']:>7.4f}  {f['method']}")

    if not levels:
        print("error: nothing could be fitted", file=sys.stderr)
        return 2

    # --- nonlinearity report -----------------------------------------
    Ks = [lv["K"] for lv in levels]
    taus = [lv["tau"] for lv in levels]
    print(f"\nspread across operating points:")
    print(f"  K   : {min(Ks):.4f} .. {max(Ks):.4f}  "
          f"({100*(max(Ks)-min(Ks))/max(1e-9, sum(Ks)/len(Ks)):.0f}% of mean)")
    print(f"  tau : {min(taus):.4f} .. {max(taus):.4f}  "
          f"({100*(max(taus)-min(taus))/max(1e-9, sum(taus)/len(taus)):.0f}% of mean)")
    print("  A large spread is normal and worth reporting -- it is the")
    print("  gearmotor's nonlinearity, and it bounds how well ANY single")
    print("  linear controller can do. Quote it in the report.")

    # --- nominal model = level closest to the cruise operating point ---
    nominal = min(levels, key=lambda lv: abs(lv["duty"] - args.nominal_duty))
    slope, deadzone = fit_static_curve(levels)
    K_corrected = slope * (1.0 - deadzone)

    params = {
        "K": round(K_corrected, 5),
        "tau": round(nominal["tau"], 5),
        "delay": round(nominal["delay"], 5),
        "deadzone": round(deadzone, 4),
        "v_max": round(max(lv["v_ss"] for lv in levels), 4),
    }

    print(f"\nNOMINAL MODEL (fitted at duty = {nominal['duty']:.2f})")
    print(f"              {params['K']:.4f}")
    print(f"  G(s) = ----------------- * exp(-{params['delay']:.4f} s)")
    print(f"          {params['tau']:.4f} s + 1")
    print(f"  dead-zone  : {params['deadzone']:.3f} duty "
          f"(below this the wheel does not turn -- NOT in the transfer function)")
    print(f"  v_max      : {params['v_max']:.3f} m/s")
    print(f"  goodness   : {nominal['fit_pct']:.1f}% fit, R2 = {nominal['r2']:.4f}")

    report = {
        "source": str(args.csv),
        "params": params,
        "nominal_duty": nominal["duty"],
        "static_curve": {"slope": slope, "deadzone": deadzone,
                         "K_corrected": K_corrected},
        "apparent_K_at_nominal": nominal["K"],
        "goodness": {"fit_pct": nominal["fit_pct"], "r2": nominal["r2"]},
        "per_level": [{k: v for k, v in lv.items() if k != "graphical"}
                      for lv in levels],
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\nsaved -> {args.out}")

    print("\nNEXT STEPS")
    print(f"  1. paste these into firmware/common/config.h section 8, then")
    print(f"     python tools/sync_common.py")
    print(f"  2. python python/scripts/design_control.py --params {args.out}")
    print(f"  3. python python/scripts/sim_only.py --params {args.out} --compare --fair")

    if args.plot or args.save_plot:
        plot(segs, levels, args.save_plot)
    return 0


def plot(segs, levels, out_path):
    try:
        import matplotlib
        if out_path:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -- skipping plot")
        return

    n = len(levels)
    fig, axes = plt.subplots(1, max(1, n), figsize=(4 * max(1, n), 4), squeeze=False)
    by_duty = {round(s["duty"], 4): s for s in segs}
    for ax, lv in zip(axes[0], levels):
        seg = by_duty.get(round(lv["duty"], 4))
        if seg is None:
            continue
        t0 = seg["t"][0]
        ts = [t - t0 for t in seg["t"]]
        ax.plot(ts, seg["v"], ".", ms=2, color="#888", label="measured")
        pred = simulate(seg["t"], lv["duty"], lv["K"], lv["tau"], lv["delay"])
        ax.plot(ts, pred, "-", lw=2, color="#d62728", label="fitted 1st order")
        ax.axhline(lv["v_ss"], ls=":", color="k", lw=0.8)
        ax.axhline(0.632 * lv["v_ss"], ls=":", color="#1f77b4", lw=0.8)
        ax.axvline(lv["delay"] + lv["tau"], ls=":", color="#1f77b4", lw=0.8)
        ax.set_title(f"duty {lv['duty']:.2f}\nK={lv['K']:.3f} tau={lv['tau']:.3f}s "
                     f"({lv['fit_pct']:.0f}% fit)", fontsize=9)
        ax.set_xlabel("time [s]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    axes[0][0].set_ylabel("speed [m/s]")
    fig.suptitle("System identification -- open-loop step response", fontsize=11)
    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=150)
        print(f"plot saved -> {out_path}")
    else:
        plt.show()


if __name__ == "__main__":
    raise SystemExit(main())
