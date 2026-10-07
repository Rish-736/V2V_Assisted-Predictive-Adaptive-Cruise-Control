#!/usr/bin/env python3
"""
design_control.py -- PID + lead-lag design and stability analysis.

Takes the identified plant and produces (a) gains to paste into
config.h, (b) the root locus / Bode / step plots for the report, and
(c) the stability margins to quote in the viva.

    python python/scripts/design_control.py --params data/plant_params.json --plot

WHAT IT DOES
------------
1. INNER LOOP (speed). Lambda tuning on the first-order plant:
   for G = K/(tau s + 1) under PI control, choosing
        Kp = tau / (K * lambda),  Ki = Kp / tau
   cancels the plant pole and gives a first-order closed loop with time
   constant lambda. One knob (lambda = how fast do you want it), no
   trial and error, and provably no overshoot in the nominal case.
   lambda is bounded below by the transport delay: asking for a closed
   loop faster than ~3L is asking for trouble.

2. Reports gain margin, phase margin and crossover from the Bode data.

3. OUTER LOOP (time gap). The gap plant is a pure integrator, so the
   outer loop with gain K_gap has crossover at K_gap rad/s. For the
   cascade to be valid the outer loop must be SLOW compared with the
   inner one -- the script checks the separation and complains if it is
   under 3x, because that is when the "design them independently"
   assumption silently stops holding.

4. LEAD-LAG. Designs a lead compensator to recover a target phase
   margin, and emits the discrete biquad coefficients for config.h.

Everything degrades gracefully: with python-control installed you get
proper root-locus/Bode plots; without it you still get every number,
computed from closed-form expressions.
"""

from __future__ import annotations

import argparse
import cmath
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from v2vacc import PlantParams  # noqa: E402


# ---------------------------------------------------------------------
def freq_response(K: float, tau: float, L: float, kp: float, ki: float,
                  kd: float, w: float) -> complex:
    """Loop gain L(jw) = C(jw) * G(jw), G first order with dead time."""
    jw = 1j * w
    C = kp + ki / jw + kd * jw
    G = K / (tau * jw + 1.0) * cmath.exp(-L * jw)
    return C * G


def margins(K: float, tau: float, L: float, kp: float, ki: float, kd: float):
    """Gain margin, phase margin and the two crossover frequencies.

    Found by scanning a log-spaced grid then bisecting -- no scipy needed.

    NOTE ON PHASE UNWRAPPING: cmath.phase() returns a value in (-pi, pi],
    so a true phase of -190 deg comes back as +170 deg. With a transport
    delay the phase falls without bound, so it wraps repeatedly. Searching
    for the -180 deg crossing on the WRAPPED phase finds nothing and
    reports an infinite gain margin -- which is exactly the wrong answer,
    because the delay is precisely what makes the gain margin finite.
    So we build an unwrapped phase curve first and search that.
    """
    def mag(w: float) -> float:
        return abs(freq_response(K, tau, L, kp, ki, kd, w))

    def raw_phase(w: float) -> float:
        return math.degrees(cmath.phase(freq_response(K, tau, L, kp, ki, kd, w)))

    ws = [10 ** (-2 + 5 * i / 4000.0) for i in range(4001)]   # 0.01 .. 1000

    # --- unwrap the phase along the grid
    phases: list[float] = []
    offset = 0.0
    prev = None
    for w in ws:
        ph = raw_phase(w)
        if prev is not None:
            d = ph - prev
            if d > 180.0:
                offset -= 360.0
            elif d < -180.0:
                offset += 360.0
        prev = ph
        phases.append(ph + offset)

    def phase_at(w: float) -> float:
        """Interpolate the unwrapped phase at an arbitrary frequency."""
        if w <= ws[0]:
            return phases[0]
        if w >= ws[-1]:
            return phases[-1]
        lo, hi = 0, len(ws) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ws[mid] <= w:
                lo = mid
            else:
                hi = mid
        f = (math.log(w) - math.log(ws[lo])) / (math.log(ws[hi]) - math.log(ws[lo]))
        return phases[lo] + f * (phases[hi] - phases[lo])

    # --- gain crossover: |L| = 1
    wgc = None
    for a, b in zip(ws, ws[1:]):
        if (mag(a) - 1.0) * (mag(b) - 1.0) < 0:
            lo, hi = a, b
            for _ in range(80):
                mid = math.sqrt(lo * hi)
                if (mag(lo) - 1.0) * (mag(mid) - 1.0) <= 0:
                    hi = mid
                else:
                    lo = mid
            wgc = math.sqrt(lo * hi)
            break
    pm = (180.0 + phase_at(wgc)) if wgc else float("inf")

    # --- phase crossover: first downward crossing of -180 deg
    wpc, gm_db = None, float("inf")
    for i in range(len(ws) - 1):
        if phases[i] > -180.0 >= phases[i + 1]:
            lo, hi = ws[i], ws[i + 1]
            for _ in range(80):
                mid = math.sqrt(lo * hi)
                if phase_at(mid) > -180.0:
                    lo = mid
                else:
                    hi = mid
            wpc = math.sqrt(lo * hi)
            m = mag(wpc)
            gm_db = 20 * math.log10(1.0 / m) if m > 1e-12 else float("inf")
            break

    return {"gm_db": gm_db, "pm_deg": pm, "wgc": wgc, "wpc": wpc}


# ---------------------------------------------------------------------
def lambda_tune(p: PlantParams, lam: float) -> dict:
    """PI by pole cancellation (lambda / IMC tuning)."""
    kp = p.tau / (p.K * lam)
    ki = kp / p.tau
    return {"kp": kp, "ki": ki, "kd": 0.0, "lam": lam}


def suggest_lambda(p: PlantParams, ctrl_dt: float) -> float:
    """A lambda that is fast but not reckless.

    Three constraints:
      * lambda >= 3*L     -- never outrun the transport delay
      * lambda >= 10*dt   -- never outrun the sample rate
      * lambda >= 0.3*tau -- do not demand 3x the plant's own bandwidth,
                             which just saturates the duty on every step
    """
    return max(3.0 * p.delay, 10.0 * ctrl_dt, 0.3 * p.tau)


def design_lead(p: PlantParams, kp: float, ki: float, kd: float,
                pm_target: float, ctrl_dt: float) -> dict | None:
    """Lead compensator to raise the phase margin to pm_target.

    Standard procedure:
      phi_needed = pm_target - pm_now + 8 deg  (the 8 deg covers the
                   crossover shift the compensator itself causes)
      alpha      = (1 - sin phi)/(1 + sin phi)
      place the centre frequency at the new crossover, where the
      compensator adds exactly phi and 1/sqrt(alpha) of gain.
    """
    m = margins(p.K, p.tau, p.delay, kp, ki, kd)
    if m["wgc"] is None or not math.isfinite(m["pm_deg"]):
        return None
    phi = pm_target - m["pm_deg"] + 8.0
    if phi <= 1.0:
        return {"needed": False, "pm_now": m["pm_deg"]}
    if phi > 60.0:
        phi = 60.0   # one lead section cannot reliably give more

    s = math.sin(math.radians(phi))
    alpha = (1 - s) / (1 + s)

    # new crossover: where |L| == sqrt(alpha), since the lead adds
    # 1/sqrt(alpha) of gain at its centre frequency
    target = math.sqrt(alpha)

    def mag(w):
        return abs(freq_response(p.K, p.tau, p.delay, kp, ki, kd, w))

    ws = [10 ** (-2 + 5 * i / 2000.0) for i in range(2001)]
    wm = None
    for a, b in zip(ws, ws[1:]):
        if (mag(a) - target) * (mag(b) - target) < 0:
            lo, hi = a, b
            for _ in range(80):
                mid = math.sqrt(lo * hi)
                if (mag(lo) - target) * (mag(mid) - target) <= 0:
                    hi = mid
                else:
                    lo = mid
            wm = math.sqrt(lo * hi)
            break
    if wm is None:
        return None

    z = wm * math.sqrt(alpha)        # zero
        # pole
    pole = wm / math.sqrt(alpha)

    # --- discretise  (s + z)/(s + p) * (p/z) by Tustin, unity DC gain
    T = ctrl_dt
    k = 2.0 / T
    # Gc(s) = (s+z)/(s+pole) * (pole/z)   -> unity gain at DC
    g = pole / z
    b0 = g * (k + z) / (k + pole)
    b1 = g * (-k + z) / (k + pole)
    a1 = (-k + pole) / (k + pole)

    return {"needed": True, "pm_now": m["pm_deg"], "pm_target": pm_target,
            "phi_deg": phi, "alpha": alpha, "zero": z, "pole": pole,
            "wm": wm, "b0": b0, "b1": b1, "a1": a1}


# ---------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", type=Path, default=None,
                    help="plant_params.json from sysid_fit.py (nominal if omitted)")
    ap.add_argument("--dt", type=float, default=0.02, help="control period [s]")
    ap.add_argument("--lam", type=float, default=None,
                    help="closed-loop time constant target [s]")
    ap.add_argument("--kgap", type=float, default=0.9, help="outer-loop gain")
    ap.add_argument("--pm-target", type=float, default=60.0,
                    help="phase margin the lead compensator should achieve")
    ap.add_argument("--kd", type=float, default=0.0,
                    help="add derivative action (usually unnecessary on a "
                         "first-order plant)")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--save-plot", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=Path("data/gains.json"))
    args = ap.parse_args()

    p = PlantParams.from_json(args.params) if args.params else PlantParams()

    print("=" * 70)
    print("PLANT")
    print("=" * 70)
    print(f"  {p.summary()}")
    print(f"                 {p.K:.4f}")
    print(f"  G(s) = ------------------- * exp(-{p.delay:.4f} s)")
    print(f"           {p.tau:.4f} s + 1")
    print(f"  open-loop bandwidth  1/tau = {1/p.tau:.2f} rad/s "
          f"({1/(2*math.pi*p.tau):.2f} Hz)")
    print(f"  control rate         {1/args.dt:.0f} Hz "
          f"({2*math.pi/args.dt:.1f} rad/s) "
          f"-> {(1/args.dt)*p.tau:.0f} samples per time constant")
    if p.delay > 0:
        print(f"  delay limits useful bandwidth to roughly "
              f"{1/(3*p.delay):.1f} rad/s")

    # ---- inner loop --------------------------------------------------
    lam = args.lam if args.lam is not None else suggest_lambda(p, args.dt)
    g = lambda_tune(p, lam)
    kp, ki, kd = g["kp"], g["ki"], args.kd

    print()
    print("=" * 70)
    print("INNER LOOP -- PI speed controller (lambda / IMC tuning)")
    print("=" * 70)
    print(f"  lambda (target closed-loop time constant) = {lam:.4f} s")
    if args.lam is None:
        print(f"    chosen as max(3L={3*p.delay:.3f}, 10*dt={10*args.dt:.3f}, "
              f"0.3*tau={0.3*p.tau:.3f})")
    print(f"  Kp = tau/(K*lambda) = {kp:.4f}")
    print(f"  Ki = Kp/tau         = {ki:.4f}")
    print(f"  Kd                  = {kd:.4f}")
    print()
    print(f"  Nominal closed loop (delay neglected): first order, "
          f"tau_cl = {lam:.3f} s")
    print(f"    -> rise time (10-90%) = {2.2*lam:.3f} s")
    print(f"    -> 2% settling time   = {3.9*lam:.3f} s")
    print(f"    -> overshoot          = 0% (pole-cancelled PI)")
    print(f"  Any overshoot you actually measure is the dead-zone, the")
    print(f"  delay, and duty saturation -- i.e. the nonlinearities. Say so.")

    m = margins(p.K, p.tau, p.delay, kp, ki, kd)
    print()
    print("  STABILITY MARGINS (with the delay included)")
    gm = m["gm_db"]
    if math.isfinite(gm) and m["wpc"]:
        print(f"    gain margin  : {gm:.1f} dB at {m['wpc']:.2f} rad/s")
    else:
        print(f"    gain margin  : infinite (phase never reaches -180 deg)")
    if m["wgc"]:
        print(f"    phase margin : {m['pm_deg']:.1f} deg at {m['wgc']:.2f} rad/s")
    else:
        print(f"    phase margin : (no gain crossover found)")
    verdict = ("comfortable" if m["pm_deg"] >= 50 else
               "acceptable" if m["pm_deg"] >= 35 else "TOO LOW -- expect ringing")
    print(f"    verdict      : {verdict}")
    print(f"    (rule of thumb: PM 45-60 deg, GM > 6 dB)")

    # ---- outer loop --------------------------------------------------
    print()
    print("=" * 70)
    print("OUTER LOOP -- constant time-gap policy")
    print("=" * 70)
    w_inner = 1.0 / lam
    w_outer = args.kgap
    sep = w_inner / w_outer if w_outer > 0 else float("inf")
    print(f"  the gap plant is a PURE INTEGRATOR (d/dt gap = v_lead - v_follow),")
    print(f"  so with gain K_gap the outer crossover sits at K_gap itself.")
    print(f"    inner bandwidth  ~ 1/lambda = {w_inner:.2f} rad/s")
    print(f"    outer bandwidth  ~ K_gap    = {w_outer:.2f} rad/s")
    print(f"    separation       = {sep:.1f}x")
    if sep >= 5:
        print("    -> good. The inner loop looks like unity gain to the outer one,")
        print("       so they can legitimately be designed independently.")
    elif sep >= 3:
        print("    -> marginal. The cascade assumption is starting to bend;")
        print("       expect some interaction between the loops.")
    else:
        print("    -> TOO CLOSE. The loops will fight each other. Either raise")
        print(f"       the inner bandwidth (smaller lambda) or drop K_gap below "
              f"{w_inner/5:.2f}.")
    print(f"  A pure integrator under proportional control has infinite gain")
    print(f"  margin and 90 deg phase margin on its own -- which is why the")
    print(f"  outer loop needs no integral term and stays stable so easily.")

    # ---- lead-lag ----------------------------------------------------
    print()
    print("=" * 70)
    print(f"LEAD COMPENSATOR -- target phase margin {args.pm_target:.0f} deg")
    print("=" * 70)
    lead = design_lead(p, kp, ki, kd, args.pm_target, args.dt)
    ll = None
    if lead is None:
        print("  could not design a lead section for this plant/gain combination")
    elif not lead["needed"]:
        print(f"  not needed -- phase margin is already {lead['pm_now']:.1f} deg,")
        print(f"  at or above the {args.pm_target:.0f} deg target.")
        print(f"  Leave ENABLE_LEADLAG = 0. Adding a compensator you do not need")
        print(f"  is a worse answer than explaining why you do not need one.")
    else:
        print(f"  current PM     : {lead['pm_now']:.1f} deg")
        print(f"  phase to add   : {lead['phi_deg']:.1f} deg (incl. 8 deg allowance)")
        print(f"  alpha          : {lead['alpha']:.4f}")
        print(f"  zero at        : {lead['zero']:.3f} rad/s")
        print(f"  pole at        : {lead['pole']:.3f} rad/s")
        print(f"  centre freq    : {lead['wm']:.3f} rad/s")
        print()
        print(f"             s + {lead['zero']:.3f}      {lead['pole']/lead['zero']:.3f}")
        print(f"  Gc(s) = --------------- * (normalised to unity DC gain)")
        print(f"             s + {lead['pole']:.3f}")
        print()
        print(f"  Tustin-discretised at dt = {args.dt:g} s, paste into config.h:")
        print(f"    #define ENABLE_LEADLAG   1")
        print(f"    #define LL_B0   {lead['b0']:+.6f}f")
        print(f"    #define LL_B1   {lead['b1']:+.6f}f")
        print(f"    #define LL_A1   {lead['a1']:+.6f}f")
        ll = lead

    # ---- the paste block ---------------------------------------------
    print()
    print("=" * 70)
    print("PASTE INTO firmware/common/config.h, THEN RUN tools/sync_common.py")
    print("=" * 70)
    print(f"  #define PID_KP                 {kp:.4f}f")
    print(f"  #define PID_KI                 {ki:.4f}f")
    print(f"  #define PID_KD                 {kd:.4f}f")
    print(f"  #define K_GAP                  {args.kgap:.4f}f")
    print(f"  #define PLANT_K                {p.K:.4f}f")
    print(f"  #define PLANT_TAU              {p.tau:.4f}f")
    print(f"  #define PLANT_DELAY_S          {p.delay:.4f}f")
    print(f"  #define PLANT_DEADZONE         {p.deadzone:.4f}f")

    out = {
        "plant": {"K": p.K, "tau": p.tau, "delay": p.delay,
                  "deadzone": p.deadzone},
        "inner": {"kp": kp, "ki": ki, "kd": kd, "lambda": lam},
        "margins": m,
        "outer": {"k_gap": args.kgap, "separation": sep},
        "leadlag": ll,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=2, default=str), encoding="utf-8")
    print(f"\nsaved -> {args.out}")
    print(f"\nverify on the model before touching hardware:")
    print(f"  python python/scripts/sim_only.py --kp {kp:.3f} --ki {ki:.3f} "
          f"--compare --fair")

    if args.plot or args.save_plot:
        do_plots(p, kp, ki, kd, lam, args)
    return 0


# ---------------------------------------------------------------------
def do_plots(p: PlantParams, kp, ki, kd, lam, args) -> None:
    try:
        import matplotlib
        if args.save_plot:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("\nmatplotlib/numpy not installed -- skipping plots")
        return

    try:
        import control as ct
        have_control = True
    except ImportError:
        have_control = False
        print("\nnote: python-control not installed, drawing Bode + step only")
        print("      (pip install control  -> adds the root locus)")

    fig = plt.figure(figsize=(12, 9))
    w = np.logspace(-2, 3, 2000)
    Lw = np.array([freq_response(p.K, p.tau, p.delay, kp, ki, kd, wi) for wi in w])

    # --- Bode magnitude
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.semilogx(w, 20 * np.log10(np.abs(Lw)), lw=1.6)
    ax1.axhline(0, color="k", lw=0.8)
    ax1.set_ylabel("|L| [dB]")
    ax1.set_title("Loop gain -- magnitude")
    ax1.grid(which="both", alpha=0.3)

    # --- Bode phase
    ax2 = fig.add_subplot(2, 2, 3)
    ph = np.degrees(np.unwrap(np.angle(Lw)))
    ax2.semilogx(w, ph, lw=1.6)
    ax2.axhline(-180, color="r", ls="--", lw=0.9)
    ax2.set_ylabel("phase [deg]")
    ax2.set_xlabel("frequency [rad/s]")
    ax2.set_title("Loop gain -- phase")
    ax2.grid(which="both", alpha=0.3)

    m = margins(p.K, p.tau, p.delay, kp, ki, kd)
    if m["wgc"]:
        for ax in (ax1, ax2):
            ax.axvline(m["wgc"], color="g", ls=":", lw=1.2)
        ax2.annotate(f"PM = {m['pm_deg']:.0f} deg", (m["wgc"], -180),
                     textcoords="offset points", xytext=(8, 14),
                     color="g", fontsize=9)

    # --- root locus (needs python-control)
    ax3 = fig.add_subplot(2, 2, 2)
    if have_control:
        # delay-free plant with PI, locus in Kp
        num = [p.K * (1.0 + 0.0), p.K * ki / max(kp, 1e-9)]
        sys_ol = ct.tf([p.K, p.K * ki / max(kp, 1e-9)], [p.tau, 1.0, 0.0])
        try:
            ct.root_locus(sys_ol, ax=ax3, plot=True, grid=False)
        except TypeError:
            ct.root_locus(sys_ol, ax=ax3)
        ax3.set_title("Root locus (PI, delay neglected)")
    else:
        ax3.text(0.5, 0.5, "pip install control\nfor the root locus",
                 ha="center", va="center", transform=ax3.transAxes)
        ax3.set_title("Root locus")
    ax3.grid(alpha=0.3)

    # --- closed-loop step, simulated on the REAL nonlinear plant
    ax4 = fig.add_subplot(2, 2, 4)
    from v2vacc.controller import PID, Gains
    from v2vacc.plant import FirstOrderPlant
    dt = args.dt
    pid = PID(Gains(kp=kp, ki=ki, kd=kd), dt)
    plant = FirstOrderPlant(p, dt)
    sp = 0.35
    ts, vs = [], []
    for k in range(int(4.0 / dt)):
        u = pid.step(sp, plant.v)
        plant.step(u)
        ts.append(k * dt)
        vs.append(plant.v)
    ax4.plot(ts, vs, lw=1.8, label="nonlinear sim (dead-zone, delay, sat.)")
    ax4.axhline(sp, color="k", ls="--", lw=0.9, label="setpoint")
    ideal = [sp * (1 - math.exp(-t / lam)) for t in ts]
    ax4.plot(ts, ideal, ls=":", color="#d62728", lw=1.5,
             label=f"linear prediction (tau={lam:.2f}s)")
    ax4.set_xlabel("time [s]")
    ax4.set_ylabel("speed [m/s]")
    ax4.set_title("Closed-loop step: prediction vs nonlinear reality")
    ax4.legend(fontsize=8)
    ax4.grid(alpha=0.3)

    fig.suptitle("V2V Predictive ACC -- inner-loop design "
                 f"(Kp={kp:.2f}, Ki={ki:.2f}, lambda={lam:.3f}s)", fontsize=12)
    fig.tight_layout()
    if args.save_plot:
        fig.savefig(args.save_plot, dpi=150)
        print(f"plots saved -> {args.save_plot}")
    else:
        plt.show()


if __name__ == "__main__":
    raise SystemExit(main())
