#!/usr/bin/env python3
"""
state_space_lqr.py -- state-space modelling and LQR design for the ACC loop.

This is the advanced-control deliverable. It produces, in one run:

  1. the 3-state model, built from the IDENTIFIED plant
  2. controllability and observability, with the two instructive failures
  3. an LQR design from Bryson weights (requirements, not guesswork)
  4. the Kalman-inequality margin check (LQR's >=60 deg PM guarantee),
     verified numerically rather than merely asserted
  5. a like-for-like LQR vs cascaded-PID comparison on the SAME nonlinear
     plant, with saturation and anti-windup active for both
  6. discrete-time gains at the firmware control rate, ready to paste
  7. plots for the report

    python python/scripts/state_space_lqr.py --plot
    python python/scripts/state_space_lqr.py --params data/plant_params.json
    python python/scripts/state_space_lqr.py --sweep

WHAT TO SAY ABOUT IT
--------------------
The cascaded PID was designed loop-by-loop in the frequency domain, which
works but is two SISO designs stapled together with a bandwidth-separation
rule we imposed on ourselves. The state-space formulation treats spacing
error and relative velocity as coupled states of one system and lets LQR
pick the gains by minimising a cost we can actually defend. Both are
implemented; the comparison is the interesting part, and it does not
automatically favour LQR -- see the output.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from v2vacc import PlantParams  # noqa: E402
from v2vacc.statespace import ACCStateSpace  # noqa: E402
from v2vacc.controller import (FollowerController, Gains, GapPolicy,  # noqa: E402
                               LinkState, clamp)
from v2vacc.plant import FirstOrderPlant, VehicleKinematics  # noqa: E402
from v2vacc.twin import lead_scenario  # noqa: E402


# =====================================================================
#  Kalman inequality:  |1 + L(jw)| >= 1  for an LQR state-feedback loop.
#  It is what guarantees GM in (0.5, inf) and PM >= 60 deg at the plant
#  input. Worth VERIFYING rather than quoting -- if the number comes out
#  below 1 something is wrong with the design or the model.
# =====================================================================
def kalman_margins(A, B, K, n_w: int = 4000):
    ws = np.logspace(-3, 3, n_w)
    n = A.shape[0]
    I = np.eye(n)
    best = np.inf
    w_at = None
    for w in ws:
        try:
            L = float((K @ np.linalg.solve(1j * w * I - A, B)).real) \
                if False else complex((K @ np.linalg.solve(1j * w * I - A, B))[0, 0])
        except np.linalg.LinAlgError:
            continue
        m = abs(1.0 + L)
        if m < best:
            best, w_at = m, w
    # Kalman inequality -> these are the guaranteed bounds
    gm_lower = 1.0 / (1.0 + best) if best < 1 else 0.5
    pm_guaranteed = 2.0 * math.degrees(math.asin(min(1.0, best / 2.0)))
    return {"min_return_difference": best, "w": w_at,
            "pm_guaranteed_deg": pm_guaranteed, "gm_lower": gm_lower}


# =====================================================================
#  Like-for-like comparison on the SAME nonlinear plant.
#  Both controllers drive a FirstOrderPlant with dead-zone, delay and
#  duty saturation, with gap kinematics integrated identically. Comparing
#  an LQR on a clean linear model against a PID on a nonlinear one would
#  flatter the LQR and prove nothing.
# =====================================================================
def run_lqr_nonlinear(ss: ACCStateSpace, K, p: PlantParams, duration=45.0,
                      dt=0.02, initial_gap=1.00):
    K = np.asarray(K, float).ravel()
    plant = FirstOrderPlant(p, dt)
    kin = VehicleKinematics(gap=initial_gap, gap_max=2.5)
    lead_plant = FirstOrderPlant(p, dt)
    lead_pid_i = 0.0

    integ = 0.0
    out = {k: [] for k in ("t", "gap", "gap_des", "e", "v_f", "v_l", "u")}

    for k in range(int(duration / dt)):
        t = k * dt
        # lead vehicle, same profile and same plant as the PID comparison
        cmd, _ = lead_scenario(t)
        lead_pid_i += (cmd - lead_plant.v) * dt * 6.0
        u_l = clamp(3.0 * (cmd - lead_plant.v) + lead_pid_i, -1.0, 1.0)
        v_l = lead_plant.step(u_l)

        v_f = plant.v
        e = kin.gap - ss.d_standstill - ss.t_gap * v_f
        v_r = v_l - v_f

        u_unsat = -(K[0] * e + K[1] * v_r + K[2] * integ)
        u = clamp(u_unsat, -1.0, 1.0)
        if u == u_unsat:                      # anti-windup, same rule as the PID
            integ += e * dt

        plant.step(u)
        kin.step(v_l, plant.v, dt)

        out["t"].append(t)
        out["gap"].append(kin.gap)
        out["gap_des"].append(ss.d_standstill + ss.t_gap * v_f)
        out["e"].append(e)
        out["v_f"].append(plant.v)
        out["v_l"].append(v_l)
        out["u"].append(u)
    return out


def run_pid_nonlinear(p: PlantParams, duration=45.0, dt=0.02, initial_gap=1.00,
                      t_gap=1.5, d0=0.20):
    ctrl = FollowerController(dt=dt, gap_policy=GapPolicy(
        t_gap=t_gap, t_gap_degraded=t_gap, d_standstill=d0, k_gap=0.9, v_max=0.60))
    plant = FirstOrderPlant(p, dt)
    kin = VehicleKinematics(gap=initial_gap, gap_max=2.5)
    lead_plant = FirstOrderPlant(p, dt)
    lead_pid_i = 0.0

    out = {k: [] for k in ("t", "gap", "gap_des", "e", "v_f", "v_l", "u")}
    for k in range(int(duration / dt)):
        t = k * dt
        cmd, hazard = lead_scenario(t)
        lead_pid_i += (cmd - lead_plant.v) * dt * 6.0
        u_l = clamp(3.0 * (cmd - lead_plant.v) + lead_pid_i, -1.0, 1.0)
        v_l = lead_plant.step(u_l)

        res = ctrl.step(gap_raw=kin.gap, v_follow=plant.v, v_lead_v2v=v_l,
                        lead_braking=False, lead_hazard=False,
                        link=LinkState.NOMINAL, filter_range=False)
        plant.step(res.duty)
        kin.step(v_l, plant.v, dt)

        out["t"].append(t)
        out["gap"].append(kin.gap)
        out["gap_des"].append(res.d_des)
        out["e"].append(kin.gap - res.d_des)
        out["v_f"].append(plant.v)
        out["v_l"].append(v_l)
        out["u"].append(res.duty)
    return out


def metrics(run: dict, skip_s: float = 5.0) -> dict:
    """Performance metrics, EXCLUDING the startup transient.

    The simulation begins with the follower stationary at a 1 m gap while
    the desired gap is only d0 = 20 cm, so the initial spacing error is
    80 cm by construction. Including it makes `max |e|` report the initial
    condition for every controller -- identical, and meaningless. What we
    care about is tracking once the thing is running, so the first
    `skip_s` seconds are dropped.
    """
    i0 = next((i for i, t in enumerate(run["t"]) if t >= skip_s), 0)
    e = run["e"][i0:]
    u = run["u"][i0:]
    gap = run["gap"][i0:]
    n = max(1, len(e))
    return {
        "max_abs_e_cm": max(abs(x) for x in e) * 100.0,
        "rms_e_cm": math.sqrt(sum(x * x for x in e) / n) * 100.0,
        "min_gap_cm": min(gap) * 100.0,
        "peak_duty": max(abs(x) for x in u),
        "sat_pct": 100.0 * sum(1 for x in u if abs(x) > 0.999) / n,
        "control_effort": sum(x * x for x in u) / n,
    }


# =====================================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", type=Path, default=None,
                    help="plant_params.json from sysid_fit.py")
    ap.add_argument("--dt", type=float, default=0.02, help="control period [s]")
    ap.add_argument("--tgap", type=float, default=1.50)
    ap.add_argument("--d0", type=float, default=0.20)
    ap.add_argument("--e-max", type=float, default=0.10, help="Bryson: tolerable spacing error [m]")
    ap.add_argument("--vr-max", type=float, default=0.15, help="Bryson: tolerable speed mismatch [m/s]")
    ap.add_argument("--i-max", type=float, default=0.30, help="Bryson: tolerable integral state")
    ap.add_argument("--sweep", action="store_true", help="sweep the Bryson weights")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--save-plot", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=Path("data/lqr_design.json"))
    args = ap.parse_args()

    p = PlantParams.from_json(args.params) if args.params else PlantParams()
    ss = ACCStateSpace(p, t_gap=args.tgap, d_standstill=args.d0)

    bar = "=" * 72
    print(bar)
    print("1. STATE-SPACE MODEL")
    print(bar)
    print(f"  plant: {p.summary()}")
    print(f"  time gap Th = {args.tgap:.2f} s,  standstill gap d0 = {args.d0:.2f} m")
    print()
    print("  states   x = [ e, v_r, integral(e) ]")
    print("           e   = d - d0 - Th*v_f     spacing error")
    print("           v_r = v_lead - v_follow   relative velocity")
    print("  input    u = normalised motor duty")
    print("  disturb. w = v_lead")
    print()
    for name, M in (("A", ss.A), ("B", ss.B), ("E", ss.E)):
        rows = np.atleast_2d(M)
        for i, row in enumerate(rows):
            lead_s = f"  {name} = " if i == 0 else "      "
            print(lead_s + "[ " + "  ".join(f"{v:8.4f}" for v in row) + " ]")
        print()
    print("  NOTE on the choice of x2. LQR drives the STATES to zero, so if x2")
    print("  were the follower speed the cost would contain q*v_f^2 and the")
    print("  optimiser would read that as 'standing still is ideal'. At")
    print("  equilibrium v_f -> v_lead, which is not zero; relative velocity")
    print("  is. Using v_r makes this a genuine regulator problem.")

    print()
    print(bar)
    print("2. STRUCTURAL PROPERTIES")
    print(bar)
    ol = ss.open_loop_poles()
    print(f"  open-loop poles: {np.round(ol, 4)}")
    print(f"    two at the origin -- the gap integrator and the added integral")
    print(f"    state -- and one at -1/tau = {-1/p.tau:.2f} from the motor.")
    print()
    c = ss.controllability()
    print(f"  CONTROLLABLE      : {c['controllable']}  (rank {c['rank']}/{c['n']})")
    print(f"    every mode can be driven by the motor, so the poles can be")
    print(f"    placed anywhere and the LQR problem is well posed.")
    print()
    o = ss.observability()
    print(f"  OBSERVABLE (full) : {o['observable']}  (rank {o['rank']}/{o['n']})")
    og = ss.observability(ss.C_gap_only())
    print(f"  OBSERVABLE (gap only) : {og['observable']}  (rank {og['rank']}/{og['n']})")
    print(f"    Instructive failure. Measuring ONLY the gap loses a state, so a")
    print(f"    gap-only ACC would need an observer to estimate relative")
    print(f"    velocity -- estimator lag on top of filter lag, exactly where")
    print(f"    reaction time matters. We measure v_f on the encoder and v_lead")
    print(f"    over V2V, so we have full state feedback with no observer at")
    print(f"    all. That is the V2V link paying off a second time.")
    print()
    c4 = ss.controllability_with_lead_as_state()
    print(f"  with v_lead as a 4th STATE : controllable = {c4['controllable']} "
          f"(rank {c4['rank']}/4)")
    print(f"    Also instructive, and a modelling trap worth showing: no amount")
    print(f"    of follower throttle changes what the lead vehicle does, so that")
    print(f"    mode is uncontrollable. v_lead is a DISTURBANCE, not a state.")

    # ---- design ------------------------------------------------------
    Q, R = ss.bryson_weights(e_max=args.e_max, vr_max=args.vr_max, i_max=args.i_max)
    r = ss.design(Q, R)
    K = np.asarray(r.K).ravel()

    print()
    print(bar)
    print("3. LQR DESIGN")
    print(bar)
    print(f"  Bryson weights from requirements:")
    print(f"    tolerable spacing error   e_max  = {args.e_max:.3f} m")
    print(f"    tolerable speed mismatch  vr_max = {args.vr_max:.3f} m/s")
    print(f"    tolerable integral state  i_max  = {args.i_max:.3f}")
    print(f"    Q = diag(1/e_max^2, 1/vr_max^2, 1/i_max^2) = "
          f"diag({Q[0,0]:.1f}, {Q[1,1]:.1f}, {Q[2,2]:.1f})")
    print(f"    R = [{R[0,0]:.1f}]")
    print(f"  J = integral ( x'Qx + u'Ru ) dt,  minimised subject to the model")
    print()
    print(f"  u = -K x   with  K = [ {K[0]:.4f}  {K[1]:.4f}  {K[2]:.4f} ]")
    print(f"             i.e. u = {-K[0]:+.4f}*e {-K[1]:+.4f}*v_r {-K[2]:+.4f}*int(e)")
    print()
    print(f"  closed-loop poles: {np.round(r.poles, 4)}")
    for wn, zeta, _ in r.damping():
        print(f"    |p| = {wn:7.3f} rad/s   zeta = {zeta:6.3f}")
    print(f"  slowest-pole 2% settling estimate: {r.settling_time():.2f} s")

    km = kalman_margins(ss.A, ss.B, r.K)
    print()
    print("  GUARANTEED MARGINS (Kalman inequality, verified numerically)")
    print(f"    min |1 + L(jw)| = {km['min_return_difference']:.4f} "
          f"at w = {km['w']:.3f} rad/s")
    print(f"    (the minimum sits at the top of the swept band because L(jw)")
    print(f"     rolls off to zero, so |1+L| -> 1 asymptotically. That is the")
    print(f"     equality case of the inequality, not a near-violation.)")
    if km["min_return_difference"] >= 0.999:
        print(f"    |1+L| >= 1 holds, so the LQR guarantees apply:")
        print(f"      gain margin  : 0.5 to infinity  (-6 dB to +inf)")
        print(f"      phase margin : at least 60 degrees")
        print(f"    This is a PROPERTY of LQR state feedback, not something we")
        print(f"    tuned for -- and it is why LQR is attractive for a safety")
        print(f"    function. Compare the PID's margins from design_control.py.")
    else:
        print(f"    WARNING: |1+L| < 1. The guarantee does not hold here, which")
        print(f"    means the model or the design is off -- investigate before")
        print(f"    quoting any margin.")

    # ---- discrete gains ---------------------------------------------
    Ad, Bd, Ed = ss.discretise(args.dt, with_disturbance=True)
    print()
    print(bar)
    print(f"4. DISCRETE-TIME FORM AT {1/args.dt:.0f} Hz")
    print(bar)
    print("  Zero-order hold. The gains run on a 50 Hz ESP32 loop, not in")
    print("  continuous time, so the discretised model is what actually applies.")
    print()
    for i, row in enumerate(Ad):
        print(("  Ad = " if i == 0 else "       ")
              + "[ " + "  ".join(f"{v:9.5f}" for v in row) + " ]")
    print("  Bd = [ " + "  ".join(f"{v:9.5f}" for v in Bd.ravel()) + " ]")
    print("  Ed = [ " + "  ".join(f"{v:9.5f}" for v in Ed.ravel()) + " ]")
    print()
    print("  paste into firmware/common/config.h to run LQR on the vehicle:")
    print(f"    #define LQR_K_E      {K[0]:+.5f}f")
    print(f"    #define LQR_K_VR     {K[1]:+.5f}f")
    print(f"    #define LQR_K_INT    {K[2]:+.5f}f")

    # ---- comparison --------------------------------------------------
    print()
    print(bar)
    print("5. LQR vs CASCADED PID  (same nonlinear plant, same scenario)")
    print(bar)
    print("  Both run against FirstOrderPlant with dead-zone, transport delay")
    print("  and duty saturation, with anti-windup active. Comparing an LQR on")
    print("  a clean linear model against a PID on a nonlinear one would")
    print("  flatter the LQR and prove nothing.")
    print()
    lqr_run = run_lqr_nonlinear(ss, r.K, p, dt=args.dt)
    pid_run = run_pid_nonlinear(p, dt=args.dt, t_gap=args.tgap, d0=args.d0)
    m_lqr, m_pid = metrics(lqr_run), metrics(pid_run)

    print("  (startup transient excluded: the run begins with an 80 cm")
    print("   spacing error by construction, which is not a tracking result)")
    print()
    print(f"  {'metric':<28}{'LQR':>12}{'cascaded PID':>16}")
    print("  " + "-" * 56)
    rows = [("max |spacing error|", "max_abs_e_cm", "cm", 2),
            ("RMS spacing error", "rms_e_cm", "cm", 2),
            ("minimum gap", "min_gap_cm", "cm", 1),
            ("peak |duty|", "peak_duty", "", 3),
            ("duty saturated", "sat_pct", "%", 1),
            ("control effort (mean u^2)", "control_effort", "", 4)]
    for label, key, unit, dp in rows:
        print(f"  {label:<28}{m_lqr[key]:>11.{dp}f}{unit:<1}"
              f"{m_pid[key]:>15.{dp}f}{unit:<1}")
    print()
    better_e = "LQR" if m_lqr["rms_e_cm"] < m_pid["rms_e_cm"] else "PID"
    better_u = "LQR" if m_lqr["control_effort"] < m_pid["control_effort"] else "PID"
    print(f"  tighter gap holding : {better_e}")
    print(f"  less control effort : {better_u}")
    print()
    print("  Read this honestly. LQR optimises the cost IT WAS GIVEN; if the")
    print("  PID wins on a metric, that metric was not in the cost. The real")
    print("  argument for LQR here is not that it beats a well-tuned PID on")
    print("  tracking -- it is that the gains come from stated requirements")
    print("  rather than loop-by-loop tuning, it handles the coupling between")
    print("  spacing and relative velocity directly, and it carries the")
    print("  margin guarantee above for free.")

    # ---- sweep -------------------------------------------------------
    if args.sweep:
        print()
        print(bar)
        print("6. WEIGHT SWEEP -- the design trade-off, quantified")
        print(bar)
        print(f"  {'e_max':>7}{'vr_max':>8}{'i_max':>7} | {'K_e':>9}{'K_vr':>8}{'K_int':>8}"
              f" | {'max|e|':>8}{'peak u':>8}{'sat':>7}")
        print("  " + "-" * 72)
        for em, vm, im in [(0.20, 0.30, 0.60), (0.10, 0.15, 0.30),
                           (0.05, 0.10, 0.20), (0.03, 0.06, 0.10),
                           (0.02, 0.04, 0.06)]:
            Qs, Rs = ss.bryson_weights(e_max=em, vr_max=vm, i_max=im)
            rs = ss.design(Qs, Rs)
            run = run_lqr_nonlinear(ss, rs.K, p, dt=args.dt)
            ms = metrics(run)
            g = np.asarray(rs.K).ravel()
            print(f"  {em:>7.3f}{vm:>8.3f}{im:>7.3f} | {g[0]:>9.2f}{g[1]:>8.2f}"
                  f"{g[2]:>8.2f} | {ms['max_abs_e_cm']:>7.2f}cm"
                  f"{ms['peak_duty']:>8.2f}{ms['sat_pct']:>6.1f}%")
        print()
        print("  Tightening the weights raises the gains and shrinks the error")
        print("  until the duty saturates -- after which further tightening")
        print("  makes things WORSE, because a saturated actuator cannot")
        print("  deliver what the design assumed. That knee is the honest")
        print("  limit of this hardware, and it is set by the motor, not by")
        print("  the controller.")

    # ---- save --------------------------------------------------------
    report = {
        "plant": {"K": p.K, "tau": p.tau, "delay": p.delay, "deadzone": p.deadzone},
        "policy": {"t_gap": args.tgap, "d_standstill": args.d0},
        "A": ss.A.tolist(), "B": ss.B.tolist(), "E": ss.E.tolist(),
        "controllable": c["controllable"], "observable_full": o["observable"],
        "observable_gap_only": og["observable"],
        "bryson": {"e_max": args.e_max, "vr_max": args.vr_max, "i_max": args.i_max},
        "Q": Q.tolist(), "R": R.tolist(),
        "K": K.tolist(),
        "closed_loop_poles": [[float(x.real), float(x.imag)] for x in r.poles],
        "settling_s": r.settling_time(),
        "kalman": {k: (float(v) if v is not None else None) for k, v in km.items()},
        "discrete": {"dt": args.dt, "Ad": Ad.tolist(),
                     "Bd": Bd.tolist(), "Ed": Ed.tolist()},
        "comparison": {"lqr": m_lqr, "pid": m_pid},
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\nsaved -> {args.out}")

    if args.plot or args.save_plot:
        do_plots(ss, r, lqr_run, pid_run, args)
    return 0


# =====================================================================
def do_plots(ss, r, lqr_run, pid_run, args):
    try:
        import matplotlib
        if args.save_plot:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -- skipping plots")
        return

    fig = plt.figure(figsize=(13, 9))

    # --- pole map
    ax = fig.add_subplot(2, 2, 1)
    ol = ss.open_loop_poles()
    ax.scatter(ol.real, ol.imag, marker="x", s=90, c="#888", label="open loop")
    ax.scatter(np.asarray(r.poles).real, np.asarray(r.poles).imag,
               marker="o", s=70, facecolors="none", edgecolors="#d62728",
               linewidths=1.8, label="LQR closed loop")
    ax.axvline(0, color="k", lw=0.8)
    ax.axhline(0, color="k", lw=0.8)
    # symlog, because the fast actuator pole is two decades from the slow
    # spacing poles and a linear axis hides the ones that matter
    ax.set_xscale("symlog", linthresh=0.1)
    ax.set_xlabel("Re  (symlog)")
    ax.set_ylabel("Im")
    ax.set_title("Pole map: open loop vs LQR")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8, loc="upper left")
    for pz in np.asarray(r.poles):
        ax.annotate(f"{pz.real:.2f}", (pz.real, pz.imag),
                    textcoords="offset points", xytext=(0, 10),
                    fontsize=7, ha="center", color="#d62728")

    # --- gap
    ax = fig.add_subplot(2, 2, 2)
    ax.plot(lqr_run["t"], lqr_run["gap"], lw=1.8, color="#d62728", label="LQR gap")
    ax.plot(pid_run["t"], pid_run["gap"], lw=1.8, color="#1f77b4", label="PID gap")
    ax.plot(lqr_run["t"], lqr_run["gap_des"], ls=":", lw=1.2, color="#d62728",
            label="LQR desired")
    ax.plot(pid_run["t"], pid_run["gap_des"], ls=":", lw=1.2, color="#1f77b4",
            label="PID desired")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("gap [m]")
    ax.set_title("Gap tracking on the nonlinear plant")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=2)

    # --- spacing error
    ax = fig.add_subplot(2, 2, 3)
    ax.plot(lqr_run["t"], [x * 100 for x in lqr_run["e"]], lw=1.6,
            color="#d62728", label="LQR")
    ax.plot(pid_run["t"], [x * 100 for x in pid_run["e"]], lw=1.6,
            color="#1f77b4", label="cascaded PID")
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("spacing error [cm]")
    ax.set_title("Spacing error")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # --- duty
    ax = fig.add_subplot(2, 2, 4)
    ax.plot(lqr_run["t"], lqr_run["u"], lw=1.4, color="#d62728", label="LQR")
    ax.plot(pid_run["t"], pid_run["u"], lw=1.4, color="#1f77b4", label="cascaded PID")
    ax.axhline(1.0, ls="--", color="k", lw=0.8)
    ax.axhline(-1.0, ls="--", color="k", lw=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("duty")
    ax.set_title("Control effort (dashed = saturation)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    K = np.asarray(r.K).ravel()
    fig.suptitle("State-space / LQR design  --  "
                 f"K = [{K[0]:.2f}, {K[1]:.2f}, {K[2]:.2f}]", fontsize=12)
    fig.tight_layout()
    if args.save_plot:
        fig.savefig(args.save_plot, dpi=150)
        print(f"plots saved -> {args.save_plot}")
    else:
        plt.show()


if __name__ == "__main__":
    raise SystemExit(main())
