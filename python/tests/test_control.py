"""
test_control.py -- pins the behaviour of the control stack.

Run:   python -m pytest python/tests -q
       (or just: python python/tests/test_control.py)

These are not "does it import" tests. Each one checks a property that, if
it broke, would produce a plausible-looking but wrong result -- the kind
of bug that survives a demo and then shows up in the report as an
unexplainable discrepancy.

The PARITY tests matter most. controller.py is a hand-maintained mirror
of the firmware headers; if someone changes a gain or a filter in one and
not the other, the digital twin silently starts predicting a controller
that is not the one running on the car. These tests encode the numbers
both sides must agree on.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from v2vacc.controller import (PID, Gains, LowPass, Median3, Differentiator,
                               LeadLag, GapPolicy, BrakePolicy,
                               FollowerController, LinkState, Mode,
                               rate_limit, clamp)
from v2vacc.plant import FirstOrderPlant, PlantParams, VehicleKinematics
from v2vacc.twin import ClosedLoopTwin, compare, lead_scenario
from v2vacc.telemetry import parse_line, FollowerRecord, LeadRecord
from v2vacc.statespace import (ACCStateSpace, lqr, solve_care, expm,
                               controllability_matrix, matrix_rank)


# =====================================================================
#  Filters
# =====================================================================
def test_median3_kills_a_single_outlier():
    """The whole point of the median stage: one bad ultrasonic ping must
    not reach the differentiator, because differentiating it produces a
    phantom closing rate and a phantom brake."""
    m = Median3()
    for x in (1.00, 1.00, 1.00):
        m.step(x)
    out = m.step(9.99)          # a missed echo reads as max range
    assert abs(out - 1.00) < 1e-9, f"outlier leaked through: {out}"


def test_lowpass_dc_gain_is_unity():
    lp = LowPass(4.0, 0.02)
    for _ in range(2000):
        y = lp.step(0.7)
    assert abs(y - 0.7) < 1e-6


def test_lowpass_corner_frequency():
    """-3 dB at fc, within 5%. A filter whose corner is wrong by 2x
    silently changes how much the closing rate lags reality."""
    fc, dt = 4.0, 0.001
    lp = LowPass(fc, dt)
    w = 2 * math.pi * fc
    n = int(6.0 / dt)
    peak = 0.0
    for k in range(n):
        y = lp.step(math.sin(w * k * dt))
        if k > n // 2:
            peak = max(peak, abs(y))
    assert 0.65 < peak < 0.76, f"gain at fc = {peak:.3f}, expected ~0.707"


def test_differentiator_recovers_a_known_slope():
    d = Differentiator(20.0, 0.001)
    slope, out = 2.5, 0.0
    for k in range(4000):
        out = d.step(slope * k * 0.001)
    assert abs(out - slope) < 0.05, f"got {out}, expected {slope}"


def test_leadlag_unity_dc_gain():
    """b0+b1 and 1+a1 must give DC gain 1, or the compensator shifts the
    operating point and the car cruises at the wrong speed."""
    b0, b1, a1 = 1.2, -0.9, -0.85
    f = LeadLag(b0, b1, a1)
    for _ in range(5000):
        y = f.step(1.0)
    expected = (b0 + b1) / (1 + a1)
    assert abs(y - expected) < 1e-6


# =====================================================================
#  PID
# =====================================================================
def test_pid_no_derivative_kick_on_setpoint_step():
    """Derivative on MEASUREMENT, not error. A setpoint step must not
    produce a derivative spike -- that would slam the motor driver."""
    pid = PID(Gains(kp=1.0, ki=0.0, kd=1.0, n_deriv=50.0), 0.02)
    pid.step(0.0, 0.0)
    u = pid.step(1.0, 0.0)       # big setpoint step, measurement unchanged
    assert abs(pid.d_term) < 1e-9, f"derivative kicked: {pid.d_term}"
    assert abs(u - 1.0) < 1e-9    # pure proportional response


def test_pid_derivative_responds_to_measurement_change():
    pid = PID(Gains(kp=0.0, ki=0.0, kd=1.0, n_deriv=50.0), 0.02)
    pid.step(0.0, 0.0)
    pid.step(0.0, 0.1)           # measurement rising -> derivative opposes
    assert pid.d_term < 0.0


def test_pid_antiwindup_bounds_the_integrator():
    """Without back-calculation the integrator charges all through a long
    saturation. Here the setpoint is unreachable, so the duty sits pinned
    at +1 and a naive integrator would grow without bound."""
    pid = PID(Gains(kp=1.0, ki=10.0, kd=0.0, u_max=1.0, u_min=-1.0), 0.02)
    for _ in range(500):          # 10 s of unreachable setpoint
        pid.step(10.0, 0.0)
    assert pid.integ < 5.0, f"integrator wound up to {pid.integ}"

    # Analytic check: with kaw = ki/kp the integrator settles at a fixed
    # point, it does not keep climbing. (Note that feeding e == 0 forever
    # would NOT discharge it -- an integrator with zero input holds its
    # value. Recovery only happens once the measurement overshoots the
    # setpoint and the error changes sign, which is what the closed-loop
    # test below actually exercises.)
    settled = pid.integ
    for _ in range(200):
        pid.step(10.0, 0.0)
    assert abs(pid.integ - settled) < 0.05, "integrator still climbing"


def test_antiwindup_limits_closed_loop_undershoot():
    """The behaviour that actually matters on the car: after a long
    saturation (a sustained hard brake, or a setpoint the motor cannot
    reach), the speed must converge cleanly when the demand comes back
    into range.

    Note the plant STARTS this phase above the target -- it is pinned at
    its top speed -- so there is no overshoot to measure on the way down.
    The windup symptom here is UNDERSHOOT: a charged integrator keeps
    pushing the duty the wrong way past the crossing point, so the speed
    sails through the setpoint and has to come back. Anti-windup is
    working when that dip is small and the response settles monotonically.
    """
    p = PlantParams(K=0.6, tau=0.25, delay=0.0, deadzone=0.0, v_max=0.6)
    plant = FirstOrderPlant(p, 0.02)
    pid = PID(Gains(kp=3.0, ki=12.0, kd=0.0, u_min=-1.0, u_max=1.0), 0.02)

    # 6 s demanding a speed the plant cannot reach -> duty pinned at +1
    for _ in range(300):
        plant.step(pid.step(2.0, plant.v))
    assert plant.v > 0.55, "plant should be saturated at its top speed"
    assert pid.integ < 5.0, f"integrator wound up to {pid.integ}"

    # now ask for something reachable and record the approach
    target = 0.30
    traj = []
    for _ in range(400):
        plant.step(pid.step(target, plant.v))
        traj.append(plant.v)

    final = traj[-1]
    assert abs(final - target) < 0.02, f"did not settle: {final:.3f}"

    # how far does it dip below the target before settling?
    undershoot = (target - min(traj)) / target * 100
    assert undershoot < 15.0, (
        f"undershoot {undershoot:.1f}% -- the integrator is still charged, "
        f"anti-windup is not working")


def test_pid_output_always_within_limits():
    pid = PID(Gains(kp=50.0, ki=100.0, kd=5.0, u_min=-1.0, u_max=1.0), 0.02)
    for k in range(1000):
        u = pid.step(math.sin(k * 0.1), math.cos(k * 0.07))
        assert -1.0 <= u <= 1.0


def test_pid_preload_is_bumpless():
    """Mode changes call preload(). The first output afterwards must equal
    the duty we were already applying, or the car jerks on every switch."""
    pid = PID(Gains(kp=2.0, ki=5.0, kd=0.0), 0.02)
    pid.preload(setpoint=0.3, measurement=0.25, u_desired=0.42)
    u = pid.step(0.3, 0.25)
    assert abs(u - 0.42) < 0.02, f"bump of {u - 0.42:.4f}"


# =====================================================================
#  Plant
# =====================================================================
def test_plant_reaches_expected_steady_state():
    """With the dead-zone applied once (not twice -- this was a real bug
    in the sysid pipeline), v_ss must equal K*(u-dz)/(1-dz)."""
    p = PlantParams(K=0.55, tau=0.3, delay=0.0, deadzone=0.15, v_max=1.0)
    for duty in (0.3, 0.5, 0.7, 1.0):
        plant = FirstOrderPlant(p, 0.005)
        for _ in range(3000):
            plant.step(duty)
        eff = (duty - p.deadzone) / (1 - p.deadzone)
        assert abs(plant.v - p.K * eff) < 1e-3


def test_plant_deadzone_blocks_small_duty():
    p = PlantParams(deadzone=0.2)
    plant = FirstOrderPlant(p, 0.02)
    for _ in range(500):
        plant.step(0.15)
    assert plant.v < 1e-6, "wheel turned below the break-away duty"


def test_plant_time_constant_is_correct():
    """63.2% of final value at t = tau."""
    p = PlantParams(K=1.0, tau=0.25, delay=0.0, deadzone=0.0, v_max=10.0)
    dt = 0.001
    plant = FirstOrderPlant(p, dt)
    for _ in range(int(0.25 / dt)):
        plant.step(1.0)
    assert abs(plant.v - 0.632) < 0.01, f"v(tau) = {plant.v:.4f}"


def test_plant_delay_is_respected():
    p = PlantParams(K=1.0, tau=0.1, delay=0.1, deadzone=0.0, v_max=10.0)
    dt = 0.02
    plant = FirstOrderPlant(p, dt)
    for _ in range(4):           # 80 ms, still inside the 100 ms delay
        plant.step(1.0)
    assert plant.v < 1e-9, "responded before the transport delay elapsed"


def test_gap_kinematics_is_an_integrator():
    k = VehicleKinematics(gap=1.0, gap_max=10.0)
    for _ in range(100):
        k.step(v_lead=0.5, v_follow=0.3, dt=0.01)   # +0.2 m/s for 1 s
    assert abs(k.gap - 1.2) < 1e-6


# =====================================================================
#  Policies
# =====================================================================
def test_time_gap_scales_with_speed():
    """The defining property: it is a TIME gap, not a distance gap."""
    gp = GapPolicy(t_gap=1.5, d_standstill=0.20)
    assert abs(gp.desired_gap(0.00) - 0.20) < 1e-9
    assert abs(gp.desired_gap(0.40) - 0.80) < 1e-9   # cruise -> 80 cm
    assert abs(gp.desired_gap(0.60) - 1.10) < 1e-9


def test_degraded_link_opens_the_gap():
    """Losing V2V must make the follower MORE conservative, never less."""
    gp = GapPolicy(t_gap=1.5, t_gap_degraded=2.2)
    assert gp.desired_gap(0.40, degraded=True) > gp.desired_gap(0.40, degraded=False)


def test_gap_policy_equilibrium_is_the_desired_gap():
    """At steady state with a matched lead speed, v_tgt == v_lead exactly
    when the gap equals the desired gap."""
    gp = GapPolicy(t_gap=1.5, d_standstill=0.20, k_gap=0.9, v_max=0.60)
    v = 0.40                      # cruise, inside v_max
    d_des = gp.desired_gap(v)
    assert abs(gp.target_speed(v_lead=v, gap=d_des, v_follow=v) - v) < 1e-9

    # and the clamp must still bite above v_max, or a large gap error could
    # command a speed the vehicle cannot reach
    assert gp.target_speed(v_lead=0.60, gap=5.0, v_follow=0.40) == 0.60


def test_v2v_flag_brakes_before_ttc_would():
    """The core claim. With a large gap and no closing rate, TTC says
    everything is fine -- only the V2V flag can know the lead has braked."""
    bp = BrakePolicy()
    danger_no_v2v, _, ttc = bp.evaluate(gap=1.0, closing=0.0,
                                        lead_braking=False, lead_hazard=False)
    assert not danger_no_v2v and ttc > 10

    bp2 = BrakePolicy()
    danger_v2v, _, _ = bp2.evaluate(gap=1.0, closing=0.0,
                                    lead_braking=True, lead_hazard=False)
    assert danger_v2v, "V2V braking flag did not trigger the braking layer"


def test_ttc_triggers_without_v2v():
    """And the safety net must still work with the radio dead."""
    bp = BrakePolicy(ttc_brake=1.2)
    danger, _, ttc = bp.evaluate(gap=0.3, closing=0.5,
                                 lead_braking=False, lead_hazard=False)
    assert abs(ttc - 0.6) < 1e-9
    assert danger


def test_brake_hysteresis_prevents_chatter():
    bp = BrakePolicy(ttc_brake=1.2, release_hyst=1.35, d_floor=0.15)
    bp.evaluate(gap=0.3, closing=0.5, lead_braking=False, lead_hazard=False)
    # just barely clear of the threshold -- must stay latched
    danger, _, _ = bp.evaluate(gap=0.63, closing=0.5,
                               lead_braking=False, lead_hazard=False)
    assert danger, "released too eagerly; this is what causes brake chatter"


def test_rate_limit_respects_acceleration():
    assert abs(rate_limit(1.0, 0.0, 0.8, 0.02) - 0.016) < 1e-9
    assert abs(rate_limit(-1.0, 0.0, 0.8, 0.02) + 0.016) < 1e-9
    assert rate_limit(0.01, 0.0, 10.0, 0.02) == 0.01     # small step passes


# =====================================================================
#  Supervisor
# =====================================================================
def test_blind_and_deaf_means_fault_and_full_brake():
    c = FollowerController()
    out = c.step(gap_raw=1.0, v_follow=0.3, v_lead_v2v=0.0,
                 lead_braking=False, lead_hazard=False,
                 link=LinkState.LOST, range_ok=False)
    assert out.mode is Mode.FAULT
    assert out.duty == -1.0, "FAULT must command a full brake"


def test_hazard_escalates_to_emergency_immediately():
    c = FollowerController()
    out = c.step(gap_raw=1.2, v_follow=0.4, v_lead_v2v=0.4,
                 lead_braking=True, lead_hazard=True, link=LinkState.NOMINAL)
    assert out.mode is Mode.EMERG
    assert out.duty == -1.0


def test_mode_dwell_blocks_de_escalation_not_escalation():
    """Escalation must never wait on a timer; calming down must."""
    c = FollowerController(mode_min_dwell_s=0.5)
    c.step(gap_raw=1.2, v_follow=0.4, v_lead_v2v=0.4,
           lead_braking=True, lead_hazard=True, link=LinkState.NOMINAL)
    assert c.mode is Mode.EMERG
    out = c.step(gap_raw=1.2, v_follow=0.4, v_lead_v2v=0.4,
                 lead_braking=False, lead_hazard=False, link=LinkState.NOMINAL)
    assert out.mode is Mode.EMERG, "de-escalated before the dwell elapsed"


# =====================================================================
#  Closed loop
# =====================================================================
def test_closed_loop_never_collides_in_the_nominal_scenario():
    res = ClosedLoopTwin(dt=0.02).run(45.0, initial_gap=1.0)
    assert not res.collided, f"contact! min gap = {res.min_gap:.3f} m"
    assert res.min_gap > 0.05


def test_closed_loop_survives_total_v2v_loss():
    """Graceful degradation, not failure. Answers the jamming paper."""
    res = ClosedLoopTwin(dt=0.02, v2v_enabled=False).run(45.0, initial_gap=1.0)
    assert not res.collided, f"contact without V2V! min gap = {res.min_gap:.3f} m"


def test_closed_loop_survives_heavy_packet_loss():
    for loss in (0.3, 0.6, 0.9):
        res = ClosedLoopTwin(dt=0.02, v2v_loss_rate=loss).run(45.0)
        assert not res.collided, f"contact at {loss:.0%} packet loss"


def test_v2v_reacts_sooner_and_harder_than_sensor_only():
    """The headline claim of the project, as an executable assertion."""
    gp_fair = GapPolicy()
    gp_fair.t_gap_degraded = gp_fair.t_gap      # equal headway, fair test

    with_v2v = ClosedLoopTwin(dt=0.02, gap_policy=GapPolicy(**vars(gp_fair)),
                              v2v_enabled=True).run(45.0)
    without = ClosedLoopTwin(dt=0.02, gap_policy=GapPolicy(**vars(gp_fair)),
                             v2v_enabled=False).run(45.0)

    mag_a = with_v2v.response_magnitude(after=23.0, window=0.5)
    mag_b = without.response_magnitude(after=23.0, window=0.5)
    assert mag_a is not None and mag_b is not None
    assert mag_a > 1.5 * mag_b, (
        f"V2V should brake much harder in the first 0.5 s: "
        f"{mag_a:.3f} vs {mag_b:.3f} m/s")
    assert with_v2v.min_gap > without.min_gap, "V2V should keep more clearance"


def test_v2v_latency_is_bounded_by_the_beacon_interval():
    """A 20 Hz beacon cannot give better than 50 ms worst-case latency.
    If this ever reports ~0 ms, the channel model has stopped being
    discrete and the reported latencies are fiction."""
    res = ClosedLoopTwin(dt=0.02, v2v_tx_hz=20.0).run(45.0)
    lat = res.reaction_latency(after=23.0)
    assert lat is not None
    assert 0.0 < lat <= 0.12, f"latency {lat*1000:.0f} ms is not plausible"


# =====================================================================
#  Twin discipline
# =====================================================================
def test_twin_metrics_are_sane():
    a = [0.1, 0.2, 0.3, 0.4]
    assert compare(a, a).nrmse_fit_pct > 99.9
    assert compare(a, a).rmse < 1e-12
    worse = compare(a, [0.2, 0.3, 0.4, 0.5])
    assert worse.rmse > 0.09 and worse.nrmse_fit_pct < 50


def test_lead_scenario_matches_the_firmware_shape():
    cruise = 0.40                                 # cruise speed
    assert lead_scenario(1.0)[0] == 0.0           # stopped at the start
    assert lead_scenario(8.0)[0] == cruise        # cruising
    assert lead_scenario(14.0)[0] < cruise        # gentle slowdown
    assert lead_scenario(14.0)[0] > 0.0           # ...but still moving
    assert lead_scenario(26.0) == (0.0, True)     # emergency stop + hazard
    assert lead_scenario(43.0)[0] == lead_scenario(1.0)[0]   # wraps at 42 s


# =====================================================================
#  State space / LQR
# =====================================================================
def _ss():
    return ACCStateSpace(PlantParams(), t_gap=1.5, d_standstill=0.20)


def test_statespace_is_controllable():
    c = _ss().controllability()
    assert c["controllable"], f"rank {c['rank']}/{c['n']}"


def test_statespace_full_state_is_observable():
    assert _ss().observability()["observable"]


def test_gap_only_measurement_is_NOT_observable():
    """The instructive failure: measuring only the gap loses relative
    velocity, so a gap-only ACC would need an observer. We do not, because
    V2V hands us the lead speed directly."""
    o = _ss().observability(_ss().C_gap_only())
    assert not o["observable"] and o["rank"] == 2


def test_lead_speed_as_a_state_is_NOT_controllable():
    """No amount of follower throttle changes what the lead does, so v_lead
    must be a disturbance, not a state. Modelling it as a state silently
    produces a meaningless LQR design."""
    c4 = _ss().controllability_with_lead_as_state()
    assert not c4["controllable"] and c4["rank"] == 3


def test_care_residual_is_essentially_zero():
    """The Riccati solution must actually satisfy the equation. If this
    drifts, every gain and every pole downstream is wrong."""
    import numpy as np
    ss = _ss()
    r = ss.design()
    A, B, Q, R = ss.A, ss.B, r.Q, r.R
    res = A.T @ r.P + r.P @ A - r.P @ B @ np.linalg.inv(R) @ B.T @ r.P + Q
    assert np.max(np.abs(res)) < 1e-8
    assert np.all(np.linalg.eigvals(r.P) > 0), "P must be positive definite"


def test_lqr_closed_loop_is_stable():
    import numpy as np
    r = _ss().design()
    assert np.all(np.asarray(r.poles).real < 0), f"unstable: {r.poles}"


def test_lqr_satisfies_the_kalman_inequality():
    """|1 + L(jw)| >= 1 is what gives LQR its >=60 deg phase margin and
    infinite gain margin. If it fails, the model or design is broken."""
    import numpy as np
    ss = _ss()
    r = ss.design()
    K = np.asarray(r.K)
    worst = min(abs(1.0 + complex((K @ np.linalg.solve(1j * w * np.eye(3) - ss.A,
                                                       ss.B))[0, 0]))
                for w in np.logspace(-3, 3, 800))
    assert worst >= 0.99, f"min |1+L| = {worst:.4f}"


def test_lqr_drives_spacing_error_to_zero():
    """Constant lead speed must give exactly zero steady-state spacing error
    and a follower speed matching the lead. This is what the integral state
    is there for -- and it caught a real bug where the disturbance matrix
    was discretised as E*dt instead of the proper ZOH form."""
    ss = _ss()
    r = ss.design()
    res = ss.simulate(r.K, lambda t: 0.40, duration=40.0, dt=0.02)
    assert abs(res["e"][-1]) < 1e-3, f"e_ss = {res['e'][-1]}"
    assert abs(res["v_r"][-1]) < 1e-3
    assert abs(res["v_f"][-1] - 0.40) < 1e-3
    assert abs(res["u"][-1] - 0.40 / 0.60) < 1e-3   # u_ss must be v_l / K


def test_discretisation_is_consistent_at_small_dt():
    """Ad -> I + A*dt as dt -> 0. A wrong discretisation is invisible in the
    gains but ruins every simulation built on them."""
    import numpy as np
    ss = _ss()
    dt = 1e-4
    Ad, Bd, Ed = ss.discretise(dt, with_disturbance=True)
    assert np.allclose(Ad, np.eye(3) + ss.A * dt, atol=1e-6)
    assert np.allclose(Bd, ss.B * dt, atol=1e-6)
    assert np.allclose(Ed, ss.E * dt, atol=1e-6)


def test_expm_matches_a_known_result():
    import numpy as np
    # expm(diag(a,b)) = diag(e^a, e^b)
    M = np.diag([0.5, -1.25])
    assert np.allclose(expm(M), np.diag([np.exp(0.5), np.exp(-1.25)]), atol=1e-10)


def test_tighter_weights_give_larger_gains():
    """Bryson's rule must behave monotonically, or the weights are not
    expressing requirements the way we claim they do."""
    import numpy as np
    ss = _ss()
    loose = np.abs(np.asarray(ss.design(*ss.bryson_weights(e_max=0.20)).K).ravel()[0])
    tight = np.abs(np.asarray(ss.design(*ss.bryson_weights(e_max=0.02)).K).ravel()[0])
    assert tight > loose


# =====================================================================
#  Telemetry
# =====================================================================
def test_parse_follower_line():
    line = ("F,1234,0.850,0.800,0.050,3.20,0.400,0.405,0.400,0.62,"
            "2,0,0,0.1,0.2,0.0,500,3,1")
    rec = parse_line(line)
    assert isinstance(rec, FollowerRecord)
    assert rec.t_ms == 1234 and abs(rec.d - 0.850) < 1e-9
    assert rec.mode_name == "FOLLOW" and rec.link_name == "OK"
    assert abs(rec.gap_error - 0.05) < 1e-9
    assert abs(rec.loss_pct - 100 * 3 / 503) < 1e-6


def test_parse_lead_line():
    #  t_ms  v    v_tgt accel duty brk haz scn run thr tx seq  ok fail supp
    rec = parse_line("L,500,0.15,0.15,0.0,0.65,0,0,2,1,0.42,1,100,99,1,0")
    assert isinstance(rec, LeadRecord)
    assert rec.seq == 100 and abs(rec.tx_fail_pct - 1.0) < 1e-6
    assert rec.scn == 2 and rec.scn_name == "PREDICT"
    assert rec.running == 1 and rec.tx_on == 1
    assert abs(rec.throttle - 0.42) < 1e-9


def test_lead_line_without_optional_trailing_columns_still_parses():
    """A golden run recorded before a column was added must still replay.
    Recorded runs outlive firmware revisions."""
    rec = parse_line("L,500,0.15,0.15,0.0,0.65,0,0,2,1,0.42,1,100,99,1")
    assert isinstance(rec, LeadRecord)
    assert rec.tx_suppressed == 0


def test_follower_line_without_scenario_column_still_parses():
    line = ("F,1234,0.270,0.270,0.0,99.0,0.150,0.150,0.150,0.62,"
            "2,0,0,0.1,0.2,0.0,500,3,1")
    rec = parse_line(line)
    assert isinstance(rec, FollowerRecord)
    assert rec.scn == 0


def test_parser_survives_garbage():
    """A board resetting mid-print must not take the dashboard down."""
    for junk in ("", "   ", "F,1,2", "F,abc,def", "X,1,2,3", "\x00\xff",
                 "F,1234,0.85,0.80,"):
        parse_line(junk)        # must not raise


# =====================================================================
if __name__ == "__main__":
    import traceback
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    passed, failed = 0, []
    for name, fn in fns:
        try:
            fn()
            passed += 1
            print(f"  PASS  {name}")
        except Exception as exc:
            failed.append((name, exc))
            print(f"  FAIL  {name}: {exc}")
    print(f"\n{passed}/{len(fns)} passed")
    if failed:
        print()
        for name, exc in failed:
            print(f"--- {name} ---")
            traceback.print_exception(type(exc), exc, exc.__traceback__)
        sys.exit(1)
