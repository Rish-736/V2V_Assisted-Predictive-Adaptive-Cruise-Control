"""
controller.py -- Python mirror of the firmware control laws.

This is a deliberate, maintained duplicate of:
    firmware/common/pid.h
    firmware/common/filters.h
    the outer loop + braking logic in firmware/follow_node/follow_node.ino

WHY DUPLICATE IT INSTEAD OF JUST SIMULATING THE PLANT
-----------------------------------------------------
A digital twin that only models the motor tells you whether the MOTOR
behaves as expected. It cannot tell you whether the CONTROLLER is doing
what you think. By running the same control laws here, we can:

  * tune gains offline against the identified plant, at 1000x real time,
    without touching hardware or risking a crash;
  * replay a recorded hardware run through the twin and see exactly
    where real and predicted diverge, and whether the divergence is in
    the plant or in the controller;
  * answer the obvious viva question "how do you know the twin is right?"
    -- because stage 1 of validation compares this code against the
    closed-form 2nd-order formulas, not against the hardware.

PARITY RULE: if you change a control law in the firmware, change it here
in the same commit. tests/test_parity.py checks the numeric agreement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

__all__ = [
    "Mode", "LinkState", "Gains", "GapPolicy", "BrakePolicy",
    "PID", "LowPass", "Median3", "Differentiator", "LeadLag",
    "FollowerController", "ControllerOutput", "rate_limit", "clamp",
]


# =====================================================================
#  Enums -- numeric values MUST match follow_node.ino
# =====================================================================
class Mode(IntEnum):
    STANDBY = 0
    CRUISE = 1
    FOLLOW = 2
    PREDICT = 3
    EMERG = 4
    FAULT = 5


class LinkState(IntEnum):
    NOMINAL = 0
    DEGRADED = 1
    LOST = 2


# =====================================================================
#  Helpers
# =====================================================================
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def rate_limit(target: float, current: float, max_rate: float, dt: float) -> float:
    d = max_rate * dt
    delta = target - current
    if delta > d:
        return current + d
    if delta < -d:
        return current - d
    return target


# =====================================================================
#  Filters -- mirrors of filters.h
# =====================================================================
@dataclass
class LowPass:
    fc_hz: float
    dt: float
    y: float = 0.0
    primed: bool = False
    alpha: float = field(init=False)

    def __post_init__(self) -> None:
        rc = 1.0 / (2.0 * math.pi * self.fc_hz)
        self.alpha = self.dt / (self.dt + rc)

    def step(self, x: float) -> float:
        if not self.primed:
            self.y, self.primed = x, True
            return self.y
        self.y += self.alpha * (x - self.y)
        return self.y

    def reset(self) -> None:
        self.y, self.primed = 0.0, False


@dataclass
class Median3:
    w: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    n: int = 0

    def step(self, x: float) -> float:
        self.w[2], self.w[1], self.w[0] = self.w[1], self.w[0], x
        if self.n < 3:
            self.n += 1
            return x
        a, b, c = self.w
        return sorted((a, b, c))[1]


@dataclass
class Differentiator:
    fc_hz: float
    dt: float
    prev: float = 0.0
    primed: bool = False
    lp: LowPass = field(init=False)

    def __post_init__(self) -> None:
        self.lp = LowPass(self.fc_hz, self.dt)

    def step(self, x: float) -> float:
        if not self.primed:
            self.prev, self.primed = x, True
            return 0.0
        raw = (x - self.prev) / self.dt
        self.prev = x
        return self.lp.step(raw)

    def reset(self) -> None:
        self.primed = False
        self.lp.reset()


@dataclass
class LeadLag:
    """Discrete first-order section: y[k] = b0 x[k] + b1 x[k-1] - a1 y[k-1]."""

    b0: float = 1.0
    b1: float = 0.0
    a1: float = 0.0
    x1: float = 0.0
    y1: float = 0.0

    def step(self, x: float) -> float:
        y = self.b0 * x + self.b1 * self.x1 - self.a1 * self.y1
        self.x1, self.y1 = x, y
        return y

    def reset(self) -> None:
        self.x1 = self.y1 = 0.0


# =====================================================================
#  PID -- mirror of pid.h
#  derivative-on-measurement, filtered, back-calculation anti-windup
# =====================================================================
@dataclass
class Gains:
    kp: float = 1.80
    ki: float = 6.00
    kd: float = 0.04
    n_deriv: float = 12.0
    u_min: float = -1.0
    u_max: float = 1.0


@dataclass
class PID:
    g: Gains = field(default_factory=Gains)
    dt: float = 0.02
    integ: float = 0.0
    d_state: float = 0.0
    y_prev: float = 0.0
    primed: bool = False
    # diagnostics
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    u_unsat: float = 0.0

    @property
    def kaw(self) -> float:
        return (self.g.ki / self.g.kp) if self.g.kp > 1e-6 else 1.0

    def reset(self) -> None:
        self.integ = self.d_state = 0.0
        self.primed = False
        self.p_term = self.i_term = self.d_term = self.u_unsat = 0.0

    def preload(self, setpoint: float, measurement: float, u_desired: float) -> None:
        """Bumpless transfer: seed the integrator so output starts at u_desired."""
        self.integ = u_desired - self.g.kp * (setpoint - measurement)
        self.d_state = 0.0
        self.y_prev = measurement
        self.primed = True

    def step(self, setpoint: float, measurement: float) -> float:
        g = self.g
        e = setpoint - measurement
        if not self.primed:
            self.y_prev, self.primed = measurement, True

        self.p_term = g.kp * e

        dy = (measurement - self.y_prev) / self.dt
        self.y_prev = measurement
        nd = g.n_deriv * self.dt
        self.d_state = (self.d_state + nd * (-dy)) / (1.0 + nd)
        self.d_term = g.kd * self.d_state

        self.integ += g.ki * e * self.dt
        self.i_term = self.integ

        self.u_unsat = self.p_term + self.i_term + self.d_term
        u = clamp(self.u_unsat, g.u_min, g.u_max)
        if u != self.u_unsat:
            self.integ += self.kaw * (u - self.u_unsat) * self.dt
            self.i_term = self.integ
        return u


# =====================================================================
#  Policies
# =====================================================================
@dataclass
class GapPolicy:
    """Constant time-gap spacing policy.

        d_des = d0 + Th * v_follow
        v_tgt = v_lead + K_gap * (d - d_des)

    The Th*v term is what makes this a TIME gap rather than a distance
    gap: the faster you go, the further back you sit. It is also the
    term that buys string stability -- with a pure constant-distance
    policy a disturbance grows as it propagates down a platoon.
    """

    # must match firmware/common/config.h section 3
    t_gap: float = 1.50
    t_gap_degraded: float = 2.20
    d_standstill: float = 0.20
    k_gap: float = 0.90
    v_max: float = 0.60

    def desired_gap(self, v_follow: float, degraded: bool = False) -> float:
        th = self.t_gap_degraded if degraded else self.t_gap
        return self.d_standstill + th * v_follow

    def target_speed(self, v_lead: float, gap: float, v_follow: float,
                     degraded: bool = False) -> float:
        d_des = self.desired_gap(v_follow, degraded)
        return clamp(v_lead + self.k_gap * (gap - d_des), 0.0, self.v_max)


@dataclass
class BrakePolicy:
    """Three OR-ed danger signals. See follow_node.ino for the rationale."""

    ttc_brake: float = 1.20
    ttc_emergency: float = 0.60
    d_floor: float = 0.15
    closing_min: float = 0.02
    release_hyst: float = 1.35
    _latched: bool = field(default=False, repr=False)

    def ttc(self, gap: float, closing: float) -> float:
        return gap / closing if closing > self.closing_min else 99.0

    def evaluate(self, gap: float, closing: float, lead_braking: bool,
                 lead_hazard: bool, range_ok: bool = True) -> tuple[bool, bool, float]:
        """Returns (danger, emergency, ttc)."""
        t = self.ttc(gap, closing)

        danger_v2v = lead_braking or lead_hazard
        danger_ttc = t < self.ttc_brake
        danger_floor = range_ok and gap < self.d_floor

        emergency = (lead_hazard
                     or t < self.ttc_emergency
                     or (range_ok and gap < self.d_floor * 0.6))

        danger = danger_v2v or danger_ttc or danger_floor
        if self._latched and not danger:
            cleared = (t > self.ttc_brake * self.release_hyst
                       and gap > self.d_floor * self.release_hyst)
            if not cleared:
                danger = True
        self._latched = danger
        return danger, emergency, t

    def reset(self) -> None:
        self._latched = False


# =====================================================================
#  The full follower controller
# =====================================================================
@dataclass
class ControllerOutput:
    duty: float
    v_target: float
    v_target_raw: float
    d_des: float
    closing: float
    ttc: float
    v_lead_est: float
    mode: Mode
    link: LinkState
    braking: bool
    p_term: float
    i_term: float
    d_term: float


@dataclass
class FollowerController:
    """Port of the follow_node control loop. Same structure, same order of
    operations, same numbers -- so the twin's predictions are predictions
    of OUR controller, not of some idealised one."""

    dt: float = 0.02
    gains: Gains = field(default_factory=Gains)
    gap_policy: GapPolicy = field(default_factory=GapPolicy)
    brake_policy: BrakePolicy = field(default_factory=BrakePolicy)

    v_cruise: float = 0.35
    v_standstill: float = 0.03
    a_max: float = 0.80
    a_brake: float = 1.60
    range_max: float = 1.20
    use_leadlag: bool = False
    leadlag: LeadLag = field(default_factory=LeadLag)

    # filter corners (match config.h)
    range_lpf_hz: float = 4.0
    closing_lpf_hz: float = 2.0

    # state
    pid: PID = field(init=False)
    _med: Median3 = field(init=False)
    _lp: LowPass = field(init=False)
    _diff: Differentiator = field(init=False)
    mode: Mode = Mode.STANDBY
    v_target: float = 0.0
    _last_duty: float = 0.0
    mode_min_dwell_s: float = 0.25
    _mode_age: float = 0.0

    def __post_init__(self) -> None:
        self.pid = PID(self.gains, self.dt)
        self._med = Median3()
        self._lp = LowPass(self.range_lpf_hz, self.dt)
        self._diff = Differentiator(self.closing_lpf_hz, self.dt)

    def reset(self) -> None:
        self.pid.reset()
        self._med = Median3()
        self._lp = LowPass(self.range_lpf_hz, self.dt)
        self._diff = Differentiator(self.closing_lpf_hz, self.dt)
        self.brake_policy.reset()
        self.leadlag.reset()
        self.mode = Mode.STANDBY
        self.v_target = 0.0
        self._last_duty = 0.0
        self._mode_age = 0.0

    def step(self, *, gap_raw: float, v_follow: float, v_lead_v2v: float,
             lead_braking: bool, lead_hazard: bool,
             link: LinkState, range_ok: bool = True,
             filter_range: bool = True) -> ControllerOutput:
        """One control tick. Mirrors loop() in follow_node.ino.

        `filter_range=False` when the caller already filtered (real HC-SR04
        path does the median+LPF inside the I/O layer).
        """
        dt = self.dt
        link_usable = link != LinkState.LOST
        degraded = link != LinkState.NOMINAL

        # --- 2. sense
        d = self._lp.step(self._med.step(gap_raw)) if filter_range else gap_raw
        closing = -self._diff.step(d)

        v_lead_est = v_lead_v2v if link_usable else max(0.0, v_follow - closing)

        # --- 3. predictive braking decision
        danger, emergency, ttc = self.brake_policy.evaluate(
            d, closing, lead_braking and link_usable,
            lead_hazard and link_usable, range_ok)

        # --- 4. supervisor
        prev_mode = self.mode
        if not range_ok and not link_usable:
            self.mode = Mode.FAULT
        elif emergency:
            self.mode = Mode.EMERG
        elif danger:
            self.mode = Mode.PREDICT
        elif (not range_ok) or d >= self.range_max * 0.95:
            self.mode = Mode.CRUISE
        elif (v_follow < self.v_standstill and v_lead_est < self.v_standstill
              and d <= self.gap_policy.d_standstill * 1.2):
            self.mode = Mode.STANDBY
        else:
            self.mode = Mode.FOLLOW

        # Mode dwell -- see MODE_MIN_DWELL_MS in config.h. Escalating to a
        # more dangerous mode is immediate; calming down waits out the dwell,
        # so the supervisor cannot dither and repeatedly reload the PID.
        self._mode_age += dt
        if self.mode != prev_mode:
            escalating = int(self.mode) > int(prev_mode)
            if not escalating and self._mode_age < self.mode_min_dwell_s:
                self.mode = prev_mode
            else:
                self._mode_age = 0.0

        # --- 5. outer loop
        d_des = self.gap_policy.desired_gap(v_follow, degraded)
        if self.mode in (Mode.FAULT, Mode.EMERG, Mode.STANDBY):
            v_raw = 0.0
        elif self.mode is Mode.PREDICT:
            v_raw = min(0.0, v_lead_est + self.gap_policy.k_gap * (d - d_des))
        elif self.mode is Mode.CRUISE:
            v_raw = self.v_cruise
        else:
            v_raw = v_lead_est + self.gap_policy.k_gap * (d - d_des)
        v_raw = clamp(v_raw, 0.0, self.gap_policy.v_max)

        a_limit = self.a_brake if self.mode in (Mode.EMERG, Mode.FAULT) else self.a_max
        self.v_target = rate_limit(v_raw, self.v_target, a_limit, dt)

        # --- 6. bumpless transfer
        if self.mode != prev_mode:
            self.pid.preload(self.v_target, v_follow, self._last_duty)
            self.leadlag.reset()

        # --- 7. inner loop
        u = self.pid.step(self.v_target, v_follow)
        if self.use_leadlag:
            u = clamp(self.leadlag.step(u), self.gains.u_min, self.gains.u_max)

        if self.mode in (Mode.EMERG, Mode.FAULT):
            u = -1.0
        if v_follow <= self.v_standstill and self.v_target <= 0.0:
            u = 0.0

        self._last_duty = u

        return ControllerOutput(
            duty=u, v_target=self.v_target, v_target_raw=v_raw, d_des=d_des,
            closing=closing, ttc=ttc, v_lead_est=v_lead_est,
            mode=self.mode, link=link, braking=danger,
            p_term=self.pid.p_term, i_term=self.pid.i_term, d_term=self.pid.d_term)
