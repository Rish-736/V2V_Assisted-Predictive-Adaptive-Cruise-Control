"""
plant.py -- the motor / vehicle model used by the digital twin.

THE MODEL
---------
Speed response of a PWM-driven DC geared motor, first order with
transport delay:

                    K
    V(s)/U(s) = ---------- * exp(-L*s)
                 tau*s + 1

  K    [m/s per unit duty]  steady-state gain
  tau  [s]                  mechanical time constant
  L    [s]                  transport delay (PWM period, driver turn-on,
                            encoder windowing, serial latency)

plus a static-friction DEAD-ZONE on the input, which is not part of the
transfer function at all -- it is a genuine nonlinearity. We keep it in
the simulation and OUT of the linear model used for root locus / Bode,
and we say so in the report. Pretending a geared DC motor is linear down
to zero duty is the most common way these projects get their predicted
settling time wrong.

Why first order and not second
------------------------------
We are controlling SPEED, not position. The electrical time constant
(L_a/R_a, order 1 ms) is two decades faster than the mechanical one
(J/b, order 100 ms), so it is legitimate to neglect it -- the classic
reduction of the 2nd-order armature model to 1st order. If sysid_fit.py
reports that a second-order fit is materially better, use SecondOrderPlant
instead; the twin supports both.

This module intentionally duplicates the discrete update used in the
firmware (firmware/common/vehicle_io.h). They must agree step for step,
so if you change one, change the other. test_parity.py checks this.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

__all__ = ["PlantParams", "FirstOrderPlant", "SecondOrderPlant", "VehicleKinematics"]


# ---------------------------------------------------------------------
@dataclass
class PlantParams:
    """Identified motor parameters. Defaults mirror config.h section 8."""

    K: float = 0.60          # m/s per unit duty
    tau: float = 0.25        # s
    delay: float = 0.05      # s
    deadzone: float = 0.12   # duty below which nothing moves
    v_max: float = 0.60      # m/s saturation
    # second-order extras (only used by SecondOrderPlant)
    zeta: float = 0.9
    wn: float = 4.0          # rad/s

    @classmethod
    def from_json(cls, path) -> "PlantParams":
        """Load the output of scripts/sysid_fit.py."""
        import json
        from pathlib import Path

        data = json.loads(Path(path).read_text(encoding="utf-8"))
        # accept either a flat dict or the full fit report
        src = data.get("params", data)
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: float(v) for k, v in src.items() if k in known})

    def tf(self):
        """Return (num, den) of the delay-free linear part, for python-control."""
        return [self.K], [self.tau, 1.0]

    def summary(self) -> str:
        return (f"K={self.K:.4f} m/s/duty  tau={self.tau:.4f} s  "
                f"L={self.delay:.4f} s  deadzone={self.deadzone:.3f}")


# ---------------------------------------------------------------------
def _apply_deadzone(u: float, dz: float) -> float:
    """Static friction: below break-away duty nothing turns. Above it, the
    effective duty is rescaled so full duty still maps to full speed."""
    if dz <= 0.0:
        return u
    if abs(u) < dz:
        return 0.0
    if u > 0.0:
        return (u - dz) / (1.0 - dz)
    return (u + dz) / (1.0 - dz)


# ---------------------------------------------------------------------
@dataclass
class FirstOrderPlant:
    """Discrete first-order motor model with dead-zone and transport delay.

    Mirrors vio_update() in firmware/common/vehicle_io.h exactly:
        v[k+1] = v[k] + (dt/tau) * (K*u_eff[k-d] - v[k])
    """

    p: PlantParams = field(default_factory=PlantParams)
    dt: float = 0.02
    v: float = 0.0
    allow_reverse: bool = False
    _delay_line: deque = field(init=False, repr=False)

    def __post_init__(self) -> None:
        n = max(1, int(round(self.p.delay / self.dt)))
        self._delay_line = deque([0.0] * n, maxlen=n)

    def reset(self, v0: float = 0.0) -> None:
        self.v = v0
        self._delay_line = deque([0.0] * self._delay_line.maxlen,
                                maxlen=self._delay_line.maxlen)

    def step(self, u: float) -> float:
        u = max(-1.0, min(1.0, float(u)))
        self._delay_line.append(u)
        u_delayed = self._delay_line[0]

        u_eff = _apply_deadzone(u_delayed, self.p.deadzone)
        v_inf = self.p.K * u_eff
        self.v += (self.dt / self.p.tau) * (v_inf - self.v)

        if not self.allow_reverse and self.v < 0.0:
            self.v = 0.0
        if self.v > self.p.v_max:
            self.v = self.p.v_max
        return self.v


# ---------------------------------------------------------------------
@dataclass
class SecondOrderPlant:
    """Standard 2nd-order form, for when the step data shows overshoot.

        d2v/dt2 + 2*zeta*wn*dv/dt + wn^2 * v = K * wn^2 * u

    Kept so the report can quote the closed-form 2nd-order metrics
    (Mp, tr, ts) against simulation -- validation stage 1 in the
    project plan.
    """

    p: PlantParams = field(default_factory=PlantParams)
    dt: float = 0.02
    v: float = 0.0
    vdot: float = 0.0
    _delay_line: deque = field(init=False, repr=False)

    def __post_init__(self) -> None:
        n = max(1, int(round(self.p.delay / self.dt)))
        self._delay_line = deque([0.0] * n, maxlen=n)

    def reset(self, v0: float = 0.0) -> None:
        self.v, self.vdot = v0, 0.0
        self._delay_line = deque([0.0] * self._delay_line.maxlen,
                                maxlen=self._delay_line.maxlen)

    def step(self, u: float) -> float:
        u = max(-1.0, min(1.0, float(u)))
        self._delay_line.append(u)
        u_eff = _apply_deadzone(self._delay_line[0], self.p.deadzone)

        wn, z, K = self.p.wn, self.p.zeta, self.p.K
        vddot = K * wn * wn * u_eff - 2.0 * z * wn * self.vdot - wn * wn * self.v
        self.vdot += vddot * self.dt
        self.v += self.vdot * self.dt

        if self.v < 0.0:
            self.v, self.vdot = 0.0, max(0.0, self.vdot)
        if self.v > self.p.v_max:
            self.v, self.vdot = self.p.v_max, min(0.0, self.vdot)
        return self.v

    # ----- closed-form metrics, for validation stage 1 ----------------
    def theoretical_metrics(self) -> dict:
        z, wn = self.p.zeta, self.p.wn
        out = {"zeta": z, "wn": wn}
        if 0.0 < z < 1.0:
            wd = wn * math.sqrt(1.0 - z * z)
            out["overshoot_pct"] = 100.0 * math.exp(-math.pi * z / math.sqrt(1 - z * z))
            out["t_peak_s"] = math.pi / wd
            out["t_rise_s"] = (math.pi - math.acos(z)) / wd
        else:
            out["overshoot_pct"] = 0.0
            out["t_peak_s"] = float("inf")
            out["t_rise_s"] = 2.2 * self.p.tau
        out["t_settle_2pct_s"] = 4.0 / (z * wn) if z * wn > 0 else float("inf")
        return out


# ---------------------------------------------------------------------
@dataclass
class VehicleKinematics:
    """Integrates the inter-vehicle gap from the two speeds.

        d(gap)/dt = v_lead - v_follow

    This is the plant of the OUTER loop, and it is a pure integrator --
    which is exactly why the outer loop needs so little gain to be
    stable, and why a constant-time-gap policy (which adds the
    Th*v_follow term) is what makes a platoon string-stable rather than
    amplifying disturbances down the line.
    """

    gap: float = 1.0
    gap_min: float = 0.0
    gap_max: float = 2.5

    def step(self, v_lead: float, v_follow: float, dt: float) -> float:
        self.gap += (v_lead - v_follow) * dt
        self.gap = max(self.gap_min, min(self.gap_max, self.gap))
        return self.gap

    @property
    def contact(self) -> bool:
        return self.gap <= self.gap_min
