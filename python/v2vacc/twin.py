"""
twin.py -- the digital twin.

THE ONE RULE
------------
The twin is fed the same COMMANDS as the hardware. It is never fed the
hardware's own measured output. If you ever let the real speed leak into
the twin's state, the twin stops being a prediction and becomes an echo
-- it will track beautifully and tell you nothing. Every function here
takes a command (duty, or a speed setpoint) and nothing else from the
real vehicle.

WHAT RUNS WHERE, AND WHY
------------------------
The twin runs on the laptop, not the ESP32:
  * it must not compete with a 50 Hz real-time control loop for CPU;
  * an independent check that shares a processor with the thing it is
    checking is not independent;
  * it needs numpy/matplotlib/scipy, which the ESP32 does not have.

The twin makes NO real-time braking decisions. Braking uses live sensor
and V2V data with conservative margins, precisely because the model is
known to be imperfect. The twin's job is validation and offline tuning.

TWO-STAGE VALIDATION (avoids circularity)
-----------------------------------------
  Stage 1  twin vs. closed-form 2nd-order formulas
           -> proves the SIMULATION CODE is implemented correctly.
  Stage 2  twin vs. FRESH hardware runs, different from the runs used to
           fit the model
           -> proves the MODEL captures reality (friction, delay, ...).
Stage 2 against the fitting data alone would be circular: of course the
model matches the data it was fitted to.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .controller import (FollowerController, Gains, GapPolicy, BrakePolicy,
                         LinkState, Mode)
from .plant import FirstOrderPlant, PlantParams, VehicleKinematics

__all__ = ["OpenLoopTwin", "ClosedLoopTwin", "FitMetrics", "compare",
           "lead_scenario", "SimResult"]


# =====================================================================
#  Stage-2 helper: how well do two traces agree?
# =====================================================================
@dataclass
class FitMetrics:
    rmse: float
    mae: float
    max_err: float
    nrmse_fit_pct: float   # the MATLAB-style "% fit" everyone quotes
    r2: float
    n: int

    def __str__(self) -> str:
        return (f"n={self.n}  RMSE={self.rmse:.4f}  MAE={self.mae:.4f}  "
                f"max|e|={self.max_err:.4f}  fit={self.nrmse_fit_pct:.1f}%  "
                f"R2={self.r2:.4f}")


def compare(actual: list[float], predicted: list[float]) -> FitMetrics:
    """Agreement metrics between a real trace and the twin's prediction.

    nrmse_fit_pct is the goodness-of-fit MATLAB's System Identification
    Toolbox reports: 100*(1 - ||y-yhat|| / ||y-mean(y)||). It is the one
    to quote in the report because it is scale-free and the examiner will
    recognise it. Above ~85% on a fresh run is a genuinely good model for
    a hobby gearmotor.
    """
    n = min(len(actual), len(predicted))
    if n == 0:
        return FitMetrics(0, 0, 0, 0, 0, 0)
    a = list(actual[:n])
    p = list(predicted[:n])

    errs = [ai - pi for ai, pi in zip(a, p)]
    sse = sum(e * e for e in errs)
    rmse = math.sqrt(sse / n)
    mae = sum(abs(e) for e in errs) / n
    max_err = max(abs(e) for e in errs)

    mean_a = sum(a) / n
    sst = sum((ai - mean_a) ** 2 for ai in a)

    if sst > 1e-12:
        nrmse = 100.0 * (1.0 - math.sqrt(sse) / math.sqrt(sst))
        r2 = 1.0 - sse / sst
    else:
        nrmse, r2 = (100.0, 1.0) if sse < 1e-12 else (0.0, 0.0)

    return FitMetrics(rmse, mae, max_err, nrmse, r2, n)


# =====================================================================
#  Open-loop twin -- the live one that runs beside the hardware
# =====================================================================
class OpenLoopTwin:
    """Fed the DUTY the hardware commanded; predicts the speed it should
    have produced. Any divergence is model error (friction, battery sag,
    wheel slip), which is exactly what we want to see and quantify.

    Usage in the dashboard: for every FollowerRecord that arrives,
        v_pred = twin.step(record.duty)
    and plot v_pred against record.v on the same time axis.
    """

    def __init__(self, params: PlantParams | None = None, dt: float = 0.04):
        self.params = params or PlantParams()
        self.dt = dt
        self.plant = FirstOrderPlant(self.params, dt)
        self.t = 0.0
        self.history_t: list[float] = []
        self.history_v: list[float] = []

    def reset(self) -> None:
        self.plant.reset()
        self.t = 0.0
        self.history_t.clear()
        self.history_v.clear()

    def step(self, duty: float, dt: float | None = None) -> float:
        """Advance by one telemetry sample. Pass the real dt between
        samples if the stream is not perfectly periodic -- serial jitter
        is real and ignoring it slowly shifts the twin in time."""
        if dt is not None and abs(dt - self.plant.dt) > 1e-6 and 0.0 < dt < 1.0:
            self.plant.dt = dt
            self.dt = dt
        v = self.plant.step(duty)
        self.t += self.plant.dt
        self.history_t.append(self.t)
        self.history_v.append(v)
        return v


# =====================================================================
#  Lead scenario -- mirror of scenarioSpeed() in lead_node.ino
# =====================================================================
def lead_scenario(t: float, *, cruise: float = 0.40, t_start: float = 3.0,
                  t_brake1: float = 12.0, t_resume: float = 17.0,
                  t_brake_hard: float = 24.0, t_restart: float = 32.0,
                  t_loop: float = 42.0) -> tuple[float, bool]:
    """Returns (commanded_speed, hazard_flag). Must match the firmware."""
    if t_loop > 0:
        t = math.fmod(t, t_loop)
    if t < t_start:
        return 0.0, False
    if t < t_brake1:
        return cruise, False
    if t < t_resume:
        return cruise * 0.45, False
    if t < t_brake_hard:
        return cruise, False
    if t < t_restart:
        return 0.0, True
    return cruise, False


# =====================================================================
#  Closed-loop twin -- the whole two-vehicle system, offline
# =====================================================================
@dataclass
class SimResult:
    """Column-wise traces. Plain lists so this works without numpy."""
    t: list[float] = field(default_factory=list)
    gap: list[float] = field(default_factory=list)
    gap_des: list[float] = field(default_factory=list)
    v_lead: list[float] = field(default_factory=list)
    v_follow: list[float] = field(default_factory=list)
    v_target: list[float] = field(default_factory=list)
    duty: list[float] = field(default_factory=list)
    closing: list[float] = field(default_factory=list)
    ttc: list[float] = field(default_factory=list)
    mode: list[int] = field(default_factory=list)
    link: list[int] = field(default_factory=list)
    braking: list[int] = field(default_factory=list)
    lead_braking: list[int] = field(default_factory=list)

    @property
    def min_gap(self) -> float:
        return min(self.gap) if self.gap else 0.0

    @property
    def collided(self) -> bool:
        return self.min_gap <= 0.001

    def brake_onset(self, after: float = 0.0) -> float | None:
        """Time the FOLLOWER first commands braking, after time `after`."""
        for t, b in zip(self.t, self.braking):
            if t >= after and b:
                return t
        return None

    def lead_brake_onset(self, after: float = 0.0) -> float | None:
        """Time the LEAD first begins braking, after time `after`."""
        for t, b in zip(self.t, self.lead_braking):
            if t >= after and b:
                return t
        return None

    def reaction_latency(self, after: float = 0.0) -> float | None:
        """Seconds between the lead braking and the follower reacting.

        THE headline metric for this project. With V2V this is bounded by
        the broadcast interval (~50 ms at 20 Hz). Without it, the follower
        must wait for the closing rate to build up through a median filter
        and a 2 Hz low-pass before it is distinguishable from sensor noise
        -- which costs hundreds of milliseconds. At full-scale speeds that
        difference is metres of stopping distance.
        """
        lead = self.lead_brake_onset(after)
        if lead is None:
            return None
        follow = self.brake_onset(lead)
        return None if follow is None else follow - lead

    def response_latency(self, after: float = 0.0,
                         threshold: float = 0.01) -> float | None:
        """Seconds from the lead braking until the follower's commanded
        speed starts falling by more than `threshold` m/s.

        This is the fairer companion to reaction_latency(). A sensor-only
        controller often never trips the discrete BRAKE layer at all -- it
        just eases off through the speed loop. Saying it "never reacts"
        would be wrong. What it actually does is react LATE and GENTLY,
        and this metric captures that, while reaction_latency() captures
        how late the hard-braking layer engages.
        """
        lead = self.lead_brake_onset(after)
        if lead is None:
            return None
        base = None
        for t, vt in zip(self.t, self.v_target):
            if t < lead:
                base = vt
                continue
            if base is not None and vt < base - threshold:
                return t - lead
        return None

    def response_magnitude(self, after: float = 0.0,
                           window: float = 0.5) -> float | None:
        """How much speed the follower has actually shed `window` seconds
        after the lead began braking, in m/s.

        Onset timing alone is a weak metric: a sensor-only controller can
        TWITCH almost immediately (the closing rate changes sign the moment
        the lead decelerates) while responding far too weakly to matter,
        because that signal is buried under a median filter and a 2 Hz
        low-pass. This measures the strength of the response, which is what
        actually determines the clearance.
        """
        lead = self.lead_brake_onset(after)
        if lead is None:
            return None
        v_at, v_after = None, None
        for t, v in zip(self.t, self.v_follow):
            if v_at is None and t >= lead:
                v_at = v
            if v_at is not None and t >= lead + window:
                v_after = v
                break
        if v_at is None or v_after is None:
            return None
        return v_at - v_after

    def headline(self) -> str:
        return (f"min gap {self.min_gap*100:.1f} cm   "
                f"{'COLLISION' if self.collided else 'no contact'}   "
                f"{len(self.t)} steps")

    def to_csv(self, path) -> None:
        import csv
        from pathlib import Path
        cols = ["t", "gap", "gap_des", "v_lead", "v_follow", "v_target",
                "duty", "closing", "ttc", "mode", "link", "braking",
                "lead_braking"]
        with Path(path).open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(cols)
            for row in zip(*(getattr(self, c) for c in cols)):
                w.writerow([f"{x:.6g}" if isinstance(x, float) else x for x in row])


class ClosedLoopTwin:
    """Full two-vehicle simulation: lead plant + follower plant +
    follower controller + gap kinematics + a model of the V2V link.

    This is the offline sandbox used for gain tuning, for exploring the
    scenarios that are dangerous or tedious to stage on the floor, and
    for producing the "with V2V vs without V2V" comparison that is the
    central claim of the project.
    """

    def __init__(self, *, dt: float = 0.02,
                 params: PlantParams | None = None,
                 gains: Gains | None = None,
                 gap_policy: GapPolicy | None = None,
                 brake_policy: BrakePolicy | None = None,
                 v2v_enabled: bool = True,
                 v2v_tx_hz: float = 20.0,
                 v2v_latency_s: float = 0.0,
                 v2v_loss_rate: float = 0.0,
                 range_noise_m: float = 0.010,
                 seed: int = 12345):
        self.dt = dt
        self.params = params or PlantParams()
        self.v2v_enabled = v2v_enabled
        self.v2v_tx_hz = v2v_tx_hz
        self.v2v_latency_s = v2v_latency_s
        self.v2v_loss_rate = v2v_loss_rate
        self.range_noise_m = range_noise_m

        self.lead_plant = FirstOrderPlant(self.params, dt)
        self.follow_plant = FirstOrderPlant(self.params, dt)
        self.kin = VehicleKinematics(gap=1.00)

        self.ctrl = FollowerController(
            dt=dt,
            gains=gains or Gains(),
            gap_policy=gap_policy or GapPolicy(),
            brake_policy=brake_policy or BrakePolicy(),
        )
        # simple P controller for the lead, tracking its scripted profile
        self.lead_pid = FollowerController(dt=dt).pid
        self._rng_state = seed
        self._v2v_queue: list[tuple[float, float, bool, bool]] = []

    # ----- tiny deterministic RNG so runs are reproducible ------------
    def _rand(self) -> float:
        self._rng_state = (1103515245 * self._rng_state + 12345) & 0x7FFFFFFF
        return self._rng_state / 0x7FFFFFFF

    def _noise(self, amp: float) -> float:
        return (self._rand() - 0.5) * 2.0 * amp

    # -----------------------------------------------------------------
    def run(self, duration: float = 45.0, *, initial_gap: float = 1.00,
            v2v_outage: tuple[float, float] | None = None) -> SimResult:
        """Simulate for `duration` seconds.

        v2v_outage: (t_start, t_end) during which the link is forced
        down -- used to demonstrate graceful degradation and to answer
        the jamming-attack paper from the literature survey.
        """
        dt = self.dt
        res = SimResult()

        self.lead_plant.reset()
        self.follow_plant.reset()
        self.ctrl.reset()
        self.lead_pid.reset()
        self.kin = VehicleKinematics(gap=initial_gap)
        self._v2v_queue.clear()

        v_lead_cmd = 0.0
        latency_steps = max(0, int(round(self.v2v_latency_s / dt)))
        last_rx: tuple[float, bool, bool] = (0.0, False, False)
        steps_since_rx = 0
        link_timeout_steps = int(round(0.300 / dt))

        n = int(duration / dt)
        for k in range(n):
            t = k * dt

            # ---- lead vehicle follows its scripted profile ------------
            cmd, hazard = lead_scenario(t)
            a_lim = 1.60 if hazard else 0.80
            from .controller import rate_limit as _rl
            v_lead_cmd = _rl(cmd, v_lead_cmd, a_lim, dt)
            u_lead = self.lead_pid.step(v_lead_cmd, self.lead_plant.v)
            v_lead = self.lead_plant.step(u_lead)

            lead_accel = (v_lead - (res.v_lead[-1] if res.v_lead else 0.0)) / dt
            lead_braking = lead_accel < -0.15 or cmd < v_lead - 0.05 or hazard

            # ---- V2V channel model ------------------------------------
            outage = (v2v_outage is not None
                      and v2v_outage[0] <= t < v2v_outage[1])
            # The lead broadcasts at v2v_tx_hz, NOT every control tick.
            # This is what makes the measured reaction latency honest: a
            # 20 Hz beacon means the follower learns about a brake event
            # 0-50 ms after it happens, never instantly.
            tx_period = 1.0 / self.v2v_tx_hz if self.v2v_tx_hz > 0 else dt
            tx_tick = (k * dt) % tx_period < dt - 1e-9
            tx_ok = (self.v2v_enabled and not outage and tx_tick
                     and self._rand() >= self.v2v_loss_rate)

            if tx_ok:
                self._v2v_queue.append((t, v_lead, lead_braking, hazard))
            # deliver anything old enough
            delivered = False
            while self._v2v_queue and (t - self._v2v_queue[0][0]) >= self.v2v_latency_s:
                _, s, b, h = self._v2v_queue.pop(0)
                last_rx = (s, b, h)
                delivered = True
            steps_since_rx = 0 if delivered else steps_since_rx + 1

            if not self.v2v_enabled:
                link = LinkState.LOST
            elif steps_since_rx > link_timeout_steps * 3:
                link = LinkState.LOST
            elif steps_since_rx > link_timeout_steps:
                link = LinkState.DEGRADED
            else:
                link = LinkState.NOMINAL

            v_lead_rx, brk_rx, haz_rx = last_rx

            # ---- follower: sense, control, actuate --------------------
            gap_raw = self.kin.gap + self._noise(self.range_noise_m)
            out = self.ctrl.step(
                gap_raw=gap_raw,
                v_follow=self.follow_plant.v,
                v_lead_v2v=v_lead_rx,
                lead_braking=brk_rx,
                lead_hazard=haz_rx,
                link=link,
            )
            v_follow = self.follow_plant.step(out.duty)

            # ---- gap kinematics ---------------------------------------
            gap = self.kin.step(v_lead, v_follow, dt)

            # ---- record ------------------------------------------------
            res.t.append(t)
            res.gap.append(gap)
            res.gap_des.append(out.d_des)
            res.v_lead.append(v_lead)
            res.v_follow.append(v_follow)
            res.v_target.append(out.v_target)
            res.duty.append(out.duty)
            res.closing.append(out.closing)
            res.ttc.append(min(out.ttc, 10.0))
            res.mode.append(int(out.mode))
            res.link.append(int(link))
            res.braking.append(1 if out.braking else 0)
            res.lead_braking.append(1 if lead_braking else 0)

            if self.kin.contact:
                # keep simulating so the plot shows the full picture,
                # but the SimResult.collided flag records what happened
                pass

        return res
