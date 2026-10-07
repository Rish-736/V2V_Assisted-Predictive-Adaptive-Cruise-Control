# Control design

The theory behind the code, with the derivations. This is the document to read
before the viva.

---

## 1. Plant model

### 1.1 Why first order

We control **speed**, not position. The armature circuit gives a second-order
model, but the electrical time constant `La/Ra` is of order 1 ms while the
mechanical one `J/b` is of order 100 ms — two decades apart. Neglecting the fast
pole is the standard reduction, and it leaves

```
                 K
    G(s) = ------------- · e^(-L·s)
            τ·s + 1
```

| symbol | meaning | units |
|---|---|---|
| `K` | steady-state gain | m/s per unit duty |
| `τ` | mechanical time constant | s |
| `L` | transport delay (PWM period, driver turn-on, encoder windowing) | s |

If `sysid_fit.py` reports that a second-order fit is materially better, use
`SecondOrderPlant` — the twin supports both, and the closed-form 2nd-order
metrics are what validation stage 1 compares against.

### 1.2 The dead-zone is not in the transfer function

Static friction means duty below a break-away threshold produces **no motion at
all**. This is a genuine nonlinearity and it is deliberately kept *out* of
`G(s)` and *in* the simulation.

Pretending a geared DC motor is linear down to zero duty is the most common way
these projects end up with a predicted settling time that the hardware refuses to
match.

**The trap this project already fell into and fixed:** the per-level gain
`v_ss/duty` is the *apparent* gain — it already has the dead-zone loss baked in,
which is why it drifts with operating point. The simulator applies the dead-zone
*separately* and then multiplies by `K`. Feeding it the apparent gain subtracts
the friction twice and makes the model predict a speed ~20% low at mid duty.

The dead-zone-corrected gain is the **slope of the static curve**:

```
    v_ss = K·(u − dz)/(1 − dz)  ⟹  slope m = K/(1 − dz)  ⟹  K = m·(1 − dz)
```

and the dead-zone is that curve's x-intercept. `sysid_fit.py` computes both and
prints a warning about the distinction. Verified: the fitter recovers `K`, `τ`,
`L` and `dz` from synthetic data with known parameters, and the twin then
reproduces the true steady state to within 0.3% at every duty.

---

## 2. Inner loop — speed PID

### 2.1 λ (IMC) tuning

For `G = K/(τs+1)` under PI control, choosing

```
    Kp = τ / (K·λ)        Ki = Kp / τ
```

places the PI zero exactly on the plant pole. It cancels, and the nominal closed
loop collapses to a single first-order lag with time constant λ:

- rise time (10–90%) = 2.2 λ
- 2% settling time = 3.9 λ
- overshoot = **0%**

One physically meaningful knob — "how fast do you want it" — instead of three
gains found by trial and error. This is worth saying explicitly in the viva: it
is a *design method*, not a tuning session.

### 2.2 Why λ cannot be made arbitrarily small

Pole cancellation is exact only in the model. The real plant has a transport
delay that **nothing causal can cancel**, plus a discrete sample period. Pushing
λ down raises loop gain at frequencies where the delay has already eaten the
phase, and the loop rings or goes unstable.

`design_control.py` picks

```
    λ = max( 3L , 10·dt , 0.3·τ )
```

- `3L` — never outrun the transport delay
- `10·dt` — never outrun the sample rate
- `0.3·τ` — do not demand ~3× the plant's own bandwidth, which just saturates
  the duty on every setpoint step

### 2.3 Four things in `pid.h` that a textbook PID omits

**Derivative on measurement, not error.** The outer loop steps the speed
setpoint. Derivative-on-error turns every step into a spike into the motor
driver ("derivative kick"). Differentiating `−y` gives identical disturbance
rejection with no kick. *(Test: `test_pid_no_derivative_kick_on_setpoint_step`.)*

**Filtered derivative.** An ideal differentiator has infinite high-frequency
gain and would amplify encoder quantisation noise straight into the PWM. We use
`Kd·N·s/(s+N)` — a real pole at `N` rad/s.

**Back-calculation anti-windup.** Duty saturates at ±1. Without this, a long
brake charges the integrator and the car sails past the setpoint on release. We
bleed the integrator by the amount actually clipped, with `Kaw = Ki/Kp`. With
`Kp=1, Ki=10` the integrator reaches a fixed point at 1.0 instead of growing
without bound. *(Tests: `test_pid_antiwindup_bounds_the_integrator`,
`test_antiwindup_limits_closed_loop_undershoot`.)*

> A subtlety worth knowing before someone asks: feeding `e = 0` forever does
> **not** discharge the integrator — an integrator with zero input holds its
> value. Recovery happens only once the measurement crosses the setpoint and the
> error changes sign. That is why the meaningful anti-windup test is closed-loop.

**Bumpless transfer.** The supervisor hands control between modes.
`pid_preload()` seeds the integrator so `u` is continuous across a mode change
instead of jumping.

---

## 3. Outer loop — constant time-gap policy

### 3.1 The law

```
    d_des = d₀ + T_h · v_follow                    (desired spacing)
    v_tgt = v_lead + K_gap · (d − d_des)           (speed setpoint)
```

The `T_h·v` term is what makes this a **time** gap: the faster you go, the
further back you sit. A constant *distance* policy is both unsafe at speed and
string-unstable — a disturbance grows as it propagates down a platoon.

### 3.2 `v_lead` is the feedforward term, and it is the whole project

With V2V we **know** `v_lead` — it arrives in the packet. Without it, the best we
can do is reconstruct it:

```
    v_lead ≈ v_follow − closing_rate
```

which requires differentiating a noisy range signal through a median filter and a
2 Hz low-pass. That chain is what costs the reaction time. The follower
implements both paths and switches automatically on link health, so the
degradation is graceful rather than a failure.

### 3.3 The gap plant is a pure integrator

```
    d(gap)/dt = v_lead − v_follow
```

Consequences worth quoting:

- Outer crossover sits at `K_gap` itself.
- A pure integrator under proportional control has **infinite gain margin and 90°
  phase margin** on its own — which is why the outer loop needs no integral term
  and stays stable so easily.
- No integral term is needed for zero steady-state gap error either, because the
  plant already integrates.

### 3.4 Cascade separation — the assumption that can silently break

Designing the two loops independently is only valid if the inner loop looks like
unity gain to the outer one. That needs a bandwidth separation:

```
    inner ≈ 1/λ = 5.0 rad/s
    outer ≈ K_gap = 0.9 rad/s        →  5.6×  ✓
```

`design_control.py` checks this and complains below 3×. Under 3× the loops fight
each other and the "design them separately" assumption stops holding *without
producing any obvious symptom* — which is exactly why it is worth checking
explicitly rather than assuming.

---

## 4. Predictive braking — three OR-ed danger signals

| signal | trips on | what it is for |
|---|---|---|
| **V2V** | `braking` or `hazard` flag in the packet | fires the instant the lead brakes, *before the gap changes at all* |
| **TTC** | `gap / closing_rate < 1.2 s` | anticipatory; works with the radio dead |
| **Floor** | `gap < 0.15 m` | last-resort backstop |

Whichever trips first wins. (a) is the novel one; (b) and (c) are the safety net.

**This layering is the honest answer to the jamming-attack paper.** V2V improves
the margin; it is not load-bearing for basic safety. `sim_only.py --sweep-loss`
shows the minimum gap stays positive up to 95% packet loss.

### 4.1 Why the raw closing rate is unusable

HC-SR04 noise is roughly ±1–2 cm with occasional total outliers (a missed echo
reads as max range). Differentiating that at 20 Hz turns a **single 2 cm glitch
into 0.4 m/s of apparent closing speed** — enough to slam the brakes on for a
stationary target.

Chain: `median-of-3 → low-pass (4 Hz) → differentiate → low-pass (2 Hz)`.

The median kills impulsive outliers (a low-pass alone just smears them across
several samples); the low-passes handle the Gaussian part. Median-of-3 costs only
2 samples of group delay, which matters when the entire point is early reaction.

### 4.2 Hysteresis

Release requires clearing the threshold by 1.35×, otherwise the controller
chatters in and out of braking right at the boundary.

---

## 5. Supervisor

```
STANDBY → CRUISE → FOLLOW → PREDICT → EMERG → FAULT
                                              (blind AND deaf)
```

Link health is tracked separately: `NOMINAL / DEGRADED / LOST`.

### 5.1 Losing V2V is a degradation, not a fault

Losing the link removes the *feedforward* term. The system is still stable on
ultrasonic feedback alone — it just demotes from predictive to reactive. So the
correct response is not to fault but to **buy back the lost reaction time by
opening up the time gap**: `T_h` goes from 1.50 s to 2.20 s.

This has a direct consequence for how you benchmark, and it caught this project
out during development: with the fallback active, the two runs are *not following
at the same distance*, so comparing their clearance is meaningless. Use
`sim_only.py --fair` to hold the headway equal and isolate the feedforward
effect.

Read the other way, the unfair comparison is itself the more interesting result:
**V2V buys a tighter gap at equal safety** — which is exactly the CACC claim in
the literature (≈0.6 s CACC headway vs ≈1.5 s for ACC), and the mechanism by
which cooperative cruise control increases road capacity.

### 5.2 Mode dwell

Without it the supervisor chatters between adjacent modes several times a second
when a signal sits on a boundary. That is not merely ugly on a plot: **every mode
change triggers a bumpless-transfer reload of the PID**, so a dithering
supervisor repeatedly resets the integrator and the speed loop never settles.

`MODE_MIN_DWELL_MS = 250` fixes it with one counter. **Escalation to a more
dangerous mode always bypasses the dwell** — you never delay a brake; only
calming down waits. Measured effect: sensor-only mode transitions over a 45 s run
went from chattering to **0**, while the V2V run keeps its 6 meaningful ones.

---

## 6. What the linear analysis cannot tell you

Every number in §2 comes from the **linear** model. The real plant has a
dead-zone, saturates at ±1, and is sampled. So:

- Measured overshoot will be non-zero even though pole-cancelled PI predicts 0%.
- Settling will be slower near zero speed, where the dead-zone dominates.
- The `K` spread across operating points (reported by `sysid_fit.py`) **bounds
  how well any single linear controller can do**.

`design_control.py --plot` deliberately plots the linear prediction against a
nonlinear simulation on the same axes, so the discrepancy is visible rather than
hidden. Quote it; do not apologise for it. "We predicted 0% overshoot, measured
12%, and here is the dead-zone that accounts for it" is a much stronger answer
than a model that happens to match.

---

## 7. Known gaps

- **String stability** for 3+ vehicles is not analysed. The constant time-gap
  policy is the standard route to it, so this is the natural extension and the
  obvious thing to be asked about.
- **State-space model + formal stability proof** — Review 3 stretch goal.
- **Sensor fusion is OR-logic, not a Kalman filter.** Deliberate: the OR is
  provable, debuggable and cannot be destabilised by a bad covariance estimate.
  A weighted/Kalman fusion of the V2V speed and the differentiated range is the
  stated upgrade path, and would give a better `v_lead` estimate in the degraded
  mode specifically.
- **The delay is treated as a lumped transport delay.** It is really a mix of
  PWM period, driver turn-on and encoder windowing, each with different
  behaviour.
