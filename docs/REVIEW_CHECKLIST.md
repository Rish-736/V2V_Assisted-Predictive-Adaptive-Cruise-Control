# Review checklist

What to have ready at each graded review, and the questions to be ready for.
Individual contribution is graded separately, so §4 matters as much as the rest.

---

## Review 1 — survey + partial output

**You can complete every technical item with the two ESP32s you already have.**

- [ ] Literature survey, 5 papers (all verified on IEEE Xplore — see
      `PROJECT_HANDOVER.md` §8) + the identified gap
- [ ] Problem formulation
- [ ] Processor rationale: ESP32 — built-in radio means no extra comms hardware,
      enough timers for PWM + encoder, dual core, and ESP-NOW gives a
      connectionless broadcast that mirrors real V2V semantics
- [ ] **V2V link working, demonstrated.** Both boards flashed, both serial
      monitors streaming, follower reaching `PREDICT` and `EMERG` during the
      lead's scenario, `pkt_lost ≈ 0`
- [ ] Block diagram finalised
- [ ] Team roles and task split agreed

**Capture the proof — do not rely on live hardware on the day:**
```bash
python python/scripts/log_serial.py --port COM5 --out data/review1_link.csv --duration 60
```
```bash
python python/scripts/dashboard.py --replay data/review1_link.csv
```

Screenshot both serial monitors side by side. Record a short video of the
follower's mode switching when the lead brakes.

> **If asked "is that real or simulated?"** — answer directly: the radio link,
> the control loop, the state machine and the telemetry are real; the motor
> physics is a model, because the motors have not arrived. Then show
> `vehicle_io.h` and point out that the controller calls the same five functions
> either way, so nothing gets rewritten when the hardware lands. That is a
> stronger answer than pretending otherwise, and it is a design decision you can
> defend.

---

## Review 2 — progress

- [ ] Step-response capture on the real motor (`firmware/tools/step_response`)
- [ ] Transfer function fitted (`sysid_fit.py` and/or `matlab/sysid_fit.m`),
      **with the per-level spread reported**, not hidden behind one number
- [ ] Dead-zone identified and the corrected gain explained (see
      `CONTROL_DESIGN.md` §1.2 — this is a trap worth showing you avoided)
- [ ] PID speed loop running on hardware
- [ ] Time-gap outer loop implemented
- [ ] Root locus + Bode + margins (`design_control.py --plot`, or the MATLAB
      scripts for the report figures)
- [ ] Lead-lag designed — **or a justified statement that it is not needed**
- [ ] V2V + ultrasonic fusion working
- [ ] Bench test of the speed loop alone

**The figure to lead with:** linear prediction vs nonlinear simulation vs measured
hardware, all three on one set of axes. It shows you understand where the model
stops being true.

---

## Review 3 — full working model

- [ ] Predictive closing-rate braking fully integrated
- [ ] Digital twin + live dashboard running
- [ ] Two-stage twin validation done and written up:
      1. twin vs closed-form 2nd-order formulas
      2. twin vs **fresh** hardware runs (not the fitting data)
- [ ] Link-failure handling demonstrated **live** — walk a hand between the cars,
      or power the lead down mid-run, and show the follower drop to `DEGRADED`
      and open the time gap rather than panic
- [ ] Packet-loss tolerance characterised (`sim_only.py --sweep-loss`)
- [ ] The headline comparison, on hardware if possible, on the twin otherwise:
      with V2V vs sensor-only, equal headway
- [ ] State-space model + stability check (stretch)
- [ ] Full integration test, final report

**The demo that lands:** cars running, dashboard projected, then kill the V2V link
mid-run. The audience watches the mode change, the time gap widen, and the car
keep going safely. It shows the system degrades by design rather than failing.

---

## 2. Figures worth generating

```bash
python python/scripts/sim_only.py --compare --fair --save-plot data/fig_compare.png
```
```bash
python python/scripts/sysid_fit.py data/step.csv --save-plot data/fig_sysid.png
```
```bash
python python/scripts/design_control.py --params data/plant_params.json --save-plot data/fig_design.png
```
```bash
python python/scripts/sim_only.py --outage 22 30 --save-plot data/fig_degradation.png
```

---

## 3. Questions to be ready for

### On the novelty

**"Isn't this just ACC, which already exists?"**
Production ACC is sensor-only and reactive. This adds a cooperative feedforward
channel so the follower reacts to the lead's *decision* to brake rather than to
the *consequence* of it. That is CACC, it is an active research area, and the
combination here — predictive V2V braking + low-cost embedded two-node hardware +
live digital-twin validation — is not covered together by any of the five
surveyed papers.

**"What exactly is new, versus the papers you cited?"**
Each paper has one or two of the three pieces, in simulation or on full-scale
testbeds. Rezaee has the constant-time-gap policy but no hardware. Shen has
predictive connected cruise control in simulation. Dong has a digital twin but at
cloud/testbed scale. Nobody has all three on a buildable low-cost platform.

### On the control theory

**"Why a time gap and not a fixed distance?"**
Spacing must scale with speed to be safe, and a constant-distance policy is
string-unstable — disturbances amplify down a platoon. The `T_h·v` term is the
standard route to string stability. (`CONTROL_DESIGN.md` §3.1)

**"Why cascade instead of one loop?"**
Different bandwidths and different physics. The speed loop fights motor
dynamics (τ ≈ 0.25 s); the gap loop integrates a velocity difference. Separating
them lets each be designed against its own plant, and the inner loop's
saturation and anti-windup are handled where they occur. Valid only while the
bandwidths are ≥3–5× apart, which `design_control.py` checks explicitly.

**"How did you choose the gains?"**
λ (IMC) tuning, not trial and error. `Kp = τ/(Kλ)`, `Ki = Kp/τ` cancels the plant
pole and leaves a first-order closed loop with time constant λ. λ is bounded
below by the transport delay and the sample rate. One physically meaningful knob.

**"What is your phase margin, and does the delay matter?"**
Quote the number from `design_control.py`. The delay matters a great deal — it is
what makes the gain margin finite. Without it the phase never reaches −180° and
the gain margin is infinite, which is why the margin calculation unwraps the
phase properly rather than using the principal value.

**"Why is there no integral term in the outer loop?"**
The gap plant is a pure integrator already (`d(gap)/dt = v_lead − v_follow`), so
proportional control gives zero steady-state gap error. Adding integral action
would add a second pole at the origin and destabilise it.

**"Your PID predicts zero overshoot but the hardware overshoots."**
Correct, and expected. The prediction is from the linear model; the real plant
has a dead-zone, saturates at ±1, and is sampled. Here is the dead-zone we
measured and here is the nonlinear simulation that reproduces the overshoot.
*(This is a strong answer. Do not hide the discrepancy.)*

### On robustness

**"What if the wireless link is jammed or lost?"**
The system degrades, it does not fail. Losing V2V removes the feedforward term;
the TTC and gap-floor layers still work on the ultrasonic alone. The controller
detects the loss by packet timeout and widens the time gap from 1.5 s to 2.2 s to
buy back the lost reaction time. Demonstrated up to 95% packet loss with no
contact.

**"What if someone forges a braking packet?"**
Out of scope and worth naming honestly. ESP-NOW supports encryption with a
pre-shared key, which would be the first mitigation; the deeper problem —
authenticating a stranger's broadcast — is what PKI in real V2V standards exists
to solve.

**"What if the ultrasonic sensor fails?"**
With V2V still up, the controller continues on the V2V speed. With both gone it
enters `FAULT` and commands a full brake — fail-safe, not fail-operational.

### On the digital twin

**"How do you know the twin is right?"**
Two-stage validation, specifically to avoid circularity. Stage 1 compares the
twin against closed-form 2nd-order formulas — that validates the simulation
*code*. Stage 2 compares it against hardware runs *different from the ones used
to fit the model* — that validates the *model*. Validating against the fitting
data alone would be circular.

**"Does the twin control the car?"**
No, and deliberately not. Real-time braking uses live sensor and V2V data with
conservative margins, precisely because the model is known to be imperfect. The
twin validates and lets us tune offline.

**"Why not run it on the ESP32?"**
It would compete with a 50 Hz real-time loop for CPU, an independent check
sharing a processor with the thing it checks is not independent, and it needs
numpy/matplotlib.

### On scope

**"Why doesn't it ever crash in your comparison?"**
Because at scale-model speeds the car stops in ~5 cm but follows at ~60 cm — it is
about an order of magnitude more over-provisioned than a real vehicle, since
braking authority does not scale down the way kinetic energy does. So we report
**reaction latency and clearance margin**, which do transfer, rather than
collision counts, which do not.

---

## 4. Individual contribution

Graded separately, so keep the record as you go.

Rishit is leaning toward owning the **control logic and software**, with
hardware/wiring split to teammates. Concretely ownable:

- the control architecture (cascade, supervisor, degradation strategy)
- `firmware/common/` — the control laws themselves
- the system-identification and control-design pipeline
- the digital twin and the validation methodology
- the test suite

Worth being able to point at specific decisions, not just files. Good examples
from this repo:

- choosing ESP-NOW **broadcast** over unicast, for both practical and
  V2V-semantic reasons
- the `SIM_PLANT` abstraction, which unblocked all software development while the
  BOM was on order
- catching that the **apparent gain and the dead-zone-corrected gain are not the
  same number**, and that using the former would have made the twin wrong by ~20%
- catching that the **degraded-mode time gap confounds the V2V comparison**, and
  adding `--fair` to separate the two effects
- fixing the **phase unwrapping** so the gain margin is finite and correct
- adding **mode dwell** after seeing the supervisor chatter in simulation, and
  recognising that chatter also resets the PID integrator through bumpless
  transfer

Each of those is a defensible engineering judgement with a before/after you can
show.
