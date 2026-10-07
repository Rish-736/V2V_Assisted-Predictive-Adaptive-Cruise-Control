# Onboarding prompt

Copy everything below the line into a fresh AI session (or hand it to a person)
to bring them up to speed on this repository from zero.

---

You are picking up an in-progress university control-systems project. Read this
whole brief before touching anything. Repository:
`https://github.com/Rish-736/V2V_Assisted-Predictive-Adaptive-Cruise-Control`

## 1. What the project is

**V2V-Assisted Predictive Adaptive Cruise Control** — BECE302L (Control Systems),
VIT Vellore, Fall 2026–27, faculty Dr Gopinath. Graded across three reviews;
individual contribution is graded separately.

Two ESP32 "vehicles" on a small scale testbed:

- The **lead** node drives its own speed to a scripted profile and broadcasts
  `{speed, acceleration, braking flag, hazard flag, sequence number}` over
  ESP-NOW at 20 Hz.
- The **follower** node receives that broadcast, combines it with its own
  ultrasonic range reading, holds a constant **time gap** (not a fixed
  distance), and brakes **predictively**.
- A **digital twin** runs on a laptop in Python, plotting its independent
  prediction beside the real hardware.

### The one-sentence thesis

A conventional ACC is purely reactive: it cannot know the car ahead is braking
until the gap has already started shrinking, because it must wait for the
integral of a speed difference to rise above sensor noise. That wait is the
collision. The V2V broadcast removes the wait entirely, converting a **feedback**
problem into a **feedforward** one.

### Novelty claim (vs. the 5 surveyed IEEE papers)

No existing work combines all three of: predictive V2V-based braking + low-cost
embedded two-node hardware + live digital-twin validation. Each paper has one or
two, in simulation or on expensive full-scale testbeds. The five papers are
listed in `PROJECT_HANDOVER.md` §8 (all verified on IEEE Xplore).

## 2. The single most important thing to understand: `SIM_PLANT`

**The owner currently has only the two ESP32 boards. No motors, no drivers, no
ultrasonic sensor.**

The firmware therefore ships with `SIM_PLANT 1` in `firmware/common/config.h`.
This flag is read by **exactly one file**, `firmware/common/vehicle_io.h`, which
is a hardware abstraction layer exposing five functions:

```c
vio_begin()        vio_set_duty(u)    vio_speed()
vio_range()        vio_update(dt)
```

| | `SIM_PLANT 1` (now) | `SIM_PLANT 0` (later) |
|---|---|---|
| speed | discrete first-order motor model + noise | quadrature encoder |
| gap | integrate `(v_lead − v_follow)` | HC-SR04 + median + low-pass |
| actuation | state update | L298N PWM @ 20 kHz |
| **V2V radio** | **real** | real |
| **control laws** | **real** | real |
| **state machine** | **real** | real |
| **telemetry** | **real** | real |

Everything above the abstraction — PID, time-gap law, predictive braking,
supervisor — calls the same five functions either way. **The controller cannot
tell the difference.** So a controller validated in simulation is the identical
binary logic that will run on the car; flipping the flag changes no control code.

This is why the whole system is demonstrable on two bare boards today, and it is
a defensible design decision, not a workaround. Do not treat it as "fake" — be
upfront that the physics is modelled while the link, loops, state machine and
telemetry are real.

## 3. What has actually been built

Roughly 1,700 lines of firmware, 3,800 of Python, 1,500 of documentation.
**36/36 tests pass.**

### `firmware/`

- **`common/`** — the single source of truth. Because the Arduino IDE copies a
  sketch folder to a temp directory before compiling (which breaks `../`
  includes), each sketch folder holds a *generated* copy. **Edit `common/` only,
  then run `python tools/sync_common.py`.** `--check` fails if a copy is stale.
  - `config.h` — every tunable parameter in one place, in 12 numbered sections
  - `v2v_protocol.h` — 26-byte packed struct, CRC16-CCITT, sequence numbers,
    magic + version, and a shim so one source compiles on ESP32 Arduino core
    2.x *and* 3.x (the ESP-NOW receive callback signature changed)
  - `filters.h` — median-of-3, first-order low-pass, filtered differentiator,
    rate limiter, lead-lag biquad
  - `pid.h` — derivative-on-measurement (no derivative kick), filtered
    derivative, back-calculation anti-windup, bumpless-transfer preload
  - `vehicle_io.h` — the abstraction layer described above
- **`lead_node/`** — scripted repeatable speed profile, own-acceleration
  estimate, braking/hazard decision, 20 Hz ESP-NOW broadcast, serial telemetry
- **`follow_node/`** — the actual control system (see §4)
- **`tools/mac_address/`** — prints MAC + channel + chip info
- **`tools/step_response/`** — system-identification data capture: steps at
  0.3/0.5/0.7/1.0 duty, both rising and falling edges, 200 Hz sampling

The V2V link uses **ESP-NOW broadcast** (`FF:FF:FF:FF:FF:FF`), not unicast MAC
pairing. Two reasons: nothing to configure or keep in sync, and real V2V
(DSRC/C-V2X) Basic Safety Messages genuinely *are* broadcast. Both nodes must
share `V2V_WIFI_CHANNEL` (currently 1).

### `python/v2vacc/`

- `plant.py` — first- and second-order motor models with dead-zone and transport
  delay, plus gap kinematics
- `controller.py` — a **hand-maintained mirror** of the firmware control laws
- `twin.py` — `OpenLoopTwin` (live, runs beside hardware) and `ClosedLoopTwin`
  (offline, full two-vehicle simulation with a V2V channel model), plus
  agreement metrics
- `telemetry.py` — parser for the CSV telemetry format, deliberately forgiving
  of truncated lines
- `serial_link.py` — threaded serial reader with a bounded queue, plus log replay

### `python/scripts/`

| script | what it does |
|---|---|
| `sim_only.py` | offline closed-loop simulation; the with-V2V vs sensor-only study; packet-loss sweep |
| `sysid_fit.py` | fits the motor transfer function from step data |
| `design_control.py` | PID + lead-lag design, stability margins, root locus, Bode |
| `dashboard.py` | live dashboard plotting the twin beside the hardware |
| `log_serial.py` | captures serial output to a file |

### `matlab/`

`sysid_fit.m` and `design_pid_leadlag.m` — companions that use the campus
Control System Toolbox licence and produce the report figures (root locus, Bode,
Nyquist, margins). The Python path is the one that feeds the twin; MATLAB is for
the write-up.

### `docs/`

`ARCHITECTURE.md`, `CONTROL_DESIGN.md` (the theory with derivations),
`WIRING.md` (pinout + the four ways to destroy a board), `REVIEW_CHECKLIST.md`
(per-review deliverables and ~20 anticipated viva questions with answers).

## 4. The control architecture

```
SUPERVISOR: STANDBY / CRUISE / FOLLOW / PREDICT / EMERG / FAULT
            link health: NOMINAL / DEGRADED / LOST
                    ↓
OUTER LOOP (time gap):   d_des = d₀ + T_h·v_follow
                         v_tgt = v_lead + K_gap·(d − d_des)
                    ↓
INNER LOOP (speed):      PID + optional lead-lag → duty ∈ [−1, +1]
                    ↓
PLANT:                   L298N + DC motor  (or the simulated model)
```

**`v_lead` is the feedforward term and it is the entire point.** With V2V we
*know* it — it arrives in the packet. Without it, the best reconstruction is
`v_lead ≈ v_follow − closing_rate`, which requires differentiating a noisy range
signal through a median filter and a 2 Hz low-pass. That filter chain is what
costs the reaction time.

### Predictive braking: three OR-ed danger signals

| signal | trips on | purpose |
|---|---|---|
| **V2V** | braking/hazard flag in the packet | fires the instant the lead brakes, *before the gap changes at all* |
| **TTC** | `gap / closing_rate < 1.2 s` | anticipatory; works with the radio dead |
| **Floor** | `gap < 0.15 m` | last-resort backstop |

Whichever trips first wins. V2V is the novel one; TTC and the floor are the
safety net. This layering is deliberate and is the honest answer to the
jamming-attack paper in the survey: **V2V improves the margin, it is not
load-bearing for basic safety.**

### Graceful degradation

Losing the link removes the *feedforward* term; the system stays stable on
ultrasonic feedback alone, just demoted from predictive to reactive. The correct
response is not to fault but to **buy back the lost reaction time by widening the
time gap** — 1.5 s → 2.2 s.

## 5. What exactly is analysed and simulated

### Analysed (control theory)

- **Plant identification.** First-order `G(s) = K/(τs+1)·e^(−Ls)`, fitted per
  step amplitude so the nonlinearity is visible rather than averaged away. The
  dead-zone is deliberately kept *out* of `G(s)` and *in* the simulation.
- **Inner-loop design by λ (IMC) tuning.** `Kp = τ/(K·λ)`, `Ki = Kp/τ` cancels
  the plant pole, leaving a first-order closed loop with time constant λ: 0%
  predicted overshoot, one physically meaningful knob. λ is bounded below by
  `max(3L, 10·dt, 0.3τ)` because nothing causal can cancel a transport delay.
- **Stability margins** — gain margin, phase margin, both crossover frequencies,
  computed from the delay-inclusive loop gain with properly unwrapped phase.
- **Cascade separation check.** The outer loop's plant is a *pure integrator*
  (`d(gap)/dt = v_lead − v_follow`), so outer crossover sits at `K_gap` itself.
  Designing the loops independently is only valid while the bandwidths are ≥3–5×
  apart; the script checks and complains.
- **Lead compensator design** to a target phase margin, emitted as discrete
  Tustin biquad coefficients ready to paste into `config.h`.

### Simulated

- **The motor**, including dead-zone, transport delay and saturation.
- **Gap kinematics** between the two vehicles.
- **The V2V channel** — discrete 20 Hz beacon (so latency is a realistic
  0–50 ms, not a fake 0 ms), configurable packet-loss rate, configurable
  latency, and forced outage windows.
- **Sensor noise** on both range and speed, so the filters are actually
  exercised.
- **The full closed loop**, both vehicles, 45 s scripted scenario: stopped →
  cruise → gentle slowdown → cruise → **emergency stop** → recover → loop.

## 6. Results already obtained (all reproducible with no hardware)

Equal headway in both runs (`sim_only.py --compare --fair`):

| | with V2V | sensor only |
|---|---|---|
| speed shed in the first 0.5 s of the emergency stop | **39.3 cm/s** | 12.6 cm/s (**3.1× weaker**) |
| minimum clearance | **45.3 cm** | 23.5 cm |
| braking layer engaged? | yes, at 60 ms | **never** — eased off through the speed loop only |

Other verified results:

- **Packet-loss sweep:** minimum gap stays positive from 0% to 95% loss.
  Contact never occurs even with the link fully dead. This demonstrates the
  layering claim.
- **sysID pipeline verified end to end against synthetic data with known
  parameters:** the fitter recovers τ to 0.2%, the delay to within one sample,
  and the dead-zone exactly; the twin then reproduces the true steady-state speed
  to within **0.3%** at every operating point.
- **Stability margins** for the nominal plant: PM 72.9° at 5.0 rad/s, GM 14.4 dB
  at 26.3 rad/s — the gain margin independently cross-checked analytically.
- **Mode chatter eliminated:** sensor-only mode transitions over a 45 s run went
  from chattering to 0, while the V2V run keeps its 6 meaningful ones.

Reproduce everything:

```bash
pip install -r python/requirements.txt
python python/scripts/sim_only.py --compare --fair --plot
python python/scripts/sim_only.py --sweep-loss
python python/scripts/dashboard.py --demo
python python/tests/test_control.py
```

## 7. Bugs already found and fixed — do not reintroduce these

Each produced a *plausible-looking but wrong* result. Re-check them whenever a
new number appears.

1. **Apparent gain vs dead-zone-corrected gain.** `v_ss/duty` already includes
   the static-friction loss, so it drifts with operating point. The simulator
   applies the dead-zone *separately*, so feeding it the apparent `K` subtracts
   friction twice (~20% low at mid duty). The correct `K` is the **slope of the
   static curve × (1 − dz)**; the dead-zone is that curve's x-intercept.
2. **The degraded time gap confounded the V2V comparison.** With V2V off the
   controller correctly falls back to a wider headway, so the two runs were not
   following at the same distance. `--fair` holds it equal. Read the other way,
   the "unfair" comparison is the *more interesting* result: V2V buys a **tighter
   gap at equal safety**, which is exactly the CACC claim in the literature
   (~0.6 s CACC headway vs ~1.5 s ACC) and the mechanism by which cooperative
   cruise control increases road capacity.
3. **Phase wrapping killed the gain margin.** `cmath.phase` returns (−π, π], so
   −190° reads as +170° and the −180° crossing is never found → infinite gain
   margin reported, when the transport delay is *precisely* what makes it finite.
   Must unwrap.
4. **Collisions are impossible at scale-model speeds.** The car stops in ~5 cm
   but follows at ~60 cm — about an order of magnitude more over-provisioned than
   a real vehicle, because braking authority does not scale down the way kinetic
   energy does. **Report reaction latency and clearance margin, never collision
   counts.**
5. **Latency metrics need a discrete beacon.** Delivering V2V packets every
   control tick gives a fake 0 ms latency.
6. **Onset timing alone is a weak metric.** Sensor-only *twitches* almost
   immediately but responds far too weakly. Measure response *magnitude*.
7. **`braking = (speed < 3.0)` is backwards** (it was in the original draft).
   Braking is a *deceleration* statement; that test flags a slow-but-steady
   vehicle and misses a fast one that has just braked.
8. **Mode chatter resets the PID.** Every mode change triggers a
   bumpless-transfer reload, so a dithering supervisor never lets the speed loop
   settle. Fixed with a 250 ms dwell that **escalation bypasses** — you never
   delay a brake; only calming down waits.
9. **A PI integrator with `e == 0` never discharges.** Recovery needs the error
   to change sign, so anti-windup must be tested closed-loop, not open-loop.

## 8. Hard rules

- **Edit `firmware/common/` only**, then run `python tools/sync_common.py`. The
  sketch-folder copies carry a `GENERATED — DO NOT EDIT` banner.
- **Firmware/Python parity.** `python/v2vacc/controller.py` mirrors the firmware
  control laws. If you change one, change the other **in the same commit** — if
  they drift, the twin silently predicts a controller that is not the one running
  on the car, and every conclusion from it becomes wrong without anything
  visibly failing. `python/tests/test_control.py` encodes the numbers both sides
  must agree on.
- **The twin is fed the same _commands_ as the hardware, never its measured
  output.** If real speed leaks into the twin's state it stops being a prediction
  and becomes an echo: it will track beautifully and tell you nothing.
- **The twin makes no real-time braking decisions.** Braking uses live sensor and
  V2V data with conservative margins, precisely because the model is imperfect.
- **Twin validation is two-stage, to avoid circularity.** (1) twin vs closed-form
  2nd-order formulas → validates the simulation *code*; (2) twin vs **fresh**
  hardware runs, different from the fitting data → validates the *model*.
  Validating against the fitting data alone is circular.
- **Verify gains in `sim_only.py` before flashing.** Trying gains on the car
  first is how you destroy a gearbox.

## 9. The intended workflow once hardware arrives

```
1. step_response.ino  →  data/step.csv            (real motor, once, offline)
2. sysid_fit.py       →  data/plant_params.json   (K, τ, L, dead-zone)
3. design_control.py  →  gains + margins + a paste-ready #define block
4. paste into config.h, run tools/sync_common.py
5. sim_only.py        →  verify on the model BEFORE touching hardware
6. flash with SIM_PLANT 0, run, log_serial.py  →  data/run.csv
7. dashboard.py       →  real vs twin, side by side
```

## 10. What is next

**Immediately (no hardware needed):**
- Flash both boards with `SIM_PLANT 1` and confirm the link end to end — the
  follower should reach `PREDICT` and `EMERG` during the lead's scenario, with
  `pkt_lost ≈ 0`. **This is the Review 1 deliverable.**
- Capture it with `log_serial.py` so it can be replayed with
  `dashboard.py --replay` if hardware misbehaves on the day.

**When the BOM arrives (Review 2):** real step response → sysID → PID on
hardware → outer loop → root locus / Bode figures → lead-lag (or a justified
statement that it is not needed).

**Review 3:** predictive braking integrated on hardware, twin + dashboard live,
link-failure demonstrated live (walk a hand between the cars), full integration
test, final report.

**Not started — the obvious extensions, and what an examiner will probe:**
- **String stability for 3+ vehicles.** The constant time-gap policy is the
  standard route to it; this is the natural extension.
- **State-space model + formal stability proof** (Review 3 stretch goal).
- **Kalman/weighted sensor fusion** instead of OR-logic. The OR is deliberate —
  provable, debuggable, and it cannot be destabilised by a bad covariance
  estimate — but a weighted fusion would give a better `v_lead` estimate in
  degraded mode specifically.
- **Link authentication.** Anyone can forge a braking packet. Out of scope but
  worth naming; ESP-NOW supports a pre-shared key as a first mitigation.
- **Loop-jitter measurement.** No histogram of actual loop timing yet, so no
  hard real-time claim should be made.

## 11. Where to start reading

1. `README.md` — orientation and the quick-start commands
2. `docs/CONTROL_DESIGN.md` — the theory with derivations (read before any viva)
3. `docs/ARCHITECTURE.md` — how the pieces fit and why
4. `firmware/follow_node/follow_node.ino` — the control system itself, heavily
   commented with the reasoning
5. `docs/REVIEW_CHECKLIST.md` — deliverables and anticipated questions

Then run `python python/scripts/sim_only.py --compare --fair --plot` and read the
figure. That single plot contains the project's central claim.
