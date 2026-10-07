# V2V-Assisted Predictive Adaptive Cruise Control

**BECE302L Control Systems** · Fall 2026–27 · VIT Vellore · Faculty: Dr Gopinath

Two ESP32 "vehicles". The lead one broadcasts its speed and braking state over a
direct wireless link. The follower fuses that broadcast with its own ultrasonic
range sensor, holds a constant **time gap** (not a fixed distance), and brakes
**predictively** — on closing rate and on the lead's braking announcement, rather
than waiting for the gap to become dangerous. A Python **digital twin** runs
alongside on a laptop and validates the real hardware against the control model.

---

## The one-sentence reason this is not just a PID demo

A conventional ACC cannot know the car ahead is braking until the gap has
already started shrinking — it has to wait for the integral of a speed
difference to rise above sensor noise, and that wait is the collision. The V2V
broadcast removes the wait entirely, turning a feedback problem into a
**feedforward** one.

Measured on the closed-loop twin, equal headway in both runs:

| | with V2V | sensor only |
|---|---|---|
| speed shed in the first 0.5 s of an emergency stop | **39.3 cm/s** | 12.6 cm/s |
| minimum clearance | **45.3 cm** | 23.5 cm |
| braking layer engaged at all? | yes, 60 ms | never — eased off through the speed loop |

Reproduce with `python python/scripts/sim_only.py --compare --fair`.

---

## Run it right now, with no hardware

Everything below works on a laptop with nothing plugged in.

```bash
pip install -r python/requirements.txt
```

```bash
python python/scripts/sim_only.py --compare --fair --plot
```

```bash
python python/scripts/dashboard.py --demo
```

```bash
python python/tests/test_control.py
```

`--demo` drives the live dashboard from the closed-loop twin, so you can build
and rehearse the whole demonstration before the motors arrive.

---

## You have two ESP32s and nothing else. Start here.

The firmware ships with `SIM_PLANT 1` (in `firmware/common/config.h`). In that
mode each node runs a **discrete motor model internally** in place of the
encoder, and the follower derives the gap by integrating relative velocity from
the V2V packet.

The radio link, the control laws, the state machine, the link-health logic and
the telemetry are all **real**. Only the physics is simulated.

```
flash lead_node to board A  ─┐
                             ├─ real ESP-NOW link ─→ real control loop ─→ real telemetry
flash follow_node to board B ─┘
```

That is a genuine, demonstrable two-node distributed control system today. When
the motors arrive, set `SIM_PLANT 0`, re-run `python tools/sync_common.py`, and
the same control code drives real hardware. **No control code changes.** The
`SIM_PLANT` flag is read only by the I/O layer (`vehicle_io.h`) — the controller
cannot tell the difference, which is exactly why a controller validated in
simulation is the same logic that runs on the car.

### Flashing

1. Open `firmware/lead_node/lead_node.ino` in Arduino IDE → board "ESP32 Dev
   Module" → upload to board A.
2. Open `firmware/follow_node/follow_node.ino` → upload to board B.
3. Open two serial monitors at **115200**.

**No MAC addresses to configure.** The link uses ESP-NOW **broadcast**
(`FF:FF:FF:FF:FF:FF`). That is both the practical choice (nothing to look up,
type in, or keep in sync) and the correct one — real V2V Basic Safety Messages
*are* broadcast, because a braking announcement is addressed to every vehicle in
range, not to one negotiated peer. `firmware/tools/mac_address` is still there if
you want the MACs for a unicast link-reliability experiment.

Both nodes must be on the same WiFi channel — `V2V_WIFI_CHANNEL`, set to 1 in
both sketches.

You should see the follower's mode switch to `PREDICT` and then `EMERG` the
instant the lead's scenario hits its emergency stop at t = 24 s. That, on two
bare boards, is the Review 1 deliverable.

---

## Repository layout

```
firmware/
  common/              SINGLE SOURCE OF TRUTH for shared headers
    config.h             every tunable parameter, one place
    v2v_protocol.h       wire format: packed struct, CRC, seq numbers
    filters.h            median / low-pass / differentiator / lead-lag
    pid.h                PID: deriv-on-measurement, filtered, anti-windup
    vehicle_io.h         the ONLY file that knows about SIM_PLANT
  lead_node/           lead vehicle: scripted profile + 20 Hz broadcast
  follow_node/         THE CONTROL SYSTEM: cascaded loops, predictive
                       braking, supervisor, link health
  tools/
    mac_address/       prints MAC + channel
    step_response/     system-identification data capture

python/
  v2vacc/
    plant.py           motor + gap-kinematics models
    controller.py      Python MIRROR of the firmware control laws
    twin.py            digital twin: open-loop (live) + closed-loop (offline)
    telemetry.py       telemetry parser
    serial_link.py     threaded serial reader + log replay
  scripts/
    sim_only.py        offline closed-loop sim + the V2V-vs-no-V2V study
    sysid_fit.py       fit the motor transfer function from step data
    design_control.py  PID + lead-lag design, margins, root locus, Bode
    dashboard.py       live dashboard with the twin plotted alongside
    log_serial.py      capture serial to a file
  tests/
    test_control.py    36 tests pinning the control-law behaviour

matlab/
  sysid_fit.m          MATLAB companion to the fitter
  design_pid_leadlag.m root locus / Bode / margins / lead design

docs/
  ARCHITECTURE.md      how the pieces fit, and why each one is there
  CONTROL_DESIGN.md    the control theory, with the derivations
  WIRING.md            pinout, wiring, and the ways to destroy a board
  REVIEW_CHECKLIST.md  what to have ready for each graded review
  ONBOARDING_PROMPT.md paste-into-a-new-session brief covering the whole repo
```

### `firmware/common/` and the generated copies

The Arduino IDE copies a sketch folder to a temp directory before compiling,
which breaks `#include "../common/config.h"`. So each sketch folder holds its own
copy, generated from `firmware/common/`:

```bash
python tools/sync_common.py
```

**Edit `firmware/common/` only.** The copies carry a `GENERATED — DO NOT EDIT`
banner. `python tools/sync_common.py --check` fails if any copy is stale.

---

## The workflow, end to end

```
 1. step_response.ino  ──→  data/step.csv          (real motor, once, offline)
 2. sysid_fit.py       ──→  data/plant_params.json (K, tau, L, dead-zone)
 3. design_control.py  ──→  gains + margins + the #define block
 4. paste into config.h, python tools/sync_common.py
 5. sim_only.py        ──→  verify on the model BEFORE touching hardware
 6. flash, run, log_serial.py  ──→  data/run.csv
 7. dashboard.py       ──→  real vs twin, side by side
```

Step 5 is not optional. Trying gains on the car first is how you destroy a
gearbox and lose an afternoon.

---

## Digital twin — the one rule

**The twin is fed the same _commands_ as the hardware. It is never fed the
hardware's measured output.**

If the real speed ever leaks into the twin's state it stops being a prediction
and becomes an echo: it will track beautifully and tell you nothing. In
`dashboard.py` the twin receives `record.duty` and nothing else. The gap between
the red twin trace and the blue measured trace is therefore pure model error —
friction, battery sag, wheel slip, unmodelled delay — and that divergence is the
thing worth discussing.

The twin **makes no real-time braking decisions**. Braking uses live sensor and
V2V data with conservative margins, precisely because the model is known to be
imperfect.

**Validation is two-stage, to avoid circularity:**

1. **Twin vs. closed-form 2nd-order formulas** — proves the *simulation code* is
   implemented correctly.
2. **Twin vs. _fresh_ hardware runs**, different from the runs used to fit the
   model — proves the *model* captures reality.

Stage 2 against the fitting data alone would be circular: of course the model
matches the data it was fitted to. If asked "how do you know the twin is right?",
that two-stage split is the answer.

---

## What is deliberately *not* claimed

Honesty here is worth more marks than overreach, and these are the questions an
examiner will actually ask.

- **Neither controller collides in the standard scenario, and that is expected.**
  This car stops in about 5 cm but follows at about 60 cm — it is roughly an
  order of magnitude more over-provisioned than a real vehicle, because braking
  authority does not scale down the way kinetic energy does. **Report reaction
  latency and clearance margin, not collision counts.** `sim_only.py` says so in
  its own output.
- **V2V is not load-bearing for basic safety.** The TTC and gap-floor layers keep
  the minimum gap positive even at 95% packet loss
  (`sim_only.py --sweep-loss` demonstrates this). V2V improves the margin; the
  sensor layers prevent the crash. That separation is deliberate, and it is the
  honest answer to the jamming-attack paper in the literature survey.
- **The linear model is only valid above the dead-zone.** A geared DC motor does
  not move at all below its break-away duty, so the "gain" changes with operating
  point. `sysid_fit.py` fits each step level separately and reports the spread
  rather than hiding it behind one number.
- **The lead-lag compensator may turn out to be unnecessary.** With lambda
  tuning the phase margin often lands near 70° already. `design_control.py` will
  say so rather than inventing a compensator to pad the report. Explaining why
  you do not need one is a better answer than adding one you cannot justify.

---

## Status

| Area | State |
|---|---|
| V2V link, protocol, CRC, loss statistics | written, needs bring-up on real boards |
| Cascaded control loops, predictive braking, supervisor | written, verified in simulation |
| Link-loss degradation | written, verified in simulation |
| Digital twin + dashboard | written, verified in `--demo` mode |
| System-identification pipeline | written, verified against synthetic data with known parameters |
| Control design (margins, lead-lag) | written, gain margin cross-checked analytically |
| Real motor, encoder, ultrasonic | **blocked on hardware** — `SIM_PLANT 1` stands in |
| State-space model + stability proof | not started (Review 3 stretch goal) |
| String stability for 3+ vehicles | not started (natural extension) |

See `docs/REVIEW_CHECKLIST.md` for what to have ready at each review.

**SDG alignment:** primary SDG 3 (Target 3.6 — reduce road traffic deaths through
earlier, predictive braking); secondary SDG 9 (low-cost embedded V2V).
