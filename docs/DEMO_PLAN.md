# Demo plan — bench build, review Wednesday 2026-10-14

**This is the active plan.** `BUILD_PLAN.md` is the 5-day track version, kept for
reference if the deadline ever moves.

Split: **~60% software, ~40% hardware.** Every control loop that matters runs on
real hardware; only the lead vehicle's physics is modelled.

---

## 1. The architecture, and why it halves the build

The lead vehicle does not need a motor. It only needs to **broadcast**. So we run
`SIM_PLANT` asymmetrically:

```
   LEAD NODE  (SIM_PLANT 1)                FOLLOWER NODE  (SIM_PLANT 0)
  ┌──────────────────────────┐            ┌────────────────────────────────┐
  │ potentiometer = throttle │            │ REAL DC motor + encoder        │
  │ button       = brake     │            │ REAL L298N PWM                 │
  │ OLED         = dash      │            │ REAL closed-loop PID           │
  │ simulated vehicle physics│  ESP-NOW   │ REAL HC-SR04 obstacle sensing  │
  │ REAL V2V broadcast ──────┼───────────►│ OLED + WS2812 mode bar         │
  └──────────────────────────┘    (real)  └────────────────────────────────┘
                                                        │ USB telemetry
                                                        ▼
                                          laptop: dashboard + digital twin
```

**One drivetrain instead of two.** No track, no chassis, no diorama, no
batteries. It all sits on a desk, and the panel can lean over it.

### What is genuinely real

| Real hardware | Modelled |
|---|---|
| ESP-NOW V2V link, CRC, packet-loss counting | lead vehicle's motor physics |
| Follower's DC motor, encoder, L298N, PWM | inter-vehicle gap kinematics |
| Follower's closed-loop PID speed control | |
| HC-SR04 obstacle detection | |
| OLED, WS2812 mode bar, throttle pot, brake button | |
| Supervisory state machine, link-health logic | |
| Telemetry, dashboard, digital-twin comparison | |

If asked what is simulated, answer plainly: *the lead vehicle's powertrain.*
Everything the controller does, and everything it senses, is real. That is
standard hardware-in-the-loop practice — you do not build a second vehicle to
test a follower.

### The gap is HIL-integrated, and that frees the sensor

`GAP_SOURCE` in `config.h` is **independent of `SIM_PLANT`**. On the bench we
run `SIM_PLANT 0` (real motor, real encoder) together with `GAP_FROM_HIL`:

```
d(gap)/dt = v_lead - v_follow
```

integrated using the follower's **real measured speed** and the lead's
**V2V-reported speed**. The powertrain and the sensing are real; the road is
modelled. No track length constrains the spacing, so the full-range parameters
stay in place — 0.40 m/s cruise, 1.5 s headway, which is ~118 rpm at the wheel:
clearly visible, and clearly audible when it brakes.

If the link drops, the gap stops updating. That is not a cheat — it is exactly
what losing your only source of information about the vehicle ahead looks like,
and it is why the degraded mode widens the time gap instead of carrying on
regardless.

### The ultrasonic has one clear job

**Forward obstacle detection.** Put your hand in front of it and the follower
emergency-brakes, the LED bar goes red, the dashboard logs `EMERG`.

No explanation needed — any audience understands it in one second. That is worth
more in a five-minute review than a subtle gap-regulation argument nobody can
see from two metres away.

It is also the one danger signal that owes nothing to the radio *or* to the gap
model: a real sensor seeing a real object, trip-confirmed over 3 consecutive
reads (a single spurious short read is common on an ultrasonic and must not slam
the brakes on) with hysteresis on release.

---

## 2. Bill of materials

| Item | Qty | ₹ | Have? |
|---|---|---|---|
| ESP32 WROOM-32 | 2 | — | **yes** |
| DC geared motor + encoder | **1** | 600 | buy |
| L298N module | **1** | 150 | buy |
| HC-SR04 | 1 | 80 | buy |
| 10 kΩ potentiometer | 1 | 20 | buy |
| Push buttons | 3 | 30 | buy |
| SSD1306 OLED 128×64 I²C | 2 | 400 | buy |
| WS2812B strip (cut to 5) | 1 | 200 | buy |
| Resistors 1 kΩ + 2 kΩ | 1 set | 20 | buy — **ECHO divider** |
| 1000 µF capacitor | 1 | 25 | buy |
| 5 V 2 A supply for the motor | 1 | 250 | buy |
| Breadboard + jumpers | 2 | 250 | buy |
| **Total** | | **≈ ₹2,000** | |

Only **one** motor. If the OLEDs or the LED strip are unavailable, the demo still
works — see §6.

---

## 3. Schedule

### Saturday + Sunday — software (not blocked on any parts)

- [x] scenario engine: 5 scenarios, button- and dashboard-triggered
- [x] throttle pot + brake button on the lead
- [x] OLED + WS2812 mode-bar drivers (compile-time optional)
- [x] `GAP_SOURCE` switch + HIL gap integration
- [x] forward obstacle sensor with confirmation count and hysteresis
- [x] sliding-window encoder speed measurement
- [x] state-space model + LQR analysis (report deliverable)
- [ ] dashboard scenario panel

All of this is testable **today** with `SIM_PLANT 1` on both boards, no motor
required.

### Monday — hardware

- Wire the follower drivetrain: motor + encoder + L298N, own supply, grounds
  tied, 1000 µF across the motor rail.
- Confirm the wheel spins both ways and the encoder counts up **and** down.
- Capture the step response and fit the plant:
  ```bash
  python python/scripts/log_serial.py --port COM5 --out data/step.csv --duration 45
  python python/scripts/sysid_fit.py data/step.csv --plot
  python python/scripts/design_control.py --params data/plant_params.json --plot
  python tools/sync_common.py
  ```
- Fit the HC-SR04 with the **1 k / 2 k divider**.
- Set `SIM_PLANT 0` on the follower (leave `GAP_SOURCE` as `GAP_FROM_HIL`),
  flash, close the loop.

> **GATE:** the real motor holds a commanded speed, and your hand in front of the
> sensor stops it. Nothing else matters until this works.

### Tuesday — integrate, record, rehearse

- OLED + LED bar on.
- Run all five scenarios back to back.
- **Record the golden run** (§5).
- Generate report figures.
- Rehearse the 5-minute walkthrough six times.

---

## 4. Scenarios

Triggered by button on the lead console, or from the dashboard.

| # | Name | What you do | What the panel sees |
|---|---|---|---|
| 0 | **Manual drive** | turn the throttle pot | real motor tracks the lead's speed over V2V |
| 1 | **Normal follow** | press S1 | lead cruises, follower holds the time gap |
| 2 | **Predictive brake** | press S2 | lead eases off → follower slows **before the gap closes**, LED amber |
| 3 | **Emergency stop** | press S3 | lead stops hard → follower full-brakes, LED red |
| 4 | **Comm loss** | press S4 (or put a tin over the lead) | link drops → `DEGRADED` → **time gap visibly widens** → recovers |
| 5 | **Obstacle** | put your hand in front of the sensor | immediate `EMERG`, motor stops |

**Scenarios 4 and 5 are the pair to lead with.** Together they say:

> Radio and sensor have complementary blind spots. The controller detects which
> channel it has lost and reconfigures its safety margin accordingly.

Scenario 4 kills the radio and the sensor carries on. Scenario 5 is a target the
radio knows nothing about and the sensor catches it. Run them back to back and
say that sentence in between.

---

## 5. Insurance — do this Monday night, not Tuesday

```bash
python python/scripts/log_serial.py --port COM5 --out data/golden_run.csv --duration 120
```

If anything fails live:

```bash
python python/scripts/dashboard.py --replay data/golden_run.csv
```

Real recorded hardware data, real dashboard. Keep talking; nobody has a bad five
minutes.

Also keep a `SIM_PLANT 1` build to hand. If the motor dies an hour before, the
complete control system still runs on two bare boards.

---

## 6. Cut list

Cut from the bottom.

| Priority | Item | Cut if |
|---|---|---|
| 5 | OLED screens | short on Tuesday — set `HMI_ENABLE_OLED 0` |
| 4 | WS2812 mode bar | short on Tuesday — set `HMI_ENABLE_LEDS 0` |
| 3 | Throttle pot (scenarios still work via buttons) | set `LEAD_ENABLE_POT 0` |
| 2 | HC-SR04 obstacle scenario | set `OBSTACLE_ENABLE 0` if the divider or sensor fails |
| 1 | **Never cut:** real motor under closed-loop PID + real V2V + dashboard + twin | — |

Every item above is a single `#define`. Nothing breaks the build when you turn it
off, so you can decide at the last minute.

---

## 7. The 5-minute walkthrough

1. **15 s — the problem.** Conventional ACC is reactive; it cannot know the lead
   is braking until the gap has already closed. That delay is the collision.
2. **30 s — the rig.** Point at it. Real motor under closed-loop control, real
   radio link, real sensor. The lead vehicle's powertrain is modelled, as it
   would be on any HIL bench.
3. **60 s — Scenario 1 then 2.** Show the gap held, then the predictive brake.
   Point at the dashboard as the mode goes amber *before* the gap moves.
4. **45 s — Scenario 4.** Kill the link. Point at the time gap widening.
5. **30 s — Scenario 5.** Hand in front. Instant stop.
6. **60 s — the control design.** Root locus, Bode, phase margin, λ tuning,
   state-space/LQR. This is where the Control Systems marks are.
7. **45 s — the twin.** Real vs predicted on one axis, and why it is fed commands
   rather than measurements.
8. **15 s — the claim.** Complementary blind spots, quantified: 3.1× stronger
   braking response, +22 cm clearance.
