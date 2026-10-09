# Build plan — 5-day HIL track demonstrator

Scope locked 2026-10-09. Read §1 and §10 before buying or cutting anything.

---

## 1. Locked scope

**Format:** two tethered cars on a short guided track, run at deliberately slow
speed, with scenario triggers, a live dashboard, and two physical set-pieces
(tunnel and occluder).

**Headline claim:** *sensors and radio have complementary blind spots.* An
ultrasonic sensor sees nothing through an occluder; a radio link dies inside a
shielded tunnel. The controller detects which channel it has lost and
reconfigures its safety margin accordingly.

### In scope

| | |
|---|---|
| 2 × ESP32 nodes, real ESP-NOW link | already built |
| Cascaded control, predictive braking, mode machine | already built |
| Graceful degradation on link loss | already built |
| Live dashboard + digital twin | already built |
| Real motors + encoders + L298N | **new, Day 1–2** |
| Guided track, ~1.3 m | **new, Day 2** |
| HC-SR04 real gap sensing | **new, Day 3** |
| OLED per node, WS2812 mode bar, scenario buttons | **new, Day 4** |
| Foil-lined tunnel (blocks V2V) | **new, Day 4** |
| Occluder (blocks ultrasonic) | **new, Day 4** |
| Diorama dressing | **new, Day 5, last** |

### Cut, deliberately

- Traffic-light V2I node — needs a 3rd ESP32 you won't have in time
- Servo cut-in obstacle — worst payoff per hour of anything on the list
- Batteries — see §6, we tether instead
- 3-node string stability — out of scope at 5 days

---

## 2. Bill of materials

Everything you already have is marked. Buy the rest in **one trip**, today.

| Item | Qty | ₹ each | ₹ | Notes |
|---|---|---|---|---|
| ESP32 WROOM-32 dev board | 2 | — | **have** | |
| DC geared motor + encoder | 2 | 600 | 1200 | **see §2.1 — gear ratio matters** |
| L298N driver module | 2 | 150 | 300 | |
| HC-SR04 | 1 | 80 | 80 | follower only |
| SSD1306 OLED 128×64 I²C | 2 | 200 | 400 | |
| WS2812B strip, 8 LED | 1 | 200 | 200 | cut to 5 for the mode bar |
| Tactile push buttons | 5 | 10 | 50 | scenario panel |
| Resistors 1 kΩ + 2 kΩ | 1 set | 20 | 20 | **ECHO divider — do not skip** |
| Capacitor 1000 µF 25 V | 2 | 25 | 50 | across motor supply |
| 5 V 3 A supply (or powered USB hub) | 1 | 400 | 400 | §6 |
| Foam board 5 mm, A1 | 2 | 120 | 240 | track base + rails |
| Aluminium foil | 1 | 50 | 50 | tunnel lining |
| Jumper wires, M-M / M-F | 2 sets | 100 | 200 | |
| Double-sided tape, packing tape, glue | — | 150 | 150 | |
| **Essential total** | | | **≈ ₹3,340** | |
| *Optional dressing: model trees, cones, LED strip* | | | *≈ 600* | Day 5 only |

### 2.1 Motor choice — the one spec that matters

You are running at **0.15 m/s**, which is slow. Buy the **highest gear ratio you
can get** (1:100 or more; "N20 1:150" or "TT motor with encoder" are both fine).

Reason: a low-ratio motor at 0.15 m/s sits barely above its dead-zone, where the
plant is most nonlinear and the control is worst. A high ratio puts 0.15 m/s
comfortably mid-range, and gives you more encoder pulses per metre as a bonus.

**If the shop only has low-ratio motors, tell me — I'll re-derive the speed and
gap targets around what you can actually get.**

---

## 3. Day-by-day plan

Each day ends in a **gate**. If you fail a gate, go to §10 and cut, don't push.

### Day 1 — one drivetrain, characterised

- Wire **one** motor + encoder + L298N to one ESP32. Motor on its own supply,
  grounds tied, 1000 µF across the motor rail.
- Confirm the wheel spins both ways and the encoder counts up *and* down.
- Run `firmware/tools/step_response`, capture with:
  ```bash
  python python/scripts/log_serial.py --port COM5 --out data/step.csv --duration 45
  ```
- Fit it:
  ```bash
  python python/scripts/sysid_fit.py data/step.csv --plot
  ```
- Design gains, paste into `config.h`, sync:
  ```bash
  python python/scripts/design_control.py --params data/plant_params.json --plot
  python tools/sync_common.py
  ```

> **GATE 1:** you have a fitted `K`, `τ`, `L` and dead-zone, and the motor holds
> a commanded speed under closed-loop PID on the bench. Nothing else matters
> until this works.

### Day 2 — second drivetrain + track

- Build the second node identically.
- Build the track (§4). Foam board, rails, run-up and run-out marked.
- Mount both cars. Fit the **reflector plate** to the lead (§4.3 — this is not
  optional, the beam is wider than the car).
- Re-tune for slow speed (§7) and verify in simulation **before** flashing:
  ```bash
  python python/scripts/sim_only.py --compare --fair --plot
  ```

> **GATE 2:** both cars drive along the track under speed control, in a straight
> line, without touching the rails. **If you are behind here, cut to Tier 1
> (§10) now, not tomorrow.**

### Day 3 — closed the loop on real hardware

- Fit the HC-SR04 to the follower with the **1 k / 2 k divider**. Mount it at
  rail-clearing height (§4.2).
- Set `SIM_PLANT 0`, sync headers, flash both.
- Get **Scenario 1 (Normal Follow)** and **Scenario 2 (Predictive Brake)**
  working end to end, with the dashboard live.
- Expect to spend most of today on range-sensor noise. The median + low-pass
  chain is already in the firmware; tune `ULTRA_LPF_FC_HZ` and
  `CLOSING_LPF_FC_HZ` if the closing rate is jumpy.

> **GATE 3:** the follower holds a visible, stable gap, and visibly brakes early
> when the lead brakes. **This is your minimum viable demo.** Record it tonight
> (§9) — from here on you always have something to show.

### Day 4 — the differentiators

- **HMI:** OLED on each node, WS2812 mode bar on the follower, 5 scenario
  buttons on a side panel wired to the lead (§5).
- **Tunnel:** foil-lined cardboard arch over the track (§4.4). Verify it
  actually drops the link — watch `pkt_lost` climb and the mode go `DEGRADED`.
- **Occluder:** a flat card that blocks the ultrasonic line of sight while the
  radio still gets through (§4.5).
- Wire up **Scenarios 3 and 4**.

> **GATE 4:** all four scenarios run on a button press, repeatably, back to back.

### Day 5 — dressing and rehearsal

- Diorama dressing: painted road, trees, cones, barrier strip, lighting.
  **Do this last and only now.**
- Rehearse the full 5-minute presentation **at least six times**.
- Record a golden run (§9).
- Generate report figures.

> **GATE 5:** you can run the whole demo start to finish, twice, without
> touching a keyboard except to click scenario buttons.

---

## 4. Track and set-pieces

### 4.1 Track dimensions

```
    ◄───────────────────── 130 cm ──────────────────────►
    ┌────────────────────────────────────────────────────┐
    │ ▓▓▓▓▓▓▓▓▓▓▓▓ rail (3 cm tall) ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  ▲
    │                                                    │  │
    │   [LEAD]◄─27cm─►[FOLLOWER]                         │ 22 cm
    │                                                    │  │
    │ ▓▓▓▓▓▓▓▓▓▓▓▓ rail (3 cm tall) ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  ▼
    └────────────────────────────────────────────────────┘
      ▲ 15cm run-up                         run-out 15cm ▲
```

- **Base:** 5 mm foam board, 130 × 22 cm. Two sheets laminated if it flexes.
- **Rails:** foam board strips, **3 cm tall, no taller** (see §4.2). Line the
  inner faces with packing tape — it's slippery and cheap.
- **Clearance:** track width 22 cm, car width ~16 cm → 3 cm either side. Enough
  to stop drift, loose enough not to jam.
- Minimum workable length is 88 cm (computed from car length + cruise gap +
  run-up). 130 cm gives you comfortable margin and a longer, more watchable run.

### 4.2 The rail height constraint — do not ignore this

The HC-SR04 has a ~15° half-angle beam. At the 27 cm cruise gap the beam is
**14 cm wide**. If the rails rise into that cone they produce false echoes and
your gap signal becomes garbage.

**Rule: mount the sensor 5–6 cm above the track surface, and keep rails ≤ 3 cm.**
The beam then clears the rails entirely.

### 4.3 The reflector plate — also not optional

At 27 cm the beam is 14 cm wide, which is probably **wider than your car**. A
narrow car body scatters the echo and the range reading becomes unreliable.

Fit a **flat plate, 16–18 cm wide × 8 cm tall**, to the back of the lead car,
vertical and square to the track. Foam board or stiff card. Flat and
perpendicular matters far more than what it's made of.

This is also where the brake-light LEDs go, if you add them.

### 4.4 Tunnel (blocks V2V, not the sensor)

Cardboard arch spanning the track, ~25 cm long, **lined inside with aluminium
foil, continuous, including the top**. Foil blocks 2.4 GHz well.

Test it before committing: park both nodes with the tunnel between them and
watch `pkt_lost` on the dashboard. If the link survives, add a second foil layer
or lengthen the tunnel. A gap in the foil is usually the culprit.

> This is a *physical* demonstration of graceful degradation. The car drives in,
> the link dies, the dashboard flips to `DEGRADED`, the time gap visibly widens
> from 1.0 s to 1.6 s, and the car keeps going safely on ultrasonic alone.

### 4.5 Occluder (blocks the sensor, not V2V)

A flat card ~20 × 15 cm on a small stand, placed to break the ultrasonic line of
sight between the cars while leaving the radio path intact (radio goes through
card trivially).

> The mirror image of the tunnel: the follower's sensor reports "clear road", the
> lead brakes, and the follower **still** brakes — because the radio told it.
> A sensor-only controller does nothing at all. **This is your strongest single
> moment; rehearse it most.**

---

## 5. Wiring

### 5.1 FOLLOWER node (ESP32 #2)

| Function | GPIO | Notes |
|---|---|---|
| L298N IN1 | 26 | direction |
| L298N IN2 | 27 | direction |
| L298N ENA | 14 | PWM, 20 kHz, 10-bit |
| Encoder A | 34 | input-only, 10 kΩ pull-up to 3.3 V |
| Encoder B | 35 | input-only, 10 kΩ pull-up to 3.3 V |
| HC-SR04 TRIG | 5 | direct |
| HC-SR04 ECHO | 18 | **1 kΩ / 2 kΩ divider — 5 V will kill the pin** |
| OLED SDA | 21 | I²C, addr 0x3C |
| OLED SCL | 22 | I²C |
| WS2812 DIN | 4 | 5 LEDs: FOLLOW / PREDICT / EMERG / V2V OK / V2V LOST |
| Onboard LED | 2 | |

Free: 13, 15, 16, 17, 19, 23, 25, 32, 33

### 5.2 LEAD node (ESP32 #1)

| Function | GPIO | Notes |
|---|---|---|
| L298N IN1 | 26 | |
| L298N IN2 | 27 | |
| L298N ENA | 14 | |
| Encoder A | 34 | pull-up |
| Encoder B | 35 | pull-up |
| OLED SDA | 21 | |
| OLED SCL | 22 | |
| Brake-light WS2812 | 4 | 2 LEDs on the reflector plate, optional |
| Scenario btn 1 | 32 | `INPUT_PULLUP`, button to GND |
| Scenario btn 2 | 33 | |
| Scenario btn 3 | 25 | |
| Scenario btn 4 | 19 | |
| Scenario btn 5 | 23 | |
| Onboard LED | 2 | |

**Buttons go on the lead**, because every scenario is a statement about what the
*lead* does. Mount them on a small panel beside the track and run a ribbon cable
to the car — the lead is tethered anyway, so one more cable costs nothing, and
you are not chasing a moving car to press a button.

Scenarios are *also* triggerable from the dashboard, so the buttons are the
showpiece, not a single point of failure.

### 5.3 The ECHO divider

```
  ECHO ──┬── 1kΩ ──┬── GPIO18
         │         │
         │        2kΩ
         │         │
        ───       GND
```
5 V × 2/3 = 3.3 V. TRIG needs no divider (it's an output, and the HC-SR04 reads
3.3 V as a valid high).

---

## 6. Power and the tether

**Both cars run off USB, tethered to a powered hub. No batteries.**

| | |
|---|---|
| Logic | USB 5 V from the hub |
| Motors | separate 5 V 3 A supply, **grounds tied to the ESP32s** |
| Telemetry | over the same USB |

Why tether:

- No battery sag mid-demo, so every run is identical
- No charging between runs, no dead battery at the worst moment
- Telemetry from **both** nodes reaches the dashboard live
- On a 130 cm track a managed cable is a non-issue

And it's defensible: real validation mules are tethered with instrumentation
harnesses. If asked, say exactly that — this is an instrumented test vehicle,
not a product.

Dress the cables overhead or from the track ends with a little slack loop, so
they never tug the cars.

**Never power the motors from the ESP32's 5 V pin.** The L298N draws amps on
stall, the regulator browns out, the ESP32 resets mid-control-loop, and you lose
a day blaming the code.

---

## 7. Retuning for slow speed

Change these in `firmware/common/config.h`, then `python tools/sync_common.py`.

| Parameter | Now | New | Why |
|---|---|---|---|
| `V_MAX_MPS` | 0.60 | **0.25** | track is short |
| cruise speed | 0.35 | **0.15** | audience can *see* the braking happen |
| `T_GAP_S` | 1.50 | **1.00** | 27 cm gap fits the track |
| `T_GAP_DEGRADED_S` | 2.20 | **1.60** | same ratio, still visibly wider |
| `D_STANDSTILL_M` | 0.20 | **0.12** | bumper gap at rest |
| `TTC_BRAKE_S` | 1.20 | 1.20 | unchanged |
| `D_BRAKE_FLOOR_M` | 0.15 | **0.08** | scaled with the gap |
| `ULTRA_MAX_M` | 2.50 | **1.20** | anything beyond the track is a false echo |

Resulting cruise gap: `0.12 + 1.0 × 0.15` = **27 cm**. Traverse time over 1 m of
free travel: **6.7 s** — long enough to narrate.

### 7.1 Encoder quantisation — a change I need to make in firmware

At 0.15 m/s you get **5.5 encoder pulses per 20 ms control tick**, which
quantises the speed measurement to **2.7 cm/s** — about 18% of cruise speed.
The existing low-pass copes, but it costs phase margin.

Fix: measure speed over a **sliding 60 ms window** (3 ticks) while still running
the control loop at 50 Hz. Trades a little latency for 3× the resolution, which
is the right trade at this speed. I'll implement it when you give the word.

If you get a higher-ratio gearmotor than the current `ENC_GEAR_RATIO 34`, this
problem shrinks on its own — another reason for §2.1.

---

## 8. Scenarios

| # | Name | What happens | Mode sequence | Hardware |
|---|---|---|---|---|
| 1 | **Normal Adaptive Cruise** | lead cruises, follower holds the time gap | `FOLLOW` | track only |
| 2 | **Predictive Braking (V2V)** | lead brakes; follower eases off **before the gap closes** | `FOLLOW → PREDICT → FOLLOW` | track only |
| 3 | **Emergency Stop** | lead stops hard; follower full-brakes | `FOLLOW → EMERG → STANDBY` | track only |
| 4 | **Comm Loss (Tunnel)** | V2V blocked; follower falls back to sensor-only and **widens the gap** | `FOLLOW → DEGRADED → LOST → FOLLOW` | tunnel |
| 5 | **Occluded Lead (blind spot)** | sensor sees nothing; V2V still brakes the car | `CRUISE → PREDICT` | occluder |

Scenarios 4 and 5 are the pair that carries the headline claim. Run them
**back to back**, in that order, and say the sentence from §1 between them.

---

## 9. Insurance — do this on Day 3, not Day 5

Record a golden run the moment Gate 3 passes:

```bash
python python/scripts/log_serial.py --port COM5 --out data/golden_run.csv --duration 120
```

Then, if anything fails live:

```bash
python python/scripts/dashboard.py --replay data/golden_run.csv
```

That replays **real recorded hardware data** through the real dashboard. You keep
talking, the panel sees real results, and nobody has a bad five minutes. Re-record
a better one each evening as the rig improves.

Also keep `SIM_PLANT 1` builds on a USB stick. If a motor dies an hour before,
you can still show the full control system running on two bare boards.

---

## 10. If you fall behind — cut in this order

Cut from the bottom. Never cut upward.

| Priority | Item | Cut if |
|---|---|---|
| 7 | Diorama dressing | behind on Day 5 |
| 6 | Brake-light LEDs on the lead | behind on Day 4 |
| 5 | OLED screens | behind on Day 4 |
| 4 | Physical scenario buttons → use dashboard clicks | behind on Day 4 |
| 3 | Occluder scenario | behind on Day 4 morning |
| 2 | Tunnel scenario | behind on Day 3 |
| 1 | **Never cut:** two cars following with a visible gap, predictive braking, live dashboard | — |

**Hard rule:** if Gate 2 fails on Day 2, stop building new things and spend Day 3
making one car follow one car. A rock-solid two-scenario demo beats a
half-working five-scenario one, every time, in front of a panel.

---

## 11. What I still need to change in the code

Tracked so nothing is forgotten:

- [ ] slow-speed parameter set (§7)
- [ ] sliding-window speed measurement (§7.1)
- [ ] scenario engine: 5 scenarios, button + serial triggered, auto-return to start
- [ ] OLED driver and screen layout for both nodes
- [ ] WS2812 mode-bar driver
- [ ] dashboard scenario panel + scenario annotation on the plots
- [ ] `ULTRA_MAX_M` and range-gating for the short track
- [ ] state-space + LQR analysis script (report deliverable)
