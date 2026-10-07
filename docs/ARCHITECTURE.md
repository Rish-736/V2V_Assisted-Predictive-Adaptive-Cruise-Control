# Architecture

How the pieces fit together, and why each one is there.

---

## 1. System overview

```
        LEAD NODE (ESP32)                      FOLLOW NODE (ESP32)
  ┌───────────────────────────┐          ┌──────────────────────────────┐
  │ scripted speed profile    │          │ SUPERVISOR (state machine)   │
  │          ↓                │          │  STANDBY/CRUISE/FOLLOW/      │
  │ PID speed loop            │          │  PREDICT/EMERG/FAULT         │
  │          ↓                │          │  + link: NOMINAL/DEGRADED/   │
  │ motor (or SIM_PLANT)      │          │          LOST                │
  │          ↓                │          │            ↓                 │
  │ accel estimate            │          │  OUTER LOOP (time gap)       │
  │ braking? hazard?          │          │   d_des = d₀ + T_h·v         │
  │          ↓                │          │   v_tgt = v_lead + K·(d−d_des)│
  │ ESP-NOW broadcast @20 Hz ─┼──────────┼→ ↓                           │
  └───────────┬───────────────┘   radio  │  INNER LOOP (PID + lead-lag) │
              │                          │            ↓                 │
              │                          │  motor (or SIM_PLANT)        │
              ↓ serial                   │            ↓                 │
         ┌────────────────────────────┐  │  HC-SR04 ──→ gap, closing    │
         │        LAPTOP              │←─┼─ serial telemetry @25 Hz     │
         │  digital twin + dashboard  │  └──────────────────────────────┘
         └────────────────────────────┘
```

---

## 2. Why two independent nodes

A single ESP32 with two motors would be simpler and would be **the wrong
project**. The entire premise is that the follower cannot see inside the lead
vehicle and must be *told*. Sharing memory between the two controllers would
quietly remove the problem being solved.

Two nodes also force the parts that make this a systems project rather than a
control exercise: a wire protocol, packet loss, latency, link-health detection,
and a defined degradation path.

---

## 3. The `SIM_PLANT` flag

**`firmware/common/vehicle_io.h` is the only file that knows about it.**

```c
vio_begin()        bring up I/O (or the sim state)
vio_set_duty(u)    command normalised duty, u ∈ [−1, +1]
vio_speed()        measured forward speed [m/s]
vio_range()        measured gap to the vehicle ahead [m]
vio_update(dt)     advance sensors / the simulated plant
```

Everything above that line — PID, time-gap law, predictive braking, supervisor —
calls the same five functions whether it is driving an L298N or a difference
equation.

| | `SIM_PLANT 1` | `SIM_PLANT 0` |
|---|---|---|
| speed | discrete first-order model + noise | quadrature encoder |
| gap | integrate `(v_lead − v_follow)` | HC-SR04, median + low-pass |
| actuation | state update | L298N PWM at 20 kHz |
| **V2V radio** | **real** | real |
| **control laws** | **real** | real |
| **state machine** | **real** | real |
| **telemetry** | **real** | real |

The simulated plant uses the *same* model as the Python twin — same dead-zone,
same transport delay, same parameters from `config.h` §8 — so sim-mode telemetry
is directly comparable with twin output.

This is why a controller validated in simulation is the same binary logic that
runs on the car: the controller genuinely cannot tell the difference.

---

## 4. The V2V protocol

26-byte packed struct, little-endian, `memcpy`'d straight into the ESP-NOW
payload. No parsing, no allocation — deterministic timing, which a real-time
control loop needs.

```c
magic(2) version(1) node_id(1) seq(4) t_ms(4)
speed(4) accel(4) flags(1) reserved(3) crc(2)
```

| field | why it exists |
|---|---|
| `seq` | lets the receiver **measure** packet loss from sequence gaps instead of assuming it. Gives a real number for the report. |
| `crc` | ESP-NOW already CRCs at the radio layer, but this also catches **struct-layout mismatch between boards built with different core versions** — a real failure that would otherwise silently decode garbage into a braking command |
| `magic` + `version` | a mismatched build fails loudly instead of quietly |
| `accel` | lets the follower distinguish "slow" from "decelerating" |

### 4.1 Broadcast, not unicast

Sent to `FF:FF:FF:FF:FF:FF`.

- **Practical:** no MAC addresses to look up, type in and keep in sync. Flash
  both boards and they talk.
- **Correct:** real V2V (DSRC / C-V2X) Basic Safety Messages *are* broadcast. A
  braking announcement is addressed to every vehicle in range, not to one
  negotiated peer.

Set `V2V_USE_UNICAST 1` in `lead_node.ino` if you want per-packet delivery
callbacks for a link-reliability experiment.

### 4.2 The braking flag

```c
flagBraking = (accel < -0.15f) || (vTargetRaw < v - 0.05f) || hazard;
```

Braking is a statement about **deceleration**, not about being slow. The original
draft used `braking = (speed < 3.0)`, which flags a slow-but-steady vehicle as
braking and *misses a fast vehicle that has just stamped on the pedal* — exactly
backwards for the one thing the follower needs to know.

---

## 5. Task structure on the follower

Single-threaded, fixed-rate, no RTOS tasks. Three rates:

| task | rate | why |
|---|---|---|
| control | 50 Hz | ~15 samples per plant time constant — comfortable |
| V2V broadcast (lead) | 20 Hz | bounds worst-case reaction latency at 50 ms |
| telemetry | 25 Hz | enough to plot, low enough not to flood the UART |

**The ESP-NOW receive callback does the minimum possible**: validate, copy,
timestamp, return. All control work happens in the fixed-rate loop so timing
stays deterministic. Shared state is guarded with `portMUX` critical sections.

### 5.1 The HC-SR04 timing problem

`pulseIn()` blocks for up to 6 ms at 1 m. That fits inside a 20 ms control
period, but it is a third of the budget and it is *variable* — which shows up as
jitter in the differentiated closing rate.

If this bites on real hardware, move to pinging every *other* control tick
(10 Hz range update, 50 Hz control), or to an interrupt-driven echo capture. The
filters are already sized for a 20 Hz range update.

---

## 6. Laptop side

### 6.1 Why the twin runs here and not on the ESP32

- It must not compete with a 50 Hz real-time control loop for CPU.
- An independent check that shares a processor with the thing it is checking is
  not independent.
- It needs numpy/matplotlib/scipy, which the ESP32 does not have.

### 6.2 Threaded serial

The dashboard must redraw at a steady frame rate regardless of what the serial
port is doing. A blocking `readline()` in the plotting loop freezes the window
every time a board resets or a cable twitches — which looks broken in a demo even
when nothing is wrong.

A daemon thread owns the port and pushes parsed records into a **bounded** queue;
the plotting loop drains whatever is there. Bounded, so a stalled consumer drops
old samples rather than growing memory without limit.

### 6.3 Three data sources

| source | use |
|---|---|
| `--demo` | closed-loop twin as a live source — no hardware at all |
| `--replay FILE` | replay a recorded run in real time |
| `--port COM5` | the real thing |

`--replay` is worth more than it looks: record one good run and you can always
show it, even if the hardware sulks on review day.

### 6.4 The firmware/Python parity rule

`python/v2vacc/controller.py` is a **hand-maintained mirror** of `pid.h`,
`filters.h` and the control logic in `follow_node.ino`.

**If you change a control law in one, change it in the other in the same commit.**

If they drift, the twin silently starts predicting a controller that is not the
one running on the car — and every conclusion drawn from it becomes wrong without
anything visibly failing. `python/tests/test_control.py` encodes the numbers both
sides must agree on.

---

## 7. Telemetry format

Plain CSV, one record per line, first field is the record type. Chosen over a
binary protocol on purpose: **you can debug the entire system with nothing but
the Arduino serial monitor**, which matters at 2 a.m. the night before a review.

```
F,t_ms,d,d_des,closing,ttc,v,v_tgt,v_lead,duty,mode,link,brake,p,i,dterm,pkt_good,pkt_lost,pkt_bad
L,t_ms,v,v_tgt,accel,duty,braking,hazard,seq,tx_ok,tx_fail
```

The PID's `p`, `i` and `d` terms are exported separately so the split between
them is visible while tuning — you can see the integrator winding up rather than
inferring it.

The parser is deliberately forgiving: a half-written line from a board that reset
mid-print is dropped, not allowed to crash a dashboard that has been running for
twenty minutes. *(Test: `test_parser_survives_garbage`.)*

---

## 8. Known architectural limitations

- **Two vehicles only.** Broadcast + `node_id` extends to a platoon, but the
  follower currently trusts the first `V2V_NODE_LEAD` packet it sees. A third
  node needs an ID filter and a notion of "the vehicle directly ahead".
- **No link authentication.** Anyone can forge a braking packet. Out of scope,
  but worth naming — it is the obvious attack and the jamming paper in the survey
  is adjacent to it.
- **Fixed control rate.** No measurement of actual loop jitter. Worth adding a
  loop-time histogram to telemetry before claiming hard real-time behaviour.
- **One-dimensional.** No steering, no lane changes, no cut-in scenario. The gap
  is a scalar.
