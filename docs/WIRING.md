# Wiring and bring-up

**None of this is needed while `SIM_PLANT 1`.** Two bare ESP32s and two USB
cables are enough to run the full control system. Come back here when the motors
arrive.

---

## 1. The four ways to destroy a board

Read this section before plugging anything in.

### 1.1 The HC-SR04 ECHO pin is 5 V. The ESP32 is 3.3 V.

Connecting ECHO directly to a GPIO is the single most common way this project
kills a board. Use a divider:

```
  ECHO ──┬── 1kΩ ──┬── GPIO18
         │         │
         │        2kΩ
         │         │
        ───       GND
```

That gives 5 V × 2/3 = 3.3 V. (Any ratio near 1:2 works — 1k/2k, 2.2k/4.7k.)

TRIG is an *output* from the ESP32 and needs no divider; the HC-SR04 reads 3.3 V
as a valid high.

*(Alternative: swap to a VL53L0X time-of-flight sensor — natively 3.3 V, I²C, no
divider, better resolution, and no acoustic cross-talk between the two cars. It
is on the optional-upgrades list for good reason.)*

### 1.2 Never power the motor from the ESP32's 5 V pin

The L298N draws amps on stall. The USB regulator will brown out, the ESP32 will
reset mid-control-loop, and you will spend a day blaming the code. **Separate
supply for the motor, grounds tied together.**

### 1.3 Common ground is mandatory

ESP32 GND, L298N GND and battery − must all be the same node. Without it the PWM
signal has no reference and the driver behaves randomly.

### 1.4 GPIO 34 and 35 are input-only and have no internal pull-ups

They are fine for encoder inputs, but you must add external pull-ups (10 kΩ to
3.3 V) if the encoder is open-collector. They cannot be used as outputs at all.

---

## 2. Pin map

From `firmware/common/config.h` §11. Change it there, then run
`python tools/sync_common.py`.

### Both nodes

| Function | ESP32 pin | Notes |
|---|---|---|
| L298N IN1 | 26 | direction |
| L298N IN2 | 27 | direction |
| L298N ENA | 14 | PWM, 20 kHz, 10-bit |
| Encoder A | 34 | input-only, interrupt on RISING |
| Encoder B | 35 | input-only, read for direction |
| Status LED | 2 | on-board LED on most dev boards |

### Follow node only

| Function | ESP32 pin | Notes |
|---|---|---|
| HC-SR04 TRIG | 5 | direct |
| HC-SR04 ECHO | 18 | **through a 1k/2k divider** |

---

## 3. Pins to avoid

| Pin | Problem |
|---|---|
| 6–11 | wired to the internal SPI flash — using them bricks the boot |
| 0, 2, 12, 15 | strapping pins; a pulled level at reset changes boot mode |
| 34–39 | input-only, no pull-ups, cannot drive anything |
| 1, 3 | UART0 — the serial monitor and the telemetry stream |

GPIO 2 is used for the status LED, which is fine (it is pulled the harmless way
on a dev board), but if upload starts failing intermittently, that is the first
thing to disconnect.

---

## 4. PWM choice

20 kHz, 10-bit (0–1023).

- **Above 20 kHz** is outside the audible range, so the motor does not whine.
- **Not much above**, because the L298N is a bipolar part with slow switching;
  push it to 100 kHz and the switching losses dominate and the driver gets hot.
- 10-bit at 20 kHz is comfortably within the ESP32 LEDC timer's resolution/
  frequency trade-off.

The L298N drops roughly 2 V across its output transistors. On a 12 V supply the
motor sees about 10 V. Budget for that when picking a supply — and if you upgrade
to a DRV8871 or BTS7960 (MOSFET, ~0.1 V drop), expect the motor to get noticeably
faster and the identified gain `K` to change. **Re-run system identification after
any change to the power path.**

---

## 5. Braking

`vio_set_duty()` with `u < 0` uses the L298N **short-brake** (both inputs HIGH),
not reverse.

A scale car reversing mid-stop is not a behaviour any real ACC has, and letting
the controller command reverse would let it cheat the gap in a way that would not
transfer to a real vehicle. Short-brake shorts the motor terminals so the back-EMF
dissipates through the windings — the real analogue of regenerative braking.

---

## 6. Bring-up order

Do not skip steps. Each one isolates a different failure mode.

**1. Boards alone, no motors.** Flash `firmware/tools/mac_address` to both.
Confirm both print a MAC and a channel. This proves the toolchain, the USB
cables and the boards.

**2. V2V link, still no motors.** Flash `lead_node` and `follow_node` with
`SIM_PLANT 1`. Both serial monitors should stream telemetry; the follower's mode
should reach `PREDICT` and `EMERG` during the lead's scenario, and `pkt_lost`
should stay near zero. **This is the Review 1 deliverable, and it needs no
hardware beyond the two boards.**

Capture proof:
```bash
python python/scripts/log_serial.py --port COM5 --out data/review1_link.csv --duration 60
```

**3. Motor + driver, no encoder.** Power the L298N from its own supply, grounds
tied. Write a trivial sketch that ramps duty 0 → 1 and confirm the wheel turns
smoothly and in the right direction. Swap IN1/IN2 if it runs backwards.

**4. Encoder.** Confirm the count increments when the wheel turns forwards and
decrements backwards. If it counts in only one direction, A and B are
swapped or B is not connected.

**5. Step response.** Put the car on blocks (better than a long run — no wheel
slip, no wall). Flash `firmware/tools/step_response`:
```bash
python python/scripts/log_serial.py --port COM5 --out data/step.csv --duration 45
python python/scripts/sysid_fit.py data/step.csv --plot
```

**6. Ultrasonic.** With the divider fitted, point it at a wall at known distances
and check the reading. Expect ±1–2 cm and occasional dropouts — that is normal
and is exactly what the median filter is for.

**7. Closed loop, `SIM_PLANT 0`.** Paste the identified parameters into
`config.h`, run `design_control.py`, verify in `sim_only.py`, *then* flash.

---

## 7. Bill of materials

### Essential

| Item | Qty | Note |
|---|---|---|
| ESP32 Dev Board (WROOM-32) | 2 | **you have these — everything in `SIM_PLANT 1` works today** |
| DC geared motor with encoder, 12 V | 2 | the encoder is the part that matters |
| L298N motor driver module | 2 | |
| HC-SR04 ultrasonic sensor | 1 | follower only |
| Wheels + small chassis | 2 | |
| 12 V battery / supply | 2 | separate from USB |
| Breadboard or perfboard | 2 | |
| Bulk capacitor 470–1000 µF | 2 | across the motor supply — tames the switching transient |
| Resistors 1 kΩ + 2 kΩ | 1 set | the ECHO divider — **do not skip** |
| Jumper wires, USB cables | — | |

Roughly ₹2,200–3,300 for the essentials.

### Worth considering

| Item | Why |
|---|---|
| VL53L0X ToF sensor | 3.3 V native, no divider, better resolution, no acoustic cross-talk between two cars |
| DRV8871 / BTS7960 | MOSFET driver, ~0.1 V drop vs the L298N's ~2 V — more usable speed range and a smaller dead-zone |
| INA219 / ACS712 | current sensing → an inner torque loop, a genuine third cascade level |
| SD card module | on-board logging, so runs do not depend on a laptop tether |
| OLED display | shows mode and gap on the car itself; good for the demo |

---

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `ESP-NOW init failed` | WiFi mode not `WIFI_STA`, or `esp_now_init()` called before `WiFi.mode()` |
| No packets received | channel mismatch — `V2V_WIFI_CHANNEL` must match in **both** sketches |
| `pkt_bad` climbing | struct layout mismatch — the two boards were built with different ESP32 core versions. Rebuild both. |
| Compile error on the receive callback | core 2.x vs 3.x signature. `v2v_protocol.h` has the shim — make sure the sketch folder copy is current (`python tools/sync_common.py`) |
| `ledcSetup` undefined | ESP32 core 3.x renamed the API; `vehicle_io.h` guards both, re-sync the headers |
| ESP32 resets when the motor starts | motor powered from the 5 V pin, or no bulk capacitor, or no common ground |
| Range reads 0 or max constantly | ECHO divider wrong, or TRIG/ECHO swapped |
| Encoder counts one direction only | A/B swapped, or B not connected |
| Speed reads zero but the wheel turns | `ENC_PPR` / `ENC_GEAR_RATIO` wrong, or the interrupt is on a pin that cannot interrupt |
| Car oscillates around the gap | `K_GAP` too high relative to the inner bandwidth — check the separation ratio in `design_control.py` |
| Brakes for no reason | closing-rate noise. Confirm the median filter is active and `CLOSING_LPF_FC_HZ` is not set too high |
