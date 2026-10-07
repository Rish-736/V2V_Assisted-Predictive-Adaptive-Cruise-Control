# Project Handover — V2V-Assisted Predictive Adaptive Cruise Control
**Course:** BECE302L (Control Systems), Fall Semester 2026–27, VIT Vellore
**Purpose of this file:** full context dump to continue development in Claude Code — architecture, decisions made, code already written, and what's left to build.

---

## 1. Project identity

- **Title:** V2V-Assisted Predictive Adaptive Cruise Control (with Predictive Time-Gap Braking and Digital-Twin Validation)
- **Team:** Rishit Sinha (24BEC0529), Rupsa Mittra (24BEC0176), Rishika Kapoor (24BEC0455), 4th member TBD
- **Faculty:** Dr Gopinath
- **Format:** 3 graded reviews — Review 1 (survey + partial output), Review 2 (progress), Review 3 (full working model). Individual contribution is graded separately.
- **SDG alignment:** Primary SDG 3 (Target 3.6 — reduce road traffic deaths via earlier, predictive braking); secondary SDG 9 (Industry & Innovation — low-cost embedded/V2V tech).

---

## 2. The core idea, in one paragraph

Most ACC systems are purely sensor-based and reactive — they only respond after detecting a change in the lead vehicle. This project builds a two-node embedded system where the following vehicle combines its own ultrasonic sensor reading with a direct wireless (V2V) broadcast from the lead vehicle, enabling it to react the instant the lead vehicle changes speed/brakes — not just when its own sensor notices. It also uses a constant **time-gap** policy (not fixed distance) and brakes **predictively** based on closing rate rather than waiting for the gap to become critically small. A software **digital twin** runs in parallel on a laptop to validate real hardware behavior against the theoretical control model.

---

## 3. System architecture

**Two independent ESP32 nodes:**

1. **Lead Vehicle Node** — its own DC motor + encoder, drives around (initially with dummy/random speed values for testing), continuously broadcasts its speed and a `braking` boolean over ESP-NOW.
2. **Following Vehicle Node** — has its own DC motor + encoder + ultrasonic sensor (HC-SR04). Receives the lead node's wireless broadcast AND reads its own distance sensor. Combines both to run the control loop and decide speed/braking.

**Control structure — cascaded loop:**
- **Outer loop:** converts gap error (desired time-gap vs. estimated actual gap) into a target speed setpoint.
- **Inner loop:** PID + lead-lag compensator drives the motor to track that target speed.
- **Predictive braking layer:** computes closing rate from consecutive ultrasonic readings (`(prev_distance - current_distance) / dt`); ORs this against the lead node's wireless `braking` flag — reacts to whichever signal indicates danger first. (Optional upgrade: weighted-average/Kalman-style fusion instead of hard OR-logic.)
- **Telemetry:** following node streams distance, speed, target, and mode data over serial to a laptop, which runs the **digital twin** (a Python simulation using the same motor transfer function, fed the same commands as hardware) and a **live dashboard** (real vs. simulated, plotted side by side).

**Full signal-path block diagram** (already generated as an image, `V2V_ACC_block_diagram.png`): reference input (desired time-gap) → Σ (gap error) → Outer Controller (time-gap policy) → Σ (speed error, also receives V2V predictive-braking feedforward) → Inner Controller (PID + lead-lag) → Plant (motor driver + DC motor) → actual speed → integrates to actual gap. Two feedback paths: inner loop feeds back measured speed (wheel encoder); outer loop feeds back estimated gap (sensor fusion: ultrasonic + V2V). V2V path is drawn as a separate dashed red feedforward line into the inner summing junction, representing the "early brake correction" that makes the system predictive rather than reactive.

---

## 4. Digital twin — design details (important, was discussed at length)

- **Runs on a laptop**, NOT on the ESP32 (avoids competing with the real-time control loop for CPU cycles; keeps it an independent check; needs Python/graphing tools the ESP32 doesn't have).
- **Built via system identification, done ONCE, offline:** apply a step voltage to the real motor, record speed-over-time via the encoder, fit a transfer function (first or second order) from that curve. This is the only point real hardware data feeds into the model.
- **During live operation:** the twin receives the *same commands* as the real hardware (e.g. "target speed = 5"), not the real hardware's output. It independently calculates predicted behavior from the fitted transfer function. Real and predicted streams are plotted side by side, same time axis, for comparison.
- **Critical distinction:** the digital twin does NOT make real-time braking decisions. Real-time control uses live sensor/V2V data only, with built-in safety margins (conservative thresholds, not razor-exact model-predicted cutoffs) precisely because the model is known to be imperfect.
- **Validation approach (two stages, to avoid circularity):**
  1. Twin vs. closed-form theoretical formulas (standard 2nd-order system formulas for overshoot/rise time/settling time) — validates the *simulation code* is implemented correctly.
  2. Twin vs. fresh real hardware runs (different from the runs used to fit the model) — validates the *model* actually captures real-world behavior (friction, delay, etc.).

---

## 5. Code already written

### ESP-NOW — Lead Vehicle Node (sender)
```cpp
#include <esp_now.h>
#include <WiFi.h>

uint8_t receiverMAC[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX}; // Following node's MAC — fill in

typedef struct {
  float speed;
  bool braking;
} Message;

Message data;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  data.speed = random(0, 100) / 10.0;   // dummy value — replace with real encoder reading
  data.braking = (data.speed < 3.0);
  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
  Serial.printf("Sent -> speed: %.1f, braking: %d\n", data.speed, data.braking);
  delay(200);
}
```

### ESP-NOW — Following Vehicle Node (receiver)
```cpp
#include <esp_now.h>
#include <WiFi.h>

typedef struct {
  float speed;
  bool braking;
} Message;

Message data;

void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));
  Serial.printf("Received <- speed: %.1f, braking: %d\n", data.speed, data.braking);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_recv_cb(onReceive);
}

void loop() {}
```

### MAC address finder (run on both boards first)
```cpp
#include "WiFi.h"
void setup() {
  Serial.begin(115200);
  Serial.println(WiFi.macAddress());
}
void loop() {}
```

**Status:** this is the Review 1 "partial progress" claim — link should be confirmed working (serial monitor showing continuous received messages) before the review, with a screenshot/recording as proof.

---

## 6. Hardware BOM (full list)

**Following Vehicle Node:** ESP32 Dev Board (WROOM-32), DC geared motor w/ encoder (12V), L298N motor driver, HC-SR04 ultrasonic sensor, wheels, small chassis/base.

**Lead Vehicle Node:** ESP32 Dev Board, DC geared motor w/ encoder, L298N motor driver, wheels, small chassis/base.

**Shared:** 12V power supply/battery ×2, battery holders, jumper wires, breadboard/perfboard ×2, bulk capacitor (470–1000µF) ×2, USB cables ×2.

**Optional upgrades (not essential yet):** better driver (DRV8871/BTS7960), current-sense module (INA219/ACS712) for an inner torque loop, VL53L0X ToF sensor as an alternative to ultrasonic, OLED display, SD card logging module.

**Estimated cost:** ₹2,200–3,300 essentials only; ₹3,500–5,000 with upgrades.

**Software/tools:** Arduino IDE or PlatformIO (firmware), Python 3 + `pyserial` + `matplotlib` (dashboard/digital twin), MATLAB/Simulink Control System Toolbox (VIT campus license) or `python-control` as free fallback.

---

## 7. Review timeline / roadmap

**Review 1 (now):**
- Literature survey (5 papers, see below) + problem formulation
- Processor rationale (ESP32 — built-in WiFi for V2V, no extra comms hardware, adequate timers for PWM/encoder)
- Wireless V2V link (ESP-NOW) — bring-up/proof this is working
- System block diagram finalized
- Team roles & task split decided

**Review 2:**
- Measure real motor, build/fit its transfer function (system identification)
- PID speed control + time-gap outer loop implementation
- Tune via root locus / Bode plots; design lead-lag compensator
- Combine wireless + ultrasonic sensor data (fusion logic)
- Bench-test the speed loop alone

**Review 3:**
- Predictive (closing-rate) braking logic, fully integrated
- Digital twin + live dashboard built and running
- Handle sensor/link failures safely (fallback to sensor-only mode if V2V drops)
- State-space model + stability check (stretch)
- Full system integration test, final testing, final report

---

## 8. Literature review (5 papers, all verified on IEEE Xplore)

1. H. Rezaee, T. Parisini, and M. M. Polycarpou, "Leaderless cooperative adaptive cruise control based on the constant time-gap spacing policy," *IEEE Trans. Autom. Control*, vol. 69, no. 1, pp. 659–666, Jan. 2024. https://ieeexplore.ieee.org/document/10202202
2. M. Shen, R. A. Dollar, T. G. Molnár, C. R. He, A. Vahidi, and G. Orosz, "Energy-efficient reactive and predictive connected cruise control," *IEEE Trans. Intell. Veh.*, vol. 9, no. 1, pp. 944–957, Jan. 2024. https://ieeexplore.ieee.org/document/10139848
3. A. Alipour-Fanid, M. Dabaghchian, and K. Zeng, "Impact of jamming attacks on vehicular cooperative adaptive cruise control systems," *IEEE Trans. Veh. Technol.*, vol. 69, no. 11, pp. 12679–12693, Nov. 2020. https://ieeexplore.ieee.org/document/9222336
4. J. Dong et al., "Mixed cloud control testbed: Validating vehicle-road-cloud integration via mixed digital twin," *IEEE Trans. Intell. Veh.*, vol. 8, no. 4, pp. 2723–2736, 2023. https://ieeexplore.ieee.org/document/10040234
5. G. Gunter, C. Janssen, W. Barbour, R. E. Stern, and D. B. Work, "Model-based string stability of adaptive cruise control systems using field data," *IEEE Trans. Intell. Veh.*, vol. 5, no. 1, pp. 90–99, Mar. 2020. https://ieeexplore.ieee.org/document/8910461

**Identified gap (our novelty):** no existing work combines predictive V2V-based braking + low-cost embedded two-node hardware + live digital-twin validation together — each paper covers at most one or two of these pieces, in simulation or on full-scale/expensive vehicle testbeds.

---

## 9. Deliverables already produced (in this chat, not yet in Claude Code)

- **Review 1 PPT** — 9 slides (title, outline, introduction, literature review table, problem formulation, SDG relevance, system architecture, work progress/timeline, references), navy/ice-blue theme, built with pptxgenjs + react-icons. Simplified language, larger fonts per request. Motor+braking-actuator box removed from architecture/timeline slides since that scope was uncertain.
- **Team explainer PDF** — 3-page plain-language document (what we're building, why it's not basic, how it works, what Review 1 shows, what's left, hardware list, why it's worth it), built with reportlab.
- **Closed-loop control block diagram** — PNG image (see Section 3 description), built as SVG → rendered via sharp.
- **Updated timeline slide** — expanded from 4 to 6 bullets per review phase.
- **Spoken talk-track** for presenting all 9 slides.
- **Large Q&A prep set** — novelty questions, real-world implementation questions, control theory fundamentals (time-gap policy, transfer functions, PID vs compensator, root locus, gain/phase margin, cascaded loops, closing rate, digital twin justification), hardware choice justifications, SDG defense.

*(Note: these files were generated in a prior chat session's sandbox and are not attached here — if needed in Claude Code, they'd need to be regenerated or re-uploaded.)*

---

## 10. Immediate next technical tasks (where Claude Code should pick up)

1. Fill in real MAC addresses in the ESP-NOW sketches and confirm the wireless link works end-to-end (if not already confirmed).
2. Replace the lead node's dummy `random()` speed value with a real encoder reading.
3. Wire up and read the HC-SR04 ultrasonic sensor on the following node; print readings alongside received V2V data.
4. Run a step-response test on the real motor (apply fixed voltage, log encoder speed over time) to begin system identification for the transfer function.
5. Implement the PID speed loop (inner loop) on the following node.
6. Implement the time-gap → target-speed conversion (outer loop).
7. Implement the closing-rate calculation and OR-fusion predictive braking logic.
8. Build the Python digital twin script (motor transfer function simulation + serial read of real telemetry + matplotlib live comparison plot).
9. Tune gains using root locus/Bode analysis (MATLAB or `python-control`), then validate on hardware.
10. Design and add the lead-lag compensator once basic PID is working.

---

## 11. Rishit's working context (for continuity, not project-specific but relevant)

- Third-year ECE student at VIT Vellore, strong hardware/embedded background (Team Ardra drone team, RoboVITics, IEEE CAS).
- Prefers plain-language explanations, conceptual grounding before formulas, hands-on learning by doing.
- Careful about not overclaiming software contributions when his role is primarily hardware — on this project he's leaning toward owning the control logic/software side specifically (as discussed), with hardware/wiring split to teammates.
- Placement targets (Bosch, Continental, Daimler, etc.) are why the automotive/ADAS framing of this project was chosen deliberately.
