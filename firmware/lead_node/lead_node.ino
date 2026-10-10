// =====================================================================
//  lead_node.ino  --  LEAD VEHICLE / DRIVER'S CONSOLE
//  V2V-Assisted Predictive Adaptive Cruise Control  /  BECE302L
//
//  WHY THIS NODE HAS NO MOTOR
//  --------------------------
//  The lead vehicle's only job in this system is to TELL the follower what
//  it is doing. Nothing in the follower's control law depends on the lead
//  having real wheels. So this node runs its vehicle physics in SIM_PLANT
//  and is built instead as a driver's console:
//
//      potentiometer  -> accelerator
//      brake button   -> brake pedal
//      two buttons    -> select and run a demo scenario
//      OLED           -> the lead's dashboard
//
//  That halves the drivetrain hardware for the demo while leaving every
//  part of the thing being demonstrated -- the radio link, the follower's
//  closed-loop control, the sensing, the state machine -- completely real.
//  It is the same reasoning a hardware-in-the-loop bench uses: you do not
//  build a second vehicle in order to test a follower.
//
//  Responsibilities
//   1. Work out the commanded lead speed, from the scenario script or from
//      the throttle pot in manual mode.
//   2. Run a PID speed loop against its (simulated) plant, so the speed it
//      broadcasts has realistic dynamics rather than being a step.
//   3. Estimate its own acceleration and decide when it is BRAKING.
//   4. Broadcast {speed, accel, flags, scenario, seq} over ESP-NOW at 20 Hz
//      -- unless the active scenario has deliberately silenced the radio.
//   5. Drive the console OLED and stream telemetry to the laptop.
//
//  BROADCAST, NOT UNICAST
//  We send to FF:FF:FF:FF:FF:FF. Practical: no MAC addresses to look up,
//  type in and keep in sync. Correct: real V2V (DSRC / C-V2X) Basic Safety
//  Messages ARE broadcast, because a braking announcement is addressed to
//  every vehicle in range, not to one negotiated peer.
// =====================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "v2v_protocol.h"
#include "filters.h"
#include "pid.h"
#include "vehicle_io.h"
#include "scenario.h"
#include "hmi.h"

// ---------------------------------------------------------------------
//  Link configuration
// ---------------------------------------------------------------------
#define V2V_USE_UNICAST   0
#define V2V_WIFI_CHANNEL  1      // BOTH NODES MUST MATCH

static uint8_t peerMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#if V2V_USE_UNICAST
// If you switch to unicast, put the FOLLOWER's MAC here
// (run firmware/tools/mac_address to read it off the board).
static uint8_t unicastMac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif

// ---------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------
static PID            speedPid;
static Differentiator accelEst;
static LeadLag        leadLag;

static Scenario       scn;
static Throttle       throttle;
static Button         btnBrake, btnScnNext, btnScnRun;

static uint32_t seqCounter  = 0;
static uint32_t txOk        = 0;
static uint32_t txFail      = 0;
static uint32_t txSuppressed = 0;   // packets NOT sent during radio silence

static uint32_t lastCtrlUs  = 0;
static uint32_t lastTxMs    = 0;
static uint32_t lastTelemMs = 0;
static uint32_t lastHmiMs   = 0;

static float    vTarget     = 0.0f;
static bool     flagBraking = false;

// ---------------------------------------------------------------------
//  ESP-NOW send callback -- lets us report a real TX success rate
// ---------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
static void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) txOk++; else txFail++;
}

// ---------------------------------------------------------------------
static bool v2vBegin(void) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(V2V_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(onSent);

#if V2V_USE_UNICAST
  memcpy(peerMac, unicastMac, 6);
#endif

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = V2V_WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  return true;
}

static void v2vBroadcast(float speed, float accel, bool braking, bool hazard) {
  // RADIO SILENCE. This is how SCN_COMM_LOSS works: the lead carries on
  // driving perfectly normally but stops announcing itself. Nothing is
  // wrong with the lead -- the follower has simply lost its feedforward
  // channel, and must notice and widen its time gap on its own.
  if (!scn.tx_enabled) { txSuppressed++; return; }

  V2VPacket p = {};
  p.seq        = ++seqCounter;
  p.t_ms       = millis();
  p.speed_mps  = speed;
  p.accel_mps2 = accel;
  p.scenario   = (uint8_t)scn.id;
  p.flags      = 0;
  if (braking)                  p.flags |= V2V_FLAG_BRAKING;
  if (hazard)                   p.flags |= V2V_FLAG_HAZARD;
  if (speed > V_STANDSTILL_MPS) p.flags |= V2V_FLAG_MOVING;
#if SIM_PLANT
  p.flags |= V2V_FLAG_SIM;
#endif
  v2v_seal(&p, V2V_NODE_LEAD);
  esp_now_send(peerMac, (const uint8_t *)&p, V2V_PACKET_SIZE);
}

// ---------------------------------------------------------------------
//  Serial commands, so scenarios can also be driven from the dashboard.
//  The physical buttons are the showpiece; this is the backup path, and it
//  means a dead button does not cost you a scenario in the review.
//    0..4  select that scenario        r  run / restart
//    n     next scenario               s  stop
// ---------------------------------------------------------------------
static void pollSerial(void) {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c >= '0' && c < ('0' + SCN_COUNT)) {
      scn_select(&scn, (ScenarioId)(c - '0'));
      Serial.printf("# scenario -> %s\n", scn_name(&scn));
    } else if (c == 'n') {
      scn_next(&scn);
      Serial.printf("# scenario -> %s\n", scn_name(&scn));
    } else if (c == 'r') {
      scn_start(&scn);
      Serial.printf("# run %s\n", scn_name(&scn));
    } else if (c == 's') {
      scn_select(&scn, scn.id);
      Serial.println("# stop");
    }
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println("# ==============================================");
  Serial.println("# V2V Predictive ACC  --  LEAD NODE / CONSOLE");
  Serial.printf ("# build: SIM_PLANT=%d  ctrl=%d Hz  tx=%d Hz\n",
                 SIM_PLANT, CTRL_HZ, V2V_TX_HZ);
  Serial.printf ("# my MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("# keys: 0-4 select scenario, n next, r run, s stop");
  Serial.println("# ==============================================");

  vio_begin();
  pid_init(&speedPid, PID_KP, PID_KI, PID_KD, PID_N_DERIV,
           PID_U_MIN, PID_U_MAX, CTRL_DT);
  diff_init(&accelEst, 3.0f, CTRL_DT);
  ll_init(&leadLag, LL_B0, LL_B1, LL_A1);

  scn_init(&scn);
  thr_init(&throttle);
  btn_init(&btnBrake,   PIN_LEAD_BRAKE);
  btn_init(&btnScnNext, PIN_LEAD_SCN_NEXT);
  btn_init(&btnScnRun,  PIN_LEAD_SCN_RUN);

  hmi_begin("LEAD CONSOLE");

  if (!v2vBegin()) {
    Serial.println("# FATAL: ESP-NOW init failed");
    while (true) { vio_status_led(true); delay(100); vio_status_led(false); delay(100); }
  }
  Serial.printf("# ESP-NOW up on channel %d, peer %s\n",
                V2V_WIFI_CHANNEL,
                (peerMac[0] == 0xFF) ? "BROADCAST" : "unicast");
  Serial.println("# TELEM_HDR,t_ms,v,v_tgt,accel,duty,braking,hazard,"
                 "scn,running,throttle,tx_on,seq,tx_ok,tx_fail,tx_suppressed");

  lastCtrlUs = micros();
}

// ---------------------------------------------------------------------
void loop() {
  pollSerial();

  const uint32_t nowUs = micros();

  // ===== fixed-rate control task =====================================
  if ((uint32_t)(nowUs - lastCtrlUs) >= (uint32_t)(CTRL_DT * 1e6f)) {
    const float dt = (float)(nowUs - lastCtrlUs) * 1e-6f;
    lastCtrlUs = nowUs;

    // --- 1. console inputs
    const float thr = thr_read(&throttle);
    if (btn_pressed(&btnScnNext)) {
      scn_next(&scn);
      Serial.printf("# scenario -> %s\n", scn_name(&scn));
    }
    if (btn_pressed(&btnScnRun)) {
      scn_start(&scn);
      Serial.printf("# run %s\n", scn_name(&scn));
    }
    btn_pressed(&btnBrake);                 // update the debounce state
    const bool brakePedal = btn_held(&btnBrake);

    // --- 2. what should the lead be doing?
    scn_update(&scn, thr, brakePedal);

    // Rate-limit the setpoint. A declared emergency gets the full braking
    // limit; everything else uses the comfort limit.
    const float limit = scn.hazard ? A_BRAKE_MPS2 : A_MAX_MPS2;
    vTarget = rate_limit(scn.cmd_speed, vTarget, limit, dt);

    // --- 3. measure
    vio_update(dt);
    const float v = vio_speed();

    // --- 4. inner speed loop
    float u = pid_step(&speedPid, vTarget, v);
#if ENABLE_LEADLAG
    u = clampf(ll_step(&leadLag, u), PID_U_MIN, PID_U_MAX);
#endif
    vio_set_duty(u);

    // --- 5. own acceleration, and "am I braking?"
    const float accel = diff_step(&accelEst, v);
    //  Braking is a DECELERATION statement, not a "going slow" statement.
    //  (The original draft had `braking = speed < 3.0`, which flags a
    //  slow-but-steady vehicle as braking and misses a fast vehicle that
    //  has just stamped on the pedal -- exactly backwards for the one
    //  thing the follower needs to know.)
    flagBraking = (accel < -0.08f) || (scn.cmd_speed < v - 0.02f)
                  || scn.hazard || brakePedal;

    vio_status_led(flagBraking);
  }

  const uint32_t nowMs = millis();

  // ===== V2V broadcast task ==========================================
  if (nowMs - lastTxMs >= (uint32_t)(1000 / V2V_TX_HZ)) {
    lastTxMs = nowMs;
    v2vBroadcast(vio_speed(), accelEst.lp.y, flagBraking, scn.hazard);
  }

  // ===== HMI task ====================================================
  if (nowMs - lastHmiMs >= (uint32_t)(1000 / HMI_UPDATE_HZ)) {
    lastHmiMs = nowMs;
    hmi_draw_lead(scn_name(&scn), scn.running, scn.elapsed_s,
                  throttle.value, vio_speed(), scn.cmd_speed,
                  flagBraking, scn.tx_enabled);
    hmi_set_brake_lights(flagBraking);
  }

  // ===== telemetry task ==============================================
  if (nowMs - lastTelemMs >= (uint32_t)(1000 / TELEM_HZ)) {
    lastTelemMs = nowMs;
    Serial.printf("L,%lu,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%.3f,%d,"
                  "%lu,%lu,%lu,%lu\n",
                  (unsigned long)nowMs,
                  vio_speed(), vTarget, accelEst.lp.y, vio_last_duty(),
                  flagBraking ? 1 : 0, scn.hazard ? 1 : 0,
                  (int)scn.id, scn.running ? 1 : 0,
                  throttle.value, scn.tx_enabled ? 1 : 0,
                  (unsigned long)seqCounter,
                  (unsigned long)txOk, (unsigned long)txFail,
                  (unsigned long)txSuppressed);
  }
}
