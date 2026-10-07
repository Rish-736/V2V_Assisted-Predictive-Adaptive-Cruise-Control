// =====================================================================
//  lead_node.ino  --  LEAD VEHICLE
//  V2V-Assisted Predictive Adaptive Cruise Control  /  BECE302L
//
//  Responsibilities
//   1. Drive its own speed to a scripted profile (the "scenario"), so
//      every demo run is byte-for-byte repeatable and the digital-twin
//      comparison is meaningful.
//   2. Estimate its own acceleration and decide when it is BRAKING.
//   3. Broadcast {speed, accel, flags, seq} over ESP-NOW at 20 Hz.
//   4. Stream telemetry to the laptop over serial.
//
//  WHY BROADCAST INSTEAD OF A UNICAST MAC PEER
//  -------------------------------------------
//  We send to FF:FF:FF:FF:FF:FF. Two reasons:
//   * Practical: no MAC addresses to look up, type in and keep in sync.
//     Flash both boards and they talk. One less thing to break at 2 a.m.
//     before a review.
//   * Correct: real V2V (DSRC / C-V2X) Basic Safety Messages ARE
//     broadcast. A braking announcement is addressed to every vehicle in
//     range, not to one negotiated peer. Our link mirrors the real thing.
//   Set V2V_USE_UNICAST to 1 if you want per-packet delivery callbacks
//   for a link-reliability experiment.
// =====================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "v2v_protocol.h"
#include "filters.h"
#include "pid.h"
#include "vehicle_io.h"

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

static uint32_t seqCounter   = 0;
static uint32_t txOk         = 0;
static uint32_t txFail       = 0;

static uint32_t lastCtrlUs   = 0;
static uint32_t lastTxMs     = 0;
static uint32_t lastTelemMs  = 0;
static uint32_t bootMs       = 0;

static float    vTarget      = 0.0f;
static float    vTargetRaw   = 0.0f;
static bool     flagBraking  = false;
static bool     flagHazard   = false;

// ---------------------------------------------------------------------
//  Scenario: the scripted speed profile the lead vehicle follows.
//  Returns the commanded speed [m/s] for a given time since boot.
//
//   0 ---3s--- stopped
//   3 --12s--- cruise 0.40
//  12 --17s--- gentle slowdown to 0.18   <- tests predictive following
//  17 --24s--- back to cruise
//  24 --32s--- EMERGENCY STOP to 0       <- the headline demo
//  32 --42s--- recover to cruise, then loop
// ---------------------------------------------------------------------
static float scenarioSpeed(float t, bool *hazard) {
  *hazard = false;
  if (SCENARIO_T_LOOP > 0.0f) t = fmodf(t, SCENARIO_T_LOOP);

  if (t < SCENARIO_T_START)       return 0.0f;
  if (t < SCENARIO_T_BRAKE1)      return SCENARIO_CRUISE_MPS;
  if (t < SCENARIO_T_RESUME)      return 0.18f;
  if (t < SCENARIO_T_BRAKE_HARD)  return SCENARIO_CRUISE_MPS;
  if (t < SCENARIO_T_RESTART)   { *hazard = true; return 0.0f; }
  return SCENARIO_CRUISE_MPS;
}

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
  V2VPacket p = {};
  p.seq        = ++seqCounter;
  p.t_ms       = millis();
  p.speed_mps  = speed;
  p.accel_mps2 = accel;
  p.flags      = 0;
  if (braking)                       p.flags |= V2V_FLAG_BRAKING;
  if (hazard)                        p.flags |= V2V_FLAG_HAZARD;
  if (speed > V_STANDSTILL_MPS)      p.flags |= V2V_FLAG_MOVING;
#if SIM_PLANT
  p.flags |= V2V_FLAG_SIM;
#endif
  v2v_seal(&p, V2V_NODE_LEAD);
  esp_now_send(peerMac, (const uint8_t *)&p, V2V_PACKET_SIZE);
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println("# ==============================================");
  Serial.println("# V2V Predictive ACC  --  LEAD NODE");
  Serial.printf ("# build: SIM_PLANT=%d  ctrl=%d Hz  tx=%d Hz\n",
                 SIM_PLANT, CTRL_HZ, V2V_TX_HZ);
  Serial.printf ("# my MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("# ==============================================");

  vio_begin();
  pid_init(&speedPid, PID_KP, PID_KI, PID_KD, PID_N_DERIV,
           PID_U_MIN, PID_U_MAX, CTRL_DT);
  diff_init(&accelEst, 3.0f, CTRL_DT);
  ll_init(&leadLag, LL_B0, LL_B1, LL_A1);

  if (!v2vBegin()) {
    Serial.println("# FATAL: ESP-NOW init failed");
    while (true) { vio_status_led(true); delay(100); vio_status_led(false); delay(100); }
  }
  Serial.printf("# ESP-NOW up on channel %d, peer %s\n",
                V2V_WIFI_CHANNEL,
                (peerMac[0] == 0xFF) ? "BROADCAST" : "unicast");
  Serial.println("# TELEM_HDR,t_ms,v,v_tgt,accel,duty,braking,hazard,seq,tx_ok,tx_fail");

  bootMs      = millis();
  lastCtrlUs  = micros();
}

// ---------------------------------------------------------------------
void loop() {
  const uint32_t nowUs = micros();

  // ===== fixed-rate control task =====================================
  if ((uint32_t)(nowUs - lastCtrlUs) >= (uint32_t)(CTRL_DT * 1e6f)) {
    const float dt = (float)(nowUs - lastCtrlUs) * 1e-6f;
    lastCtrlUs = nowUs;

    const float tSec = (float)(millis() - bootMs) * 1e-3f;

    // --- 1. where should we be going?
    bool hazard = false;
#if SCENARIO_ENABLED
    vTargetRaw = scenarioSpeed(tSec, &hazard);
#else
    vTargetRaw = SCENARIO_CRUISE_MPS;
#endif
    // Rate-limit the setpoint. An emergency stop is allowed to use the
    // full braking authority; normal changes use the comfort limit.
    const float limit = hazard ? A_BRAKE_MPS2 : A_MAX_MPS2;
    vTarget = rate_limit(vTargetRaw, vTarget, limit, dt);
    flagHazard = hazard;

    // --- 2. measure
    vio_update(dt);
    const float v = vio_speed();

    // --- 3. inner speed loop
    float u = pid_step(&speedPid, vTarget, v);
#if ENABLE_LEADLAG
    u = clampf(ll_step(&leadLag, u), PID_U_MIN, PID_U_MAX);
#endif
    vio_set_duty(u);

    // --- 4. estimate our own acceleration and decide "am I braking?"
    const float accel = diff_step(&accelEst, v);
    //  Braking is a DECELERATION statement, not a "going slow" statement.
    //  (The handover draft had `braking = speed < 3.0`, which flags a
    //  slow-but-steady vehicle as braking and misses a fast vehicle that
    //  has just stamped on the pedal -- exactly backwards for the thing
    //  the follower needs to know.)
    flagBraking = (accel < -0.15f) || (vTargetRaw < v - 0.05f) || hazard;

    vio_status_led(flagBraking);
  }

  // ===== V2V broadcast task ==========================================
  const uint32_t nowMs = millis();
  if (nowMs - lastTxMs >= (uint32_t)(1000 / V2V_TX_HZ)) {
    lastTxMs = nowMs;
    v2vBroadcast(vio_speed(), accelEst.lp.y, flagBraking, flagHazard);
  }

  // ===== telemetry task ==============================================
  if (nowMs - lastTelemMs >= (uint32_t)(1000 / TELEM_HZ)) {
    lastTelemMs = nowMs;
    Serial.printf("L,%lu,%.4f,%.4f,%.4f,%.4f,%d,%d,%lu,%lu,%lu\n",
                  (unsigned long)(nowMs - bootMs),
                  vio_speed(), vTarget, accelEst.lp.y, vio_last_duty(),
                  flagBraking ? 1 : 0, flagHazard ? 1 : 0,
                  (unsigned long)seqCounter,
                  (unsigned long)txOk, (unsigned long)txFail);
  }
}
