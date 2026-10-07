// =====================================================================
//  follow_node.ino  --  FOLLOWING VEHICLE
//  V2V-Assisted Predictive Adaptive Cruise Control  /  BECE302L
//
//  This is the whole control system. Structure:
//
//    +------------------------------------------------------------+
//    |  SUPERVISOR (state machine)                                 |
//    |    STANDBY / CRUISE / FOLLOW / PREDICT_BRAKE / EMERGENCY    |
//    |    + link health:  NOMINAL / DEGRADED / LOST                |
//    +------------------------------------------------------------+
//              |                                       ^
//              v                                       |
//    +--------------------+   v_tgt   +-------------+  | v (encoder)
//    | OUTER: time-gap    |---------->| INNER: PID  |--+
//    | d_des = d0 + Th*v  |           | + lead-lag  |
//    | v_tgt = v_lead     |           +------+------+
//    |   + Kgap*(d-d_des) |                  | u
//    +---------+----------+                  v
//              ^                        [ L298N + motor ]
//              | d (ultrasonic, filtered)
//              | v_lead, braking  <--- V2V (ESP-NOW)
//
//  THE PREDICTIVE PART
//  -------------------
//  A conventional ACC closes the outer loop on range alone. It cannot
//  know the lead is braking until the gap has ALREADY started shrinking
//  -- it has to wait for the integral of the speed difference to become
//  observable above sensor noise. That wait is the collision.
//
//  Here, three danger signals are evaluated every tick and OR-ed:
//    (a) V2V braking/hazard flag   -- fires the instant the lead brakes,
//                                     before the gap changes at all
//    (b) time-to-contact           -- gap / closing-rate, anticipatory
//    (c) absolute gap floor        -- last-resort backstop
//  Whichever trips first wins. (a) is the novel one; (b) and (c) are the
//  safety net that keeps the system sane if the radio is jammed or lost.
// =====================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "v2v_protocol.h"
#include "filters.h"
#include "pid.h"
#include "vehicle_io.h"

#define V2V_WIFI_CHANNEL  1      // MUST MATCH THE LEAD NODE

// ---------------------------------------------------------------------
//  Supervisory states
// ---------------------------------------------------------------------
enum Mode {
  MODE_STANDBY = 0,   // stopped, waiting for a lead vehicle to appear
  MODE_CRUISE  = 1,   // no target in range -> hold set speed
  MODE_FOLLOW  = 2,   // tracking the time-gap
  MODE_PREDICT = 3,   // predictive braking engaged
  MODE_EMERG   = 4,   // emergency stop
  MODE_FAULT   = 5    // range sensor dead AND no V2V -> fail safe, stop
};
static const char *MODE_NAME[] = {"STANDBY","CRUISE","FOLLOW","PREDICT","EMERG","FAULT"};

enum LinkState { LINK_NOMINAL = 0, LINK_DEGRADED = 1, LINK_LOST = 2 };
static const char *LINK_NAME[] = {"OK","DEGRADED","LOST"};

// ---------------------------------------------------------------------
//  Received V2V data. Written in the ESP-NOW ISR context -> volatile,
//  and copied out under a critical section before use.
// ---------------------------------------------------------------------
typedef struct {
  volatile float    speed;
  volatile float    accel;
  volatile uint8_t  flags;
  volatile uint32_t seq;
  volatile uint32_t rxMs;
  volatile bool     fresh;
} V2VRx;
static V2VRx v2v = {0, 0, 0, 0, 0, false};

static portMUX_TYPE v2vMux = portMUX_INITIALIZER_UNLOCKED;

// link statistics
static uint32_t pktGood    = 0;
static uint32_t pktBad     = 0;
static uint32_t pktLost    = 0;   // inferred from sequence-number gaps
static uint32_t lastSeq    = 0;
static uint16_t goodStreak = 0;

// ---------------------------------------------------------------------
//  Controller state
// ---------------------------------------------------------------------
static PID            speedPid;
static LeadLag        leadLag;
static Median3        rangeMed;
static LowPass        rangeLp;
static Differentiator rangeDiff;     // -> closing rate

static Mode      mode       = MODE_STANDBY;
static LinkState link       = LINK_LOST;

static float dMeas      = 1.00f;   // filtered gap [m]
static float dDes       = 0.40f;   // desired gap  [m]
static float closingMps = 0.0f;    // positive = gap shrinking
static float ttc        = 99.0f;   // time to contact [s]
static float vLeadEst   = 0.0f;    // from V2V, or estimated if link lost
static float vTarget    = 0.0f;    // outer-loop output, rate limited
static float vTargetRaw = 0.0f;
static float vSetCruise = 0.35f;   // cruise set speed when no target
static float tGapActive = T_GAP_S;

static uint32_t lastCtrlUs  = 0;
static uint32_t lastTelemMs = 0;
static uint32_t bootMs      = 0;
static bool     brakeLatched = false;
static uint32_t modeEnteredMs = 0;

// =====================================================================
//  ESP-NOW receive callback
//  Keep it SHORT. Validate, copy, timestamp, return. All the control
//  work happens in the fixed-rate loop so timing stays deterministic.
// =====================================================================
static void onReceive(V2V_RECV_CB_ARGS) {
  (void)V2V_RECV_PEER_MAC;
  V2VPacket p;
  if (!v2v_validate(data, len, &p)) { pktBad++; return; }
  if (p.node_id != V2V_NODE_LEAD)   { return; }

  portENTER_CRITICAL_ISR(&v2vMux);
  // sequence gap -> packets we never saw
  if (lastSeq != 0 && p.seq > lastSeq + 1) pktLost += (p.seq - lastSeq - 1);
  lastSeq    = p.seq;
  v2v.speed  = p.speed_mps;
  v2v.accel  = p.accel_mps2;
  v2v.flags  = p.flags;
  v2v.seq    = p.seq;
  v2v.rxMs   = millis();
  v2v.fresh  = true;
  portEXIT_CRITICAL_ISR(&v2vMux);

  pktGood++;
  if (goodStreak < 65535) goodStreak++;
}

static bool v2vBegin(void) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(V2V_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onReceive);

  // Register the broadcast address as a peer so broadcast frames from
  // the lead node are accepted.
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, bcast, 6);
  peer.channel = V2V_WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;
  esp_now_add_peer(&peer);
  return true;
}

// =====================================================================
//  LINK HEALTH
//  Losing V2V does not break the controller -- it removes the
//  feedforward term and demotes us from predictive to reactive. The
//  correct response is not to fault but to BUY BACK the lost reaction
//  time by opening up the time gap.
// =====================================================================
static void updateLink(uint32_t nowMs) {
  uint32_t age;
  portENTER_CRITICAL(&v2vMux);
  age = nowMs - v2v.rxMs;
  const bool everRx = (v2v.seq != 0);
  portEXIT_CRITICAL(&v2vMux);

  if (!everRx) { link = LINK_LOST; goodStreak = 0; return; }

  if (age > LINK_TIMEOUT_MS * 3) {
    link = LINK_LOST;
    goodStreak = 0;
  } else if (age > LINK_TIMEOUT_MS) {
    if (link == LINK_NOMINAL) link = LINK_DEGRADED;
    goodStreak = 0;
  } else {
    // only re-trust the link after a run of consecutive good packets,
    // so we do not flap on a single lucky frame
    if (link != LINK_NOMINAL && goodStreak >= LINK_RECOVER_PKTS) link = LINK_NOMINAL;
  }

  tGapActive = (link == LINK_NOMINAL) ? T_GAP_S : T_GAP_DEGRADED_S;
}

// =====================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println("# ==============================================");
  Serial.println("# V2V Predictive ACC  --  FOLLOW NODE");
  Serial.printf ("# build: SIM_PLANT=%d  ctrl=%d Hz\n", SIM_PLANT, CTRL_HZ);
  Serial.printf ("# my MAC: %s\n", WiFi.macAddress().c_str());
  Serial.printf ("# time gap: %.2f s nominal / %.2f s degraded\n",
                 T_GAP_S, T_GAP_DEGRADED_S);
  Serial.println("# ==============================================");

  vio_begin();
#if SIM_PLANT
  vio_sim_set_gap(1.00f);          // start one metre behind
#endif

  pid_init(&speedPid, PID_KP, PID_KI, PID_KD, PID_N_DERIV,
           PID_U_MIN, PID_U_MAX, CTRL_DT);
  ll_init(&leadLag, LL_B0, LL_B1, LL_A1);
  med3_init(&rangeMed);
  lp_init(&rangeLp, ULTRA_LPF_FC_HZ, CTRL_DT);
  diff_init(&rangeDiff, CLOSING_LPF_FC_HZ, CTRL_DT);

  if (!v2vBegin()) {
    Serial.println("# FATAL: ESP-NOW init failed");
    while (true) { vio_status_led(true); delay(100); vio_status_led(false); delay(100); }
  }
  Serial.printf("# ESP-NOW up on channel %d (broadcast rx)\n", V2V_WIFI_CHANNEL);

  // Telemetry schema -- the Python side parses this header, so the two
  // never drift out of sync.
  Serial.println("# TELEM_HDR,t_ms,d,d_des,closing,ttc,v,v_tgt,v_lead,duty,"
                 "mode,link,brake,p,i,dterm,pkt_good,pkt_lost,pkt_bad");

  bootMs     = millis();
  lastCtrlUs = micros();
}

// =====================================================================
void loop() {
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastCtrlUs) < (uint32_t)(CTRL_DT * 1e6f)) return;

  const float dt = (float)(nowUs - lastCtrlUs) * 1e-6f;
  lastCtrlUs = nowUs;
  const uint32_t nowMs = millis();

  // -------------------------------------------------------------------
  // 1. LINK HEALTH + snapshot of the V2V data
  // -------------------------------------------------------------------
  updateLink(nowMs);

  portENTER_CRITICAL(&v2vMux);
  const float   leadSpeed = v2v.speed;
  const uint8_t leadFlags = v2v.flags;
  portEXIT_CRITICAL(&v2vMux);

  const bool linkUsable  = (link != LINK_LOST);
  const bool leadBraking = linkUsable && (leadFlags & V2V_FLAG_BRAKING);
  const bool leadHazard  = linkUsable && (leadFlags & V2V_FLAG_HAZARD);

  // -------------------------------------------------------------------
  // 2. SENSE
  // -------------------------------------------------------------------
  vio_update(dt);
  const float v = vio_speed();

#if SIM_PLANT
  // In sim the gap comes from integrating relative velocity. If the link
  // is down we have no lead speed, so the gap simply stops updating --
  // which is realistic: that IS what losing your only information source
  // looks like.
  vio_sim_integrate_gap(linkUsable ? leadSpeed : v, v, dt);
  dMeas = lp_step(&rangeLp, med3_step(&rangeMed, vio_range()));
#else
  dMeas = vio_range();            // already median+LPF filtered in vio_update
#endif

  const bool rangeOk = vio_range_ok();

  // closing rate: positive when the gap is SHRINKING
  closingMps = -diff_step(&rangeDiff, dMeas);

  // time-to-contact
  ttc = (closingMps > CLOSING_RATE_MIN_MPS) ? (dMeas / closingMps) : 99.0f;

  // Lead speed estimate. With V2V we KNOW it. Without, we can still
  // reconstruct it from our own speed and the closing rate -- noisier
  // and lagged by the filters, which is precisely the penalty the V2V
  // link exists to remove.
  vLeadEst = linkUsable ? leadSpeed : (v - closingMps);
  if (vLeadEst < 0.0f) vLeadEst = 0.0f;

  // -------------------------------------------------------------------
  // 3. PREDICTIVE BRAKING DECISION  (three OR-ed danger signals)
  // -------------------------------------------------------------------
  const bool dangerV2V   = leadBraking || leadHazard;
  const bool dangerTtc   = (ttc < TTC_BRAKE_S);
  const bool dangerFloor = rangeOk && (dMeas < D_BRAKE_FLOOR_M);

  const bool emergency = leadHazard
                      || (ttc < TTC_EMERGENCY_S)
                      || (rangeOk && dMeas < D_BRAKE_FLOOR_M * 0.6f);

  // Hysteresis on release, so we do not chatter in and out of braking
  // right at the threshold.
  bool danger = dangerV2V || dangerTtc || dangerFloor;
  if (brakeLatched && !danger) {
    const bool clearedTtc = (ttc > TTC_BRAKE_S * BRAKE_RELEASE_HYST);
    const bool clearedGap = (dMeas > D_BRAKE_FLOOR_M * BRAKE_RELEASE_HYST);
    if (!(clearedTtc && clearedGap)) danger = true;
  }
  brakeLatched = danger;

  // -------------------------------------------------------------------
  // 4. SUPERVISOR -- pick the mode
  // -------------------------------------------------------------------
  const Mode prevMode = mode;

  if (!rangeOk && !linkUsable) {
    mode = MODE_FAULT;                       // blind and deaf -> stop
  } else if (emergency) {
    mode = MODE_EMERG;
  } else if (danger) {
    mode = MODE_PREDICT;
  } else if (!rangeOk || dMeas >= ULTRA_MAX_M * 0.95f) {
    mode = MODE_CRUISE;                      // nothing ahead
  } else if (v < V_STANDSTILL_MPS && vLeadEst < V_STANDSTILL_MPS
             && dMeas <= D_STANDSTILL_M * 1.2f) {
    mode = MODE_STANDBY;                     // queued up behind a stopped car
  } else {
    mode = MODE_FOLLOW;
  }

  //  Mode dwell: hold a mode for at least MODE_MIN_DWELL_MS before
  //  dropping to a CALMER one. Escalation (to a higher/more dangerous
  //  mode number) is always immediate -- safety never waits on a timer.
  if (mode != prevMode) {
    const bool escalating = ((int)mode > (int)prevMode);
    const bool dwellOk = (nowMs - modeEnteredMs) >= MODE_MIN_DWELL_MS;
    if (!escalating && !dwellOk) {
      mode = prevMode;            // too soon to calm down; hold
    } else {
      modeEnteredMs = nowMs;
    }
  }

  // -------------------------------------------------------------------
  // 5. OUTER LOOP -- constant time-gap spacing policy
  //      d_des = d0 + Th * v_follow
  //      v_tgt = v_lead + Kgap * (d - d_des)
  //    The v_lead term is FEEDFORWARD. It is what lets the follower
  //    match a speed change immediately rather than integrating a gap
  //    error until it becomes large enough to act on.
  // -------------------------------------------------------------------
  dDes = D_STANDSTILL_M + tGapActive * v;

  switch (mode) {
    case MODE_FAULT:
    case MODE_EMERG:
      vTargetRaw = 0.0f;
      break;

    case MODE_PREDICT:
      // Command a stop, but let the rate limiter and the PID shape the
      // deceleration rather than slamming the duty to -1. If the gap is
      // already below target we also take the gap error into account so
      // the response scales with how bad the situation is.
      vTargetRaw = fminf(0.0f, vLeadEst + K_GAP * (dMeas - dDes));
      break;

    case MODE_STANDBY:
      vTargetRaw = 0.0f;
      break;

    case MODE_CRUISE:
      vTargetRaw = vSetCruise;
      break;

    case MODE_FOLLOW:
    default:
      vTargetRaw = vLeadEst + K_GAP * (dMeas - dDes);
      break;
  }
  vTargetRaw = clampf(vTargetRaw, 0.0f, V_MAX_MPS);

  // Rate-limit the setpoint to a physically achievable acceleration.
  const float aLimit = (mode == MODE_EMERG || mode == MODE_FAULT)
                       ? A_BRAKE_MPS2 : A_MAX_MPS2;
  vTarget = rate_limit(vTargetRaw, vTarget, aLimit, dt);

  // -------------------------------------------------------------------
  // 6. BUMPLESS TRANSFER on mode change
  // -------------------------------------------------------------------
  if (mode != prevMode) {
    pid_preload(&speedPid, vTarget, v, vio_last_duty());
    ll_reset(&leadLag);
  }

  // -------------------------------------------------------------------
  // 7. INNER LOOP -- PID (+ optional lead-lag) on speed
  // -------------------------------------------------------------------
  float u = pid_step(&speedPid, vTarget, v);
#if ENABLE_LEADLAG
  u = clampf(ll_step(&leadLag, u), PID_U_MIN, PID_U_MAX);
#endif

  // Hard overrides. Below the controller, above the driver -- these are
  // the "no matter what the loop thinks" rules.
  if (mode == MODE_EMERG || mode == MODE_FAULT) u = -1.0f;      // full brake
  if (v <= V_STANDSTILL_MPS && vTarget <= 0.0f) u = 0.0f;       // stay stopped

  vio_set_duty(u);
  vio_status_led(mode == MODE_PREDICT || mode == MODE_EMERG);

  // -------------------------------------------------------------------
  // 8. TELEMETRY
  // -------------------------------------------------------------------
  if (nowMs - lastTelemMs >= (uint32_t)(1000 / TELEM_HZ)) {
    lastTelemMs = nowMs;
    Serial.printf(
      "F,%lu,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,"
      "%.4f,%.4f,%.4f,%lu,%lu,%lu\n",
      (unsigned long)(nowMs - bootMs),
      dMeas, dDes, closingMps, (ttc > 99.0f ? 99.0f : ttc),
      v, vTarget, vLeadEst, u,
      (int)mode, (int)link, brakeLatched ? 1 : 0,
      speedPid.p_term, speedPid.i_term, speedPid.d_term,
      (unsigned long)pktGood, (unsigned long)pktLost, (unsigned long)pktBad);
  }
}
