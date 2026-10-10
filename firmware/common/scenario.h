// =====================================================================
//  scenario.h  --  Demo scenario engine (runs on the LEAD node).
//  MASTER COPY in firmware/common/. Run tools/sync_common.py to propagate.
//
//  WHY THE SCENARIOS LIVE ON THE LEAD
//  Every scenario is a statement about what the LEAD vehicle does -- cruise,
//  ease off, stop hard, go quiet. The follower has no scenario logic at all;
//  it just reacts to what it senses and receives, exactly as it would on a
//  real road. That keeps the thing being demonstrated honest: we are not
//  scripting the controller, we are scripting the world it sees.
//
//  WHY A SCENARIO ENGINE AT ALL
//  A demo you cannot repeat on demand is not a demo, it is a stunt. With
//  this you press one button and the same event plays out identically, as
//  many times as the panel wants to see it. It also means the digital-twin
//  comparison is meaningful, because both runs saw the same input.
//
//  The active scenario id rides in the V2V packet, so the follower's
//  telemetry -- and therefore the dashboard and every recorded log -- is
//  automatically annotated with which scenario produced it.
// =====================================================================
#ifndef V2V_SCENARIO_H
#define V2V_SCENARIO_H

#include <Arduino.h>
#include "config.h"
#include "filters.h"

// ---------------------------------------------------------------------
//  Scenario ids. Keep in sync with SCENARIO_NAMES below, with
//  python/v2vacc/telemetry.py, and with docs/DEMO_PLAN.md section 4.
// ---------------------------------------------------------------------
enum ScenarioId {
  SCN_MANUAL     = 0,   // throttle pot drives the lead; you drive it by hand
  SCN_FOLLOW     = 1,   // steady cruise -- shows the time gap being held
  SCN_PREDICT    = 2,   // gentle slowdown -- the predictive-braking case
  SCN_EMERGENCY  = 3,   // hard stop -- full braking authority
  SCN_COMM_LOSS  = 4,   // cruise, but the lead MUTES ITS TRANSMITTER
  SCN_COUNT      = 5
};

static const char *SCENARIO_NAMES[] = {
  "MANUAL", "FOLLOW", "PREDICT", "EMERG-STOP", "COMM-LOSS"
};

// ---------------------------------------------------------------------
//  Engine state
// ---------------------------------------------------------------------
typedef struct {
  ScenarioId id;
  bool       running;      // false -> holding at the start condition
  uint32_t   t0_ms;        // when the current run began
  float      elapsed_s;
  float      cmd_speed;    // commanded lead speed [m/s] (pre rate-limit)
  bool       hazard;       // declare an emergency to the follower
  bool       tx_enabled;   // false -> radio silence (the comm-loss case)
  bool       finished;
} Scenario;

static inline void scn_init(Scenario *s) {
  s->id         = SCN_FOLLOW;
  s->running    = false;
  s->t0_ms      = millis();
  s->elapsed_s  = 0.0f;
  s->cmd_speed  = 0.0f;
  s->hazard     = false;
  s->tx_enabled = true;
  s->finished   = false;
}

static inline void scn_select(Scenario *s, ScenarioId id) {
  s->id         = id;
  s->running    = false;
  s->elapsed_s  = 0.0f;
  s->cmd_speed  = 0.0f;
  s->hazard     = false;
  s->tx_enabled = true;
  s->finished   = false;
}

static inline void scn_start(Scenario *s) {
  s->running    = true;
  s->t0_ms      = millis();
  s->elapsed_s  = 0.0f;
  s->hazard     = false;
  s->tx_enabled = true;
  s->finished   = false;
}

static inline void scn_next(Scenario *s) {
  scn_select(s, (ScenarioId)(((int)s->id + 1) % SCN_COUNT));
}

static inline const char *scn_name(const Scenario *s) {
  return SCENARIO_NAMES[(int)s->id];
}

// ---------------------------------------------------------------------
//  scn_update -- advance the scenario one control tick.
//
//  `throttle` is the filtered pot value in [0,1], used only by SCN_MANUAL.
//  `brake_btn` is the hand brake, which is honoured in EVERY scenario --
//  you can always stamp on the brake, which is both realistic and useful
//  when something is about to run off the end of the bench.
//
//  Each scripted scenario holds for SCN_LEAD_IN seconds, does its thing,
//  then settles and reports finished. The runner loops back to the start
//  condition so the next press replays it identically.
// ---------------------------------------------------------------------
#define SCN_LEAD_IN_S       2.0f    // steady cruise before the event
#define SCN_EVENT_HOLD_S    4.0f    // how long the event condition persists
#define SCN_RECOVER_S       4.0f    // back to cruise afterwards
#define SCN_CRUISE          SCENARIO_CRUISE_MPS
#define SCN_SLOWDOWN        (SCENARIO_CRUISE_MPS * 0.35f)

static inline void scn_update(Scenario *s, float throttle, bool brake_btn) {
  if (s->running) {
    s->elapsed_s = (float)(millis() - s->t0_ms) * 1e-3f;
  }
  const float t = s->elapsed_s;

  s->tx_enabled = true;
  s->hazard     = false;

  switch (s->id) {

    // --- you drive it. The pot is the accelerator.
    case SCN_MANUAL:
      s->cmd_speed = clampf(throttle, 0.0f, 1.0f) * V_MAX_MPS;
      s->running   = true;             // manual is always live
      s->finished  = false;
      break;

    // --- steady cruise: shows the follower settling onto the time gap
    case SCN_FOLLOW:
      s->cmd_speed = s->running ? SCN_CRUISE : 0.0f;
      s->finished  = s->running && (t > SCN_LEAD_IN_S + SCN_EVENT_HOLD_S
                                        + SCN_RECOVER_S);
      break;

    // --- gentle slowdown: the predictive-braking case.
    //     The follower should start easing off from the V2V braking flag
    //     BEFORE the gap has visibly changed. That is the whole claim.
    case SCN_PREDICT:
      if (!s->running)                       s->cmd_speed = 0.0f;
      else if (t < SCN_LEAD_IN_S)            s->cmd_speed = SCN_CRUISE;
      else if (t < SCN_LEAD_IN_S + SCN_EVENT_HOLD_S)
                                             s->cmd_speed = SCN_SLOWDOWN;
      else                                   s->cmd_speed = SCN_CRUISE;
      s->finished = s->running && (t > SCN_LEAD_IN_S + SCN_EVENT_HOLD_S
                                       + SCN_RECOVER_S);
      break;

    // --- hard stop. Sets the hazard flag, which escalates the follower
    //     straight to EMERG with full braking authority.
    case SCN_EMERGENCY:
      if (!s->running)                       s->cmd_speed = 0.0f;
      else if (t < SCN_LEAD_IN_S)            s->cmd_speed = SCN_CRUISE;
      else if (t < SCN_LEAD_IN_S + SCN_EVENT_HOLD_S) {
        s->cmd_speed = 0.0f;
        s->hazard    = true;
      } else                                 s->cmd_speed = SCN_CRUISE;
      s->finished = s->running && (t > SCN_LEAD_IN_S + SCN_EVENT_HOLD_S
                                       + SCN_RECOVER_S);
      break;

    // --- COMM LOSS. The lead keeps cruising perfectly normally but goes
    //     RADIO SILENT. Nothing is wrong with the lead; the follower has
    //     simply lost its feedforward channel.
    //     Expected follower behaviour: NOMINAL -> DEGRADED -> LOST, and the
    //     desired gap widens from T_GAP_S to T_GAP_DEGRADED_S. It keeps
    //     driving safely on the ultrasonic alone -- degradation, not failure.
    //     (A tin can over this node does the same thing physically.)
    case SCN_COMM_LOSS:
      s->cmd_speed = s->running ? SCN_CRUISE : 0.0f;
      if (s->running && t > SCN_LEAD_IN_S
                     && t < SCN_LEAD_IN_S + SCN_EVENT_HOLD_S * 2.0f) {
        s->tx_enabled = false;         // <-- the radio goes quiet here
      }
      s->finished = s->running && (t > SCN_LEAD_IN_S
                                     + SCN_EVENT_HOLD_S * 2.0f + SCN_RECOVER_S);
      break;

    default:
      s->cmd_speed = 0.0f;
      break;
  }

  // The hand brake overrides everything, in every scenario.
  if (brake_btn) {
    s->cmd_speed = 0.0f;
    s->hazard    = true;
  }

  // Auto-return to the start condition so the next press replays cleanly.
  if (s->finished) {
    s->running   = false;
    s->cmd_speed = 0.0f;
    s->elapsed_s = 0.0f;
  }
}

// =====================================================================
//  Debounced button helper
// =====================================================================
typedef struct {
  uint8_t  pin;
  bool     stable;        // debounced level (true = pressed, active-low pin)
  bool     last_raw;
  uint32_t last_change_ms;
  bool     edge;          // true for one call after a press
} Button;

static inline void btn_init(Button *b, uint8_t pin) {
  b->pin            = pin;
  b->stable         = false;
  b->last_raw       = false;
  b->last_change_ms = millis();
  b->edge           = false;
  pinMode(pin, INPUT_PULLUP);
}

// Returns true exactly once per press (rising edge, after debounce).
static inline bool btn_pressed(Button *b) {
  const bool raw = (digitalRead(b->pin) == LOW);    // active low
  const uint32_t now = millis();
  b->edge = false;

  if (raw != b->last_raw) {
    b->last_raw = raw;
    b->last_change_ms = now;
  } else if ((now - b->last_change_ms) >= BTN_DEBOUNCE_MS && raw != b->stable) {
    b->stable = raw;
    if (raw) b->edge = true;
  }
  return b->edge;
}

static inline bool btn_held(const Button *b) { return b->stable; }

// =====================================================================
//  Throttle pot helper
// =====================================================================
typedef struct {
  LowPass lp;
  float   value;          // 0..1
} Throttle;

static inline void thr_init(Throttle *t) {
  lp_init(&t->lp, POT_LPF_FC_HZ, CTRL_DT);
  t->value = 0.0f;
}

static inline float thr_read(Throttle *t) {
#if LEAD_ENABLE_POT
  // 12-bit ADC. The ESP32 ADC is noisy and nonlinear near the rails, so we
  // filter it and treat the bottom of the travel as a hard zero -- otherwise
  // the "idle" position still commands a crawl.
  const float raw = (float)analogRead(PIN_LEAD_POT) / 4095.0f;
  float v = lp_step(&t->lp, raw);
  if (v < POT_DEADBAND)        v = 0.0f;
  if (v > 1.0f - POT_DEADBAND) v = 1.0f;
  t->value = v;
#else
  t->value = 0.0f;
#endif
  return t->value;
}

#endif // V2V_SCENARIO_H
