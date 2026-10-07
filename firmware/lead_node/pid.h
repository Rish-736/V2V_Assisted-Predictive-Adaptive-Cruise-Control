// >>> GENERATED FILE -- DO NOT EDIT <<<
// Source of truth: firmware/common/pid.h
// Regenerate with: python tools/sync_common.py
// =====================================================================
//  pid.h  --  Inner-loop speed PID.
//  MASTER COPY in firmware/common/.
//
//  Not a textbook three-liner. The things that matter on real hardware:
//
//  1. DERIVATIVE ON MEASUREMENT, not on error.
//     The outer loop steps the speed setpoint. Derivative-on-error would
//     turn every setpoint step into a huge spike into the motor driver
//     ("derivative kick"). Differentiating -y instead gives identical
//     disturbance rejection with no kick.
//
//  2. FILTERED DERIVATIVE.
//     An ideal differentiator has infinite high-frequency gain, so it
//     amplifies encoder quantisation noise straight into the PWM.
//     We use Kd*N*s/(s+N), i.e. a real pole at N rad/s.
//
//  3. BACK-CALCULATION ANTI-WINDUP.
//     Duty saturates at +/-1. Without this, during a long brake the
//     integrator charges up and the car overshoots badly on release.
//     We bleed the integrator by the amount we actually clipped.
//
//  4. BUMPLESS MODE SWITCHING.
//     The state machine hands control between CRUISE / FOLLOW / BRAKE.
//     pid_preload() seeds the integrator so u is continuous across a
//     mode change instead of jumping.
// =====================================================================
#ifndef V2V_PID_H
#define V2V_PID_H

#include "filters.h"

typedef struct {
  // gains
  float kp, ki, kd;
  float n_deriv;       // derivative filter pole [rad/s]
  float u_min, u_max;
  float kaw;           // anti-windup back-calculation gain
  float dt;
  // state
  float integ;
  float d_state;       // filtered derivative state
  float y_prev;
  bool  primed;
  // diagnostics -- exported in telemetry so we can SEE the split
  float p_term, i_term, d_term, u_unsat;
} PID;

static inline void pid_init(PID *c, float kp, float ki, float kd,
                            float n_deriv, float u_min, float u_max, float dt) {
  c->kp = kp; c->ki = ki; c->kd = kd;
  c->n_deriv = n_deriv;
  c->u_min = u_min; c->u_max = u_max;
  c->dt = dt;
  // Standard choice: kaw = 1/Ti = ki/kp. Falls back to 1 if kp is zero.
  c->kaw = (kp > 1e-6f) ? (ki / kp) : 1.0f;
  c->integ = 0.0f;
  c->d_state = 0.0f;
  c->y_prev = 0.0f;
  c->primed = false;
  c->p_term = c->i_term = c->d_term = c->u_unsat = 0.0f;
}

static inline void pid_reset(PID *c) {
  c->integ = 0.0f;
  c->d_state = 0.0f;
  c->primed = false;
  c->p_term = c->i_term = c->d_term = c->u_unsat = 0.0f;
}

// Seed the integrator so that the next output starts at u_desired.
// Used for bumpless transfer when the supervisor changes mode.
static inline void pid_preload(PID *c, float setpoint, float measurement,
                               float u_desired) {
  const float e = setpoint - measurement;
  c->integ = u_desired - c->kp * e;
  c->d_state = 0.0f;
  c->y_prev = measurement;
  c->primed = true;
}

// One control step. `setpoint` and `measurement` in m/s, returns duty.
static inline float pid_step(PID *c, float setpoint, float measurement) {
  const float e = setpoint - measurement;

  if (!c->primed) { c->y_prev = measurement; c->primed = true; }

  // --- proportional
  c->p_term = c->kp * e;

  // --- derivative ON MEASUREMENT, filtered, negated
  //     dfilt[k] = (N*dt*(-dy) + dfilt[k-1]) / (1 + N*dt)  (backward Euler)
  const float dy = (measurement - c->y_prev) / c->dt;
  c->y_prev = measurement;
  const float nd = c->n_deriv * c->dt;
  c->d_state = (c->d_state + nd * (-dy)) / (1.0f + nd);
  c->d_term = c->kd * c->d_state;

  // --- integral (trapezoid-free forward Euler; dt is fixed and small)
  c->integ += c->ki * e * c->dt;
  c->i_term = c->integ;

  // --- sum, saturate, then back-calculate the windup correction
  c->u_unsat = c->p_term + c->i_term + c->d_term;
  const float u = clampf(c->u_unsat, c->u_min, c->u_max);
  if (u != c->u_unsat) {
    c->integ += c->kaw * (u - c->u_unsat) * c->dt;
    c->i_term = c->integ;
  }
  return u;
}

#endif // V2V_PID_H
