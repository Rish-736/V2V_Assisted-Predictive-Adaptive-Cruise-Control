// =====================================================================
//  filters.h  --  Small, allocation-free signal conditioning blocks.
//  MASTER COPY in firmware/common/.
//
//  Why this file exists:
//  The single biggest practical obstacle in this project is that the
//  predictive braking layer differentiates the range signal. Raw HC-SR04
//  noise is roughly +/- 1-2 cm with occasional total outliers (a missed
//  echo reads as max range). Differentiating that at 20 Hz gives closing
//  rates of several m/s out of a stationary target, which would slam the
//  brakes on for no reason.
//
//  Chain used on the follower:  median3 -> low-pass -> differentiate
//                               -> low-pass again
//  Median kills the impulsive outliers (a low-pass alone just smears
//  them); the low-passes handle the gaussian part.
// =====================================================================
#ifndef V2V_FILTERS_H
#define V2V_FILTERS_H

#include <math.h>
#include <stdint.h>

// ---------------------------------------------------------------------
//  First-order low-pass, discretised by the bilinear-equivalent
//  exponential form:  y[k] = y[k-1] + alpha*(x[k] - y[k-1])
//  with alpha = dt / (dt + 1/(2*pi*fc)).
// ---------------------------------------------------------------------
typedef struct {
  float y;
  float alpha;
  bool  primed;
} LowPass;

static inline void lp_init(LowPass *f, float fc_hz, float dt) {
  const float rc = 1.0f / (2.0f * (float)M_PI * fc_hz);
  f->alpha  = dt / (dt + rc);
  f->y      = 0.0f;
  f->primed = false;
}

static inline float lp_step(LowPass *f, float x) {
  if (!f->primed) { f->y = x; f->primed = true; return f->y; }
  f->y += f->alpha * (x - f->y);
  return f->y;
}

static inline void lp_reset(LowPass *f) { f->y = 0.0f; f->primed = false; }

// ---------------------------------------------------------------------
//  Median-of-3. Removes single-sample impulses with only 2 samples of
//  group delay, which matters when the whole point is early reaction.
// ---------------------------------------------------------------------
typedef struct {
  float w[3];
  uint8_t n;
} Median3;

static inline void med3_init(Median3 *m) { m->n = 0; m->w[0] = m->w[1] = m->w[2] = 0.0f; }

static inline float med3_step(Median3 *m, float x) {
  m->w[2] = m->w[1];
  m->w[1] = m->w[0];
  m->w[0] = x;
  if (m->n < 3) { m->n++; return x; }     // not enough history yet
  const float a = m->w[0], b = m->w[1], c = m->w[2];
  // branch-free-ish median of three
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

// ---------------------------------------------------------------------
//  Filtered differentiator. Produces d(x)/dt with a low-pass on the
//  output. Sign convention is left to the caller.
// ---------------------------------------------------------------------
typedef struct {
  float   prev;
  bool    primed;
  LowPass lp;
  float   dt;
} Differentiator;

static inline void diff_init(Differentiator *d, float fc_hz, float dt) {
  d->prev = 0.0f;
  d->primed = false;
  d->dt = dt;
  lp_init(&d->lp, fc_hz, dt);
}

static inline float diff_step(Differentiator *d, float x) {
  if (!d->primed) { d->prev = x; d->primed = true; return 0.0f; }
  const float raw = (x - d->prev) / d->dt;
  d->prev = x;
  return lp_step(&d->lp, raw);
}

static inline void diff_reset(Differentiator *d) {
  d->primed = false;
  lp_reset(&d->lp);
}

// ---------------------------------------------------------------------
//  Rate limiter -- enforces the comfort acceleration limit on a speed
//  setpoint so the outer loop cannot command a step the plant could
//  never follow (which would just wind up the integrator).
// ---------------------------------------------------------------------
static inline float rate_limit(float target, float current, float max_rate, float dt) {
  const float max_delta = max_rate * dt;
  const float delta = target - current;
  if (delta >  max_delta) return current + max_delta;
  if (delta < -max_delta) return current - max_delta;
  return target;
}

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

// ---------------------------------------------------------------------
//  Discrete lead-lag / generic first-order biquad section:
//     y[k] = b0*x[k] + b1*x[k-1] - a1*y[k-1]
// ---------------------------------------------------------------------
typedef struct {
  float b0, b1, a1;
  float x1, y1;
} LeadLag;

static inline void ll_init(LeadLag *f, float b0, float b1, float a1) {
  f->b0 = b0; f->b1 = b1; f->a1 = a1;
  f->x1 = f->y1 = 0.0f;
}

static inline float ll_step(LeadLag *f, float x) {
  const float y = f->b0 * x + f->b1 * f->x1 - f->a1 * f->y1;
  f->x1 = x;
  f->y1 = y;
  return y;
}

static inline void ll_reset(LeadLag *f) { f->x1 = f->y1 = 0.0f; }

#endif // V2V_FILTERS_H
