// >>> GENERATED FILE -- DO NOT EDIT <<<
// Source of truth: firmware/common/vehicle_io.h
// Regenerate with: python tools/sync_common.py
// =====================================================================
//  vehicle_io.h  --  Hardware abstraction layer.
//  MASTER COPY in firmware/common/.
//
//  THIS IS THE ONLY FILE THAT KNOWS ABOUT SIM_PLANT.
//
//  Everything above it -- the PID, the time-gap law, the predictive
//  braking, the state machine -- calls the same five functions whether
//  it is driving a real L298N or a difference equation. That is what
//  lets us develop and demo the complete control system on two bare
//  ESP32 boards, then move to real motors by changing one #define
//  without touching a single line of control code.
//
//      vio_begin()        bring up I/O (or the sim state)
//      vio_set_duty(u)    command normalised duty, u in [-1, +1]
//      vio_speed()        measured forward speed [m/s]
//      vio_range()        measured gap to the vehicle ahead [m]
//      vio_update(dt)     advance sensors / the simulated plant
//
//  The simulated plant is the SAME first-order model the Python digital
//  twin uses (config.h section 8), including dead-zone and transport
//  delay, so sim-mode telemetry is directly comparable to twin output.
// =====================================================================
#ifndef V2V_VEHICLE_IO_H
#define V2V_VEHICLE_IO_H

#include <Arduino.h>
#include "config.h"
#include "filters.h"

// ---------------------------------------------------------------------
//  Shared state
// ---------------------------------------------------------------------
static float   vio_duty_cmd   = 0.0f;   // last commanded duty
static float   vio_speed_mps  = 0.0f;   // measured / simulated speed
static float   vio_range_m    = 1.00f;  // measured / simulated gap
static bool    vio_range_valid = true;
static uint8_t vio_range_misses = 0;

// =====================================================================
// =========================  SIMULATED PLANT  =========================
// =====================================================================
#if SIM_PLANT

// transport-delay line for the duty command
#define VIO_DELAY_TAPS  16
static float   vio_delay_buf[VIO_DELAY_TAPS];
static uint8_t vio_delay_idx = 0;
static uint8_t vio_delay_n   = 1;
static uint32_t vio_noise_seed = 0xC0FFEEu;

// cheap xorshift so the sim has a little measurement noise, like reality
static inline float vio_noise(float amplitude) {
  vio_noise_seed ^= vio_noise_seed << 13;
  vio_noise_seed ^= vio_noise_seed >> 17;
  vio_noise_seed ^= vio_noise_seed << 5;
  const float u = (float)(vio_noise_seed & 0xFFFFu) / 65535.0f;  // 0..1
  return (u - 0.5f) * 2.0f * amplitude;
}

static inline void vio_begin(void) {
  pinMode(PIN_STATUS_LED, OUTPUT);
  for (uint8_t i = 0; i < VIO_DELAY_TAPS; ++i) vio_delay_buf[i] = 0.0f;
  vio_delay_n = (uint8_t)max(1.0f, roundf(PLANT_DELAY_S / CTRL_DT));
  if (vio_delay_n >= VIO_DELAY_TAPS) vio_delay_n = VIO_DELAY_TAPS - 1;
  vio_speed_mps = 0.0f;
  vio_range_m = 1.00f;
  vio_range_valid = true;
}

static inline void vio_set_duty(float u) {
  vio_duty_cmd = clampf(u, -1.0f, 1.0f);
}

// Advance the first-order motor model one step.
//   tau * dv/dt + v = K * u_effective
// with a symmetric dead-zone (static friction) on the delayed command.
static inline void vio_update(float dt) {
  // push into the delay line, pop the delayed sample
  vio_delay_buf[vio_delay_idx] = vio_duty_cmd;
  vio_delay_idx = (uint8_t)((vio_delay_idx + 1) % VIO_DELAY_TAPS);
  const uint8_t tap = (uint8_t)((vio_delay_idx + VIO_DELAY_TAPS - vio_delay_n) % VIO_DELAY_TAPS);
  float u = vio_delay_buf[tap];

  // dead-zone: below break-away duty the wheel does not turn
  if (fabsf(u) < PLANT_DEADZONE) {
    u = 0.0f;
  } else {
    u = (u > 0.0f) ? (u - PLANT_DEADZONE) / (1.0f - PLANT_DEADZONE)
                   : (u + PLANT_DEADZONE) / (1.0f - PLANT_DEADZONE);
  }

  const float v_inf = PLANT_K * u;
  vio_speed_mps += (dt / PLANT_TAU) * (v_inf - vio_speed_mps);

  // a scale car cannot reverse out of a braking command
  if (vio_speed_mps < 0.0f) vio_speed_mps = 0.0f;
  if (vio_speed_mps > V_MAX_MPS) vio_speed_mps = V_MAX_MPS;
}

// Follower only: derive the gap by integrating relative velocity.
// This is what replaces the HC-SR04 in sim mode.
static inline void vio_sim_integrate_gap(float v_lead, float v_follow, float dt) {
  vio_range_m += (v_lead - v_follow) * dt;
  if (vio_range_m < 0.0f)        vio_range_m = 0.0f;   // contact
  if (vio_range_m > ULTRA_MAX_M) vio_range_m = ULTRA_MAX_M;
  vio_range_valid = true;
}

static inline void vio_sim_set_gap(float d) { vio_range_m = d; }

// Speed "measurement" = true state + a little encoder quantisation noise.
static inline float vio_speed(void) { return vio_speed_mps + vio_noise(0.004f); }

// Range "measurement" = true gap + HC-SR04-like noise, so the median and
// low-pass filters in the follower are actually exercised in sim.
static inline float vio_range(void) { return vio_range_m + vio_noise(0.010f); }

static inline bool  vio_range_ok(void) { return vio_range_valid; }

// =====================================================================
// ==========================  REAL HARDWARE  ==========================
// =====================================================================
#else   // SIM_PLANT == 0

static volatile int32_t vio_enc_count = 0;
static int32_t  vio_enc_prev = 0;
static uint8_t  vio_win_n    = 0;      // ticks accumulated in this window
static float    vio_win_dt   = 0.0f;   // true elapsed time of the window
static LowPass  vio_speed_lp;
static Median3  vio_range_med;
static LowPass  vio_range_lp;

// metres of wheel travel per encoder pulse
static inline float vio_m_per_pulse(void) {
  const float pulses_per_wheel_rev = ENC_PPR * ENC_GEAR_RATIO;
  const float wheel_circumference  = (float)M_PI * WHEEL_DIAMETER_M;
  return wheel_circumference / pulses_per_wheel_rev;
}

// Quadrature decode on channel A edges, direction from channel B.
// Half-quadrature (1x) is plenty here and halves the ISR load.
static void IRAM_ATTR vio_enc_isr(void) {
  if (digitalRead(PIN_ENC_B)) vio_enc_count++;
  else                        vio_enc_count--;
}

static inline void vio_begin(void) {
  pinMode(PIN_STATUS_LED, OUTPUT);

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Core 3.x API
  ledcAttach(PIN_MOTOR_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
  // Core 2.x API
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(PIN_MOTOR_PWM, PWM_CHANNEL);
#endif

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), vio_enc_isr, RISING);

  pinMode(PIN_ULTRA_TRIG, OUTPUT);
  pinMode(PIN_ULTRA_ECHO, INPUT);
  digitalWrite(PIN_ULTRA_TRIG, LOW);

  lp_init(&vio_speed_lp, ENC_SPEED_LPF_FC_HZ, CTRL_DT);
  med3_init(&vio_range_med);
  lp_init(&vio_range_lp, ULTRA_LPF_FC_HZ, CTRL_DT);
}

static inline void vio_write_pwm(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_MOTOR_PWM, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
}

//  u > 0 : drive forward
//  u < 0 : active brake. We use the L298N short-brake (both inputs HIGH)
//          rather than reversing, because a scale car reversing mid-stop
//          is not a behaviour any real ACC has.
static inline void vio_set_duty(float u) {
  vio_duty_cmd = clampf(u, -1.0f, 1.0f);
  const uint32_t full = (1u << PWM_RESOLUTION_BITS) - 1u;

  if (vio_duty_cmd > 0.001f) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    vio_write_pwm((uint32_t)(vio_duty_cmd * (float)full));
  } else if (vio_duty_cmd < -0.001f) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);   // both HIGH = short brake
    digitalWrite(PIN_MOTOR_IN2, HIGH);
    vio_write_pwm((uint32_t)(-vio_duty_cmd * (float)full));
  } else {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    vio_write_pwm(0);
  }
}

// Blocking HC-SR04 ping. ~6 ms worst case at 1 m, which fits inside a
// 20 ms control period, but see docs/ARCHITECTURE.md for the reason we
// only ping every other control tick.
static inline float vio_ping_raw(void) {
  digitalWrite(PIN_ULTRA_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRA_TRIG, LOW);

  const unsigned long us = pulseIn(PIN_ULTRA_ECHO, HIGH, ULTRA_TIMEOUT_US);
  if (us == 0) return -1.0f;             // timeout / no echo
  return (float)us * 0.0001715f;         // us * 343 m/s / 2, in metres
}

static inline void vio_update(float dt) {
  // ---- speed from encoder deltas, over a SLIDING WINDOW
  //  At a 0.15 m/s cruise this encoder yields only ~5.5 pulses per 20 ms
  //  tick, so differencing every tick quantises the speed estimate to
  //  ~2.7 cm/s -- about 18% of cruise -- and feeds mostly quantisation
  //  noise to the PID. Accumulating over ENC_SPEED_WINDOW_TICKS ticks and
  //  dividing by the true elapsed time gives N times the resolution for a
  //  little phase lag. The control loop still runs at CTRL_HZ.
  vio_win_dt += dt;
  if (++vio_win_n >= ENC_SPEED_WINDOW_TICKS) {
    noInterrupts();
    const int32_t c = vio_enc_count;
    interrupts();
    const int32_t d = c - vio_enc_prev;
    vio_enc_prev = c;
    const float raw = ((float)d * vio_m_per_pulse()) / vio_win_dt;
    vio_speed_mps = lp_step(&vio_speed_lp, raw);
    vio_win_n  = 0;
    vio_win_dt = 0.0f;
  }

  // ---- range: median -> low-pass, with miss counting
  const float r = vio_ping_raw();
  if (r < ULTRA_MIN_M || r > ULTRA_MAX_M) {
    if (vio_range_misses < 255) vio_range_misses++;
    if (vio_range_misses >= ULTRA_MAX_MISSES) vio_range_valid = false;
  } else {
    vio_range_misses = 0;
    vio_range_valid = true;
    vio_range_m = lp_step(&vio_range_lp, med3_step(&vio_range_med, r));
  }
}

static inline void vio_sim_integrate_gap(float v_lead, float v_follow, float dt) {
  (void)v_lead; (void)v_follow; (void)dt;   // real sensor supplies the gap
}
static inline void vio_sim_set_gap(float d) { (void)d; }

static inline float vio_speed(void)   { return vio_speed_mps; }
static inline float vio_range(void)   { return vio_range_m; }
static inline bool  vio_range_ok(void){ return vio_range_valid; }

#endif  // SIM_PLANT

// ---------------------------------------------------------------------
//  Common helpers
// ---------------------------------------------------------------------
static inline float vio_last_duty(void) { return vio_duty_cmd; }

static inline void vio_coast(void) { vio_set_duty(0.0f); }

static inline void vio_status_led(bool on) { digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW); }

#endif // V2V_VEHICLE_IO_H
