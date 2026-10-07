// =====================================================================
//  step_response.ino  --  SYSTEM IDENTIFICATION DATA CAPTURE
//
//  This sketch produces the single most important measurement in the
//  project: the open-loop step response of the real motor. Everything
//  downstream -- the PID gains, the root locus, the Bode plots, the
//  lead-lag design, the digital twin -- is built on the transfer
//  function fitted to this data.
//
//  HOW TO RUN
//    1. Put the car on blocks so the wheels spin free, or on a long
//       straight run. Blocks are better: no wheel slip, no wall.
//    2. Flash this to the node under test. Open the serial monitor.
//    3. It runs a sequence of steps automatically and prints CSV.
//    4. Copy the serial output into data/step_<name>.csv
//       (or use: python python/scripts/log_serial.py --out data/step.csv)
//    5. Fit it:  python python/scripts/sysid_fit.py data/step.csv
//
//  WHY SEVERAL STEP AMPLITUDES
//    A DC motor with a gearbox is NOT linear near zero -- static
//    friction means small duties produce no motion at all (the
//    dead-zone). Stepping at 0.3 / 0.5 / 0.7 / 1.0 lets the fitting
//    script see how K and tau vary with operating point, and lets us
//    state honestly in the report where the linear model is valid.
//    A single step would hide that completely.
//
//  WHY BOTH RISING AND FALLING STEPS
//    Spin-up is driven by the motor; spin-down is driven by friction
//    and back-EMF. They have different time constants. The controller
//    has to handle both, so we measure both.
// =====================================================================

#include <Arduino.h>
#include "config.h"
#include "filters.h"

// ---- sampling -------------------------------------------------------
#define SAMPLE_HZ        200          // faster than the control loop, so
#define SAMPLE_DT        (1.0f / (float)SAMPLE_HZ)   // we resolve tau well
#define SETTLE_MS        2500         // hold each level this long
#define REST_MS          2000         // rest between steps

// ---- test sequence --------------------------------------------------
static const float STEP_LEVELS[] = {0.30f, 0.50f, 0.70f, 1.00f};
static const int   N_LEVELS = sizeof(STEP_LEVELS) / sizeof(STEP_LEVELS[0]);

// ---- encoder --------------------------------------------------------
static volatile int32_t encCount = 0;
static int32_t encPrev = 0;

static void IRAM_ATTR encIsr(void) {
  if (digitalRead(PIN_ENC_B)) encCount++;
  else                        encCount--;
}

static float mPerPulse(void) {
  return ((float)M_PI * WHEEL_DIAMETER_M) / (ENC_PPR * ENC_GEAR_RATIO);
}

static void writePwm(float duty) {
  const uint32_t full = (1u << PWM_RESOLUTION_BITS) - 1u;
  duty = clampf(duty, 0.0f, 1.0f);
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_MOTOR_PWM, (uint32_t)(duty * (float)full));
#else
  ledcWrite(PWM_CHANNEL, (uint32_t)(duty * (float)full));
#endif
}

static void motorOff(void) {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_MOTOR_PWM, 0);
#else
  ledcWrite(PWM_CHANNEL, 0);
#endif
}

// Log one phase at a fixed duty for `durationMs`, printing raw samples.
static void logPhase(float duty, uint32_t durationMs, const char *tag,
                     uint32_t t0) {
  writePwm(duty);
  uint32_t nextUs = micros();
  const uint32_t endMs = millis() + durationMs;

  while (millis() < endMs) {
    while ((int32_t)(micros() - nextUs) < 0) { /* spin to the next tick */ }
    nextUs += (uint32_t)(SAMPLE_DT * 1e6f);

    noInterrupts();
    const int32_t c = encCount;
    interrupts();
    const int32_t d = c - encPrev;
    encPrev = c;

    const float speed = ((float)d * mPerPulse()) / SAMPLE_DT;

    // CSV: t_ms, duty, speed_mps, enc_count, tag
    Serial.printf("%lu,%.4f,%.5f,%ld,%s\n",
                  (unsigned long)(millis() - t0), duty, speed, (long)c, tag);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PIN_MOTOR_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(PIN_MOTOR_PWM, PWM_CHANNEL);
#endif
  motorOff();

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encIsr, RISING);

  Serial.println();
  Serial.println("# step-response capture");
  Serial.printf ("# sample_hz=%d settle_ms=%d levels=%d\n",
                 SAMPLE_HZ, SETTLE_MS, N_LEVELS);
  Serial.printf ("# wheel_d=%.4f enc_ppr=%.1f gear=%.1f\n",
                 WHEEL_DIAMETER_M, ENC_PPR, ENC_GEAR_RATIO);
  Serial.println("# starting in 3 s -- make sure the wheels are clear");
  delay(3000);
  Serial.println("t_ms,duty,speed_mps,enc_count,tag");

  const uint32_t t0 = millis();
  noInterrupts(); encCount = 0; interrupts();
  encPrev = 0;

  for (int i = 0; i < N_LEVELS; ++i) {
    char up[16], down[16];
    snprintf(up,   sizeof(up),   "up_%d",   (int)(STEP_LEVELS[i] * 100));
    snprintf(down, sizeof(down), "down_%d", (int)(STEP_LEVELS[i] * 100));

    logPhase(STEP_LEVELS[i], SETTLE_MS, up,   t0);   // rising edge
    motorOff();
    logPhase(0.0f,           REST_MS,   down, t0);   // falling edge (coast)
  }

  motorOff();
  Serial.println("# done");
}

void loop() {
  motorOff();
  delay(1000);
}
