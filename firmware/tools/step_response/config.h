// >>> GENERATED FILE -- DO NOT EDIT <<<
// Source of truth: firmware/common/config.h
// Regenerate with: python tools/sync_common.py
// =====================================================================
//  config.h  --  Single source of truth for every tunable parameter.
//  MASTER COPY in firmware/common/. Run tools/sync_common.py to propagate.
// =====================================================================
#ifndef V2V_CONFIG_H
#define V2V_CONFIG_H

// =====================================================================
//  0. BUILD MODE  -- the most important switch in the project
// =====================================================================
//  SIM_PLANT 1 : no motor, no encoder, no ultrasonic required.
//                The node integrates a discrete motor model in place of
//                the encoder, and the follower derives gap by integrating
//                relative velocity. The V2V radio link, the control laws,
//                the state machine and the telemetry are all REAL.
//                -> lets us develop + demo the whole system on two bare
//                   ESP32s while the mechanical BOM is still on order.
//  SIM_PLANT 0 : real encoder, real L298N PWM, real HC-SR04.
//
//  Nothing in the control code reads this flag. Only the I/O layer does.
//  That is the point: the controller cannot tell the difference, so a
//  controller validated in sim is the same binary logic on hardware.
#ifndef SIM_PLANT
#define SIM_PLANT              1
#endif

// =====================================================================
//  1. TIMING
// =====================================================================
#define CTRL_HZ                50      // control loop rate [Hz]
#define CTRL_DT                (1.0f / (float)CTRL_HZ)
#define V2V_TX_HZ              20      // lead broadcast rate [Hz]
#define TELEM_HZ               25      // serial telemetry rate [Hz]
#define SERIAL_BAUD            115200

// =====================================================================
//  2. VEHICLE / PLANT LIMITS
// =====================================================================
#define V_MAX_MPS              0.60f   // top speed of the scale car [m/s]
#define V_STANDSTILL_MPS       0.03f   // below this we call it stopped
#define A_MAX_MPS2             0.80f   // comfort accel limit [m/s^2]
#define A_BRAKE_MPS2           1.60f   // hard-braking decel limit [m/s^2]

// =====================================================================
//  3. OUTER LOOP -- constant time-gap spacing policy
// =====================================================================
//  Desired spacing:  d_des = D_STANDSTILL + T_GAP * v_follow
//  Target speed:     v_tgt = v_lead + K_GAP * (d_meas - d_des)
//
//  v_lead comes from the V2V packet -> the lead speed is FEEDFORWARD,
//  not something we have to infer by differentiating a noisy range
//  sensor. That is the core of why this is predictive, not reactive.
#define T_GAP_S                1.50f   // nominal time headway [s]
#define T_GAP_DEGRADED_S       2.20f   // headway when the V2V link is lost
#define D_STANDSTILL_M         0.20f   // bumper-to-bumper gap at rest [m]
#define K_GAP                  0.90f   // outer-loop gap gain [1/s]

// =====================================================================
//  4. INNER LOOP -- PID speed controller
// =====================================================================
//  Replace these with the values that design_control.py / MATLAB produce
//  once the step-response sysID is done. These are sane starting guesses
//  for the nominal plant in section 8.
#define PID_KP                 1.80f
#define PID_KI                 6.00f
#define PID_KD                 0.04f
#define PID_N_DERIV            12.0f   // derivative filter pole [rad/s]
#define PID_U_MIN             -1.00f   // normalised duty, negative = brake
#define PID_U_MAX              1.00f

// =====================================================================
//  5. LEAD-LAG COMPENSATOR (series, after PID)
// =====================================================================
//  Discrete biquad: u[k] = b0*e[k] + b1*e[k-1] - a1*u[k-1]
//  Coefficients come from matlab/design_pid_leadlag.m or
//  python/scripts/design_control.py -- paste them here.
#define ENABLE_LEADLAG         0
#define LL_B0                  1.0000f
#define LL_B1                 -0.8500f
#define LL_A1                 -0.9000f

// =====================================================================
//  6. PREDICTIVE BRAKING LAYER
// =====================================================================
//  Three independent danger signals, OR-ed. Whichever fires first wins.
//    (a) V2V:  lead says it is braking / hazard          -> zero latency
//    (b) TTC:  gap / closing-rate below threshold        -> anticipatory
//    (c) HARD: absolute gap below floor                  -> last resort
#define TTC_BRAKE_S            1.20f   // brake if time-to-contact under this
#define TTC_EMERGENCY_S        0.60f   // full stop below this
#define D_BRAKE_FLOOR_M        0.15f   // absolute minimum gap [m]
#define CLOSING_RATE_MIN_MPS   0.02f   // ignore TTC below this closing rate
#define BRAKE_RELEASE_HYST     1.35f   // must clear threshold x this to exit

//  MODE DWELL. Without this the supervisor can chatter between adjacent
//  modes several times a second when a signal sits right on a boundary
//  (most visible in the degraded sensor-only path, where the closing-rate
//  estimate is noisy). Chattering is not just ugly on a plot: every mode
//  change triggers a bumpless-transfer reload of the PID, so a dithering
//  supervisor repeatedly resets the integrator and the speed loop never
//  settles. A minimum dwell time fixes it with one counter.
//  ESCALATION to a more dangerous mode always bypasses the dwell -- you
//  never delay a brake. Only the de-escalation is held back.
#define MODE_MIN_DWELL_MS      250

// =====================================================================
//  7. V2V LINK HEALTH
// =====================================================================
//  Losing the V2V channel means losing the FEEDFORWARD term. The system
//  is still stable on the ultrasonic feedback alone -- it just degrades
//  from predictive to reactive. So we do not fault: we fall back to
//  sensor-only and OPEN UP THE TIME GAP to buy back the lost margin.
//  (This directly answers the jamming-attack paper in the lit survey.)
#define LINK_TIMEOUT_MS        300     // no packet for this long -> degraded
#define LINK_RECOVER_PKTS      5       // consecutive good pkts to re-trust
#define LINK_STATS_WINDOW      100     // packets per loss-rate report

// =====================================================================
//  8. NOMINAL PLANT MODEL  (used by SIM_PLANT and by the digital twin)
// =====================================================================
//  First-order speed response:  V(s)/U(s) = K / (tau*s + 1),  delay L.
//  REPLACE with the identified values from scripts/sysid_fit.py.
#define PLANT_K                0.60f   // [m/s per unit duty]
#define PLANT_TAU              0.25f   // [s]
#define PLANT_DELAY_S          0.05f   // [s] transport delay
#define PLANT_DEADZONE         0.12f   // duty below this produces no motion

// =====================================================================
//  9. SENSOR MODEL / FILTERING
// =====================================================================
//  A raw HC-SR04 differenced sample-to-sample is useless for closing rate
//  -- a single 2 cm glitch at 20 Hz reads as 0.4 m/s of closing speed and
//  triggers phantom braking. So: median-of-3 to kill spikes, then a
//  first-order low-pass, and only then differentiate.
#define ULTRA_MIN_M            0.03f
#define ULTRA_MAX_M            2.50f
#define ULTRA_MEDIAN_N         3
#define ULTRA_LPF_FC_HZ        4.0f    // range low-pass corner
#define CLOSING_LPF_FC_HZ      2.0f    // closing-rate low-pass corner
#define ULTRA_TIMEOUT_US       20000UL // ~3.4 m round trip
#define ULTRA_MAX_MISSES       5       // consecutive bad reads -> sensor fault

// =====================================================================
// 10. ENCODER
// =====================================================================
#define ENC_PPR                11.0f   // encoder pulses per motor rev (1 ch)
#define ENC_GEAR_RATIO         34.0f   // gearbox reduction
#define WHEEL_DIAMETER_M       0.065f
#define ENC_SPEED_LPF_FC_HZ    8.0f

// =====================================================================
// 11. PIN MAP  (ignored entirely when SIM_PLANT == 1)
// =====================================================================
// --- L298N motor driver
#define PIN_MOTOR_IN1          26
#define PIN_MOTOR_IN2          27
#define PIN_MOTOR_PWM          14      // L298N ENA
#define PWM_CHANNEL            0
#define PWM_FREQ_HZ            20000   // 20 kHz -> above audible range
#define PWM_RESOLUTION_BITS    10      // 0..1023
// --- quadrature encoder
#define PIN_ENC_A              34      // input-only pins, use ext. pullups
#define PIN_ENC_B              35
// --- HC-SR04 (follow node only).  ECHO IS 5V -- USE A DIVIDER.
#define PIN_ULTRA_TRIG         5
#define PIN_ULTRA_ECHO         18
// --- status LED
#define PIN_STATUS_LED         2

// =====================================================================
// 12. SIM-ONLY: lead vehicle scenario script
// =====================================================================
//  A repeatable speed profile so every demo run is identical and the
//  digital-twin comparison is meaningful. Times are seconds from boot.
#define SCENARIO_ENABLED       1
#define SCENARIO_CRUISE_MPS    0.40f
#define SCENARIO_T_START       3.0f    // begin cruising
#define SCENARIO_T_BRAKE1      12.0f   // first gentle slowdown
#define SCENARIO_T_RESUME      17.0f
#define SCENARIO_T_BRAKE_HARD  24.0f   // emergency stop -- the money shot
#define SCENARIO_T_RESTART     32.0f
#define SCENARIO_T_LOOP        42.0f   // wrap around

#endif // V2V_CONFIG_H
