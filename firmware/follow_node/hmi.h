// >>> GENERATED FILE -- DO NOT EDIT <<<
// Source of truth: firmware/common/hmi.h
// Regenerate with: python tools/sync_common.py
// =====================================================================
//  hmi.h  --  On-node human interface: OLED dash + WS2812 mode bar.
//  MASTER COPY in firmware/common/. Run tools/sync_common.py to propagate.
//
//  DESIGN RULE: this file must never be able to break the demo.
//
//  Both HMI_ENABLE_OLED and HMI_ENABLE_LEDS default to 0, so the firmware
//  compiles and runs with no extra libraries installed and nothing wired.
//  Every function below becomes an empty inline when its feature is off, so
//  the call sites in the sketches need no #ifdefs and the compiler removes
//  the calls entirely.
//
//  That matters because the HMI is the FIRST thing on the cut list (see
//  docs/DEMO_PLAN.md section 6). An hour before the review you can turn the
//  display off with one #define and the control system is untouched.
//
//  LIBRARIES (install via Arduino Library Manager only if you enable these)
//    HMI_ENABLE_OLED -> "Adafruit SSD1306" + "Adafruit GFX Library"
//    HMI_ENABLE_LEDS -> "Adafruit NeoPixel"
// =====================================================================
#ifndef V2V_HMI_H
#define V2V_HMI_H

#include <Arduino.h>
#include "config.h"

#if HMI_ENABLE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  static Adafruit_SSD1306 hmi_oled(OLED_W, OLED_H, &Wire, -1);
  static bool hmi_oled_ok = false;
#endif

#if HMI_ENABLE_LEDS
  #include <Adafruit_NeoPixel.h>
  static Adafruit_NeoPixel hmi_bar(LED_BAR_COUNT, PIN_LED_BAR,
                                   NEO_GRB + NEO_KHZ800);
#endif

// ---------------------------------------------------------------------
//  Mode-bar colours. Deliberately chosen to be readable across a room and
//  distinguishable to the most common forms of colour blindness -- the
//  amber/red pair differs in brightness as well as hue, and the positions
//  are fixed so position alone carries the meaning.
//
//    LED 0  FOLLOW      green
//    LED 1  PREDICT     amber
//    LED 2  EMERG       red
//    LED 3  V2V OK      blue
//    LED 4  V2V LOST    magenta
// ---------------------------------------------------------------------
#define HMI_LED_FOLLOW    0
#define HMI_LED_PREDICT   1
#define HMI_LED_EMERG     2
#define HMI_LED_V2VOK     3
#define HMI_LED_V2VLOST   4

// =====================================================================
static inline void hmi_begin(const char *title) {
#if HMI_ENABLE_OLED
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  hmi_oled_ok = hmi_oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  if (hmi_oled_ok) {
    hmi_oled.clearDisplay();
    hmi_oled.setTextColor(SSD1306_WHITE);
    hmi_oled.setTextSize(1);
    hmi_oled.setCursor(0, 0);
    hmi_oled.println(title);
    hmi_oled.println("starting...");
    hmi_oled.display();
  } else {
    // Not fatal. A missing display must never stop the control loop.
    Serial.println("# WARN: OLED not found at the configured address");
  }
#else
  (void)title;
#endif

#if HMI_ENABLE_LEDS
  hmi_bar.begin();
  hmi_bar.setBrightness(LED_BAR_BRIGHTNESS);
  // brief self-test sweep so a dead strip is obvious at boot
  for (uint16_t i = 0; i < LED_BAR_COUNT; ++i) {
    hmi_bar.clear();
    hmi_bar.setPixelColor(i, hmi_bar.Color(80, 80, 80));
    hmi_bar.show();
    delay(60);
  }
  hmi_bar.clear();
  hmi_bar.show();
#endif
}

// ---------------------------------------------------------------------
//  Mode bar. `mode` uses the follower's Mode enum values, `link` its
//  LinkState values. Passed as ints so this header does not need to know
//  about the sketch's enums.
// ---------------------------------------------------------------------
static inline void hmi_set_mode_bar(int mode, int link, bool braking) {
#if HMI_ENABLE_LEDS
  hmi_bar.clear();

  // --- mode lamp (mode values: 0 STANDBY 1 CRUISE 2 FOLLOW
  //                             3 PREDICT 4 EMERG 5 FAULT)
  switch (mode) {
    case 2:  // FOLLOW
    case 1:  // CRUISE -- same lamp, it is still normal operation
      hmi_bar.setPixelColor(HMI_LED_FOLLOW, hmi_bar.Color(0, 200, 40));
      break;
    case 3:  // PREDICT
      hmi_bar.setPixelColor(HMI_LED_PREDICT, hmi_bar.Color(255, 140, 0));
      break;
    case 4:  // EMERG
    case 5:  // FAULT -- red too; from the outside both mean "stop now"
      hmi_bar.setPixelColor(HMI_LED_EMERG, hmi_bar.Color(255, 0, 0));
      break;
    default: // STANDBY -- dim green, system alive but not moving
      hmi_bar.setPixelColor(HMI_LED_FOLLOW, hmi_bar.Color(0, 30, 8));
      break;
  }

  // --- link lamp
  if (link == 0) {
    hmi_bar.setPixelColor(HMI_LED_V2VOK, hmi_bar.Color(0, 80, 255));
  } else if (link == 1) {
    // DEGRADED -- blink the OK lamp rather than add a sixth LED
    if ((millis() / 250) % 2) {
      hmi_bar.setPixelColor(HMI_LED_V2VOK, hmi_bar.Color(0, 80, 255));
    }
  } else {
    hmi_bar.setPixelColor(HMI_LED_V2VLOST, hmi_bar.Color(255, 0, 160));
  }

  // --- braking makes the whole bar flash its mode lamp brighter
  if (braking && (millis() / 150) % 2) {
    hmi_bar.setPixelColor(HMI_LED_EMERG, hmi_bar.Color(255, 60, 60));
  }

  hmi_bar.show();
#else
  (void)mode; (void)link; (void)braking;
#endif
}

// ---------------------------------------------------------------------
//  Follower dash. Four lines, big enough to read at arm's length.
// ---------------------------------------------------------------------
static inline void hmi_draw_follower(const char *mode_name,
                                     const char *link_name,
                                     const char *scn_name,
                                     float gap_m, float gap_des_m,
                                     float speed, float lead_speed,
                                     float ttc, uint32_t pkt_lost) {
#if HMI_ENABLE_OLED
  if (!hmi_oled_ok) return;
  hmi_oled.clearDisplay();

  hmi_oled.setTextSize(1);
  hmi_oled.setCursor(0, 0);
  hmi_oled.print("MODE ");
  hmi_oled.print(mode_name);
  hmi_oled.setCursor(0, 10);
  hmi_oled.print("V2V  ");
  hmi_oled.print(link_name);
  hmi_oled.print("  ls:");
  hmi_oled.print(pkt_lost);

  // gap, in the biggest text that fits -- it is the number people look at
  hmi_oled.setTextSize(2);
  hmi_oled.setCursor(0, 22);
  hmi_oled.print(gap_m * 100.0f, 0);
  hmi_oled.setTextSize(1);
  hmi_oled.print(" cm  want ");
  hmi_oled.print(gap_des_m * 100.0f, 0);

  hmi_oled.setCursor(0, 42);
  hmi_oled.print("v ");
  hmi_oled.print(speed, 2);
  hmi_oled.print("  lead ");
  hmi_oled.print(lead_speed, 2);

  hmi_oled.setCursor(0, 54);
  if (ttc < 90.0f) {
    hmi_oled.print("TTC ");
    hmi_oled.print(ttc, 1);
    hmi_oled.print("s  ");
  }
  hmi_oled.print(scn_name);

  hmi_oled.display();
#else
  (void)mode_name; (void)link_name; (void)scn_name; (void)gap_m;
  (void)gap_des_m; (void)speed; (void)lead_speed; (void)ttc; (void)pkt_lost;
#endif
}

// ---------------------------------------------------------------------
//  Lead console dash.
// ---------------------------------------------------------------------
static inline void hmi_draw_lead(const char *scn_name, bool running,
                                 float elapsed_s, float throttle,
                                 float speed, float cmd_speed,
                                 bool braking, bool tx_enabled) {
#if HMI_ENABLE_OLED
  if (!hmi_oled_ok) return;
  hmi_oled.clearDisplay();

  hmi_oled.setTextSize(1);
  hmi_oled.setCursor(0, 0);
  hmi_oled.print("SCN  ");
  hmi_oled.print(scn_name);

  hmi_oled.setCursor(0, 10);
  hmi_oled.print(running ? "RUN  t=" : "READY  ");
  if (running) hmi_oled.print(elapsed_s, 1);

  hmi_oled.setTextSize(2);
  hmi_oled.setCursor(0, 22);
  hmi_oled.print(speed, 2);
  hmi_oled.setTextSize(1);
  hmi_oled.print(" m/s  cmd ");
  hmi_oled.print(cmd_speed, 2);

  hmi_oled.setCursor(0, 42);
  hmi_oled.print("throttle ");
  hmi_oled.print(throttle * 100.0f, 0);
  hmi_oled.print("%");

  hmi_oled.setCursor(0, 54);
  if (!tx_enabled)   hmi_oled.print("** RADIO SILENT **");
  else if (braking)  hmi_oled.print("** BRAKING **");
  else               hmi_oled.print("broadcasting");

  hmi_oled.display();
#else
  (void)scn_name; (void)running; (void)elapsed_s; (void)throttle;
  (void)speed; (void)cmd_speed; (void)braking; (void)tx_enabled;
#endif
}

// Lead brake lights: reuse the mode bar pins if a strip is fitted.
static inline void hmi_set_brake_lights(bool on) {
#if HMI_ENABLE_LEDS
  hmi_bar.clear();
  if (on) {
    for (uint16_t i = 0; i < LED_BAR_COUNT; ++i)
      hmi_bar.setPixelColor(i, hmi_bar.Color(255, 0, 0));
  } else {
    for (uint16_t i = 0; i < LED_BAR_COUNT; ++i)
      hmi_bar.setPixelColor(i, hmi_bar.Color(20, 4, 0));
  }
  hmi_bar.show();
#else
  (void)on;
#endif
}

#endif // V2V_HMI_H
