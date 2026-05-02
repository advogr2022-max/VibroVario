// VibroVarioAuto v1.6 — Configuration
// All hardware pins, filter parameters, and system constants.
// Include from main sketch: #include "config.h"

#ifndef CONFIG_H
#define CONFIG_H

// --- FILTER CONFIG ---
constexpr float CFG_TAU_BARO_ALT        = 0.1f;   // Altitude smoothing time constant (s)
constexpr float CFG_TAU_BARO_VARIO_BASE = 0.3f;   // Base vario time constant (calm air)
constexpr float CFG_TAU_BARO_VARIO_TURB = 1.0f;   // Vario time constant in turbulence
constexpr float CFG_TAU_ACCEL           = 0.1f;   // Accelerometer filter time constant
constexpr float CFG_ACCEL_TURB_REF      = 2.0f;   // Reference accel (m/s²) for turbulence level
constexpr float CFG_VARIO_SENS          = 1.0f;   // Vario sensitivity scaling factor
constexpr float CFG_TAU_COMPLEMENTARY   = 3.0f;   // Complementary filter time constant (s)
                                                    // > 3s: baro dominates, < 3s: IMU dominates
constexpr float CFG_TAU_GRAVITY_VEC     = 2.0f;   // Gravity vector LPF time constant (s)
                                                    // Determines "down" regardless of watch orientation

// --- SYSTEM CONFIG ---
constexpr unsigned long REFRESH_MS = 1000;         // Display refresh period (ms)
constexpr int LOOP_HZ = 50;                        // Main processing loop frequency (Hz)
constexpr float SINK_TRH = -5.0f;                  // Hard sink alarm threshold (m/s)
constexpr float ISA_H = 44330.769f;                // Standard atmosphere scale height
constexpr float ISA_POW = 5.255876f;               // Standard atmosphere exponent

// --- POWER SAVE ---
constexpr int CLOCK_SLEEP_MOTION = 60;             // Sleep with motion (s)
constexpr int CLOCK_SLEEP_STILL = 86400;           // Sleep without motion (24h)
constexpr float FLIGHT_DETECT_VZ = 0.75f;          // Vz threshold for auto-flight start (m/s)
constexpr float LANDING_DETECT_VZ = 0.75f;         // Vz threshold for landing detection (m/s)
constexpr int LANDING_DETECT_SEC = 300;            // 5 min idle before auto-landing

// --- ACCELEROMETER CONFIG ---
constexpr float GRAVITY_G = 9.80665f;              // Standard gravity

// --- VIBRO PULSE THRESHOLDS ---
// Vertical speed thresholds (m/s) and corresponding vibration pulse count per burst
const float LIFT_TH[]    = {0.15f, 0.4f, 1.0f, 2.0f, 1000.0f};
const int   LIFT_PULSES[] = {1, 2, 3, 4};

// --- VIBRATION TIMING ---
constexpr int V_PULSE = 30;                        // Single vibration pulse duration (ms)
constexpr int V_GAP    = 100;                      // Gap between pulses in a burst (ms)
constexpr int V_PAUSE  = 500;                      // Long pause between bursts (ms)
constexpr int VIBRO_COOLDOWN_MS = 200;             // Cooldown after vibration before accel read

// --- GPIO PINS ---
// Button 1: GPIO 26 = OK (confirm/enter) — was PIN_VARIO_EN
// Button 2: GPIO 25 = UP (navigation up)
// Button 3: GPIO  4 = EDIT (change value)
// Button 4: GPIO 35 = DOWN (navigation down)
constexpr int BTN_OK = 26;                          // Button 1: confirm/enter
constexpr int BTN_EDIT = 4;                         // Button 3: change value
constexpr int BTN_UP = 25;                          // Button 2: navigation up
constexpr int BTN_DOWN = 35;                        // Button 4: navigation down
constexpr int PIN_VARIO_EN = 33;                   // Dummy pin — BMP not connected
constexpr int PIN_VIBRO = 13;                      // Vibration motor pin
constexpr int PIN_BATT = 34;                       // Battery ADC pin

// --- BUZZER (Brauneiger-style) ---
constexpr int BUZZER_PIN = 32;                     // Buzzer PWM pin
constexpr uint8_t BUZZER_CHANNEL = 0;              // LEDC channel (Core 2.x: channel != pin)

// Tone settings (Brauneiger IQ style)
constexpr float BZ_SILENT_MIN = -0.3f;             // Silent zone lower bound (m/s)
constexpr float BZ_SILENT_MAX = 0.3f;              // Silent zone upper bound (m/s)
constexpr int BZ_CLIMB_BASE = 700;                 // Climb base frequency (Hz)
constexpr int BZ_CLIMB_MOD = 200;                  // Frequency increase per m/s (Hz)
constexpr int BZ_CLIMB_MAX = 2200;                 // Maximum climb frequency (Hz)
constexpr int BZ_SINK_FREQ = 500;                  // Sink frequency (Hz)
constexpr float BZ_SINK_ALARM = -3.0f;             // Emergency sink alarm threshold (m/s)
constexpr int BZ_BEAT_BASE = 600;                  // Base beat duration (ms) at Vz=0
constexpr int BZ_BEAT_MIN = 80;                    // Minimum beat duration (ms)
constexpr float BZ_BEAT_RATE = 2.0f;               // Beat acceleration factor vs Vz

// --- DISPLAY & I2C ---
constexpr int EPD_CS = 5;                          // E-Ink chip select
constexpr int EPD_RES = 9;                         // E-Ink reset
constexpr int EPD_DC = 10;                         // E-Ink data/command
constexpr int EPD_BUSY = 19;                       // E-Ink busy
constexpr uint8_t ADDR_BMA = 0x18;                 // BMA423 I2C address
constexpr uint8_t ADDR_RTC = 0x51;                 // PCF8563 RTC I2C address
// ACC_INT_1_PIN (14) and RTC_INT_PIN (27) — defined in Watchy variant pins_arduino.h

#endif // CONFIG_H
