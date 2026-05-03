/* Version 1.6 — Core 2.0.17 compat, Watchy pin cleanup
 * ESP32 Variometer firmware
 * Fork/port of original VibroVario (github.com/isemaster/VibroVario)
 * Main features:
 * - Reads BMP3XX barometer and BMA accelerometer data.
 * - Fusion (EMA) filtering for fast variometer response.
 * - E-Ink display output.
 * - Vibration/Sound lift and sink indication.
 * - Power management (Deep Sleep).
 */

#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <esp_partition.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <GxEPD2_BW.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include "FreeMonoBold36pt7b.h"
#include <cmath>

#include "config.h"
#include "VarioEMA.h"

// --- GLOBAL STATE ---
enum FsmState { FSM_CLOCK, FSM_SETTINGS, FSM_CALIBRATING, FSM_RUNNING, FSM_STOPPED, FSM_WEB_EXPORT };
RTC_DATA_ATTR FsmState fsmStateRTC = FSM_CLOCK;          // FSM state across deep sleep
RTC_DATA_ATTR unsigned long stopwatchElapsed = 0;        // Accumulated flight time
RTC_DATA_ATTR int lastWakeMinute = -1;                   // Last wake minute (for 1/min check)
RTC_DATA_ATTR float altCheck = 0.0f;                     // Last altitude for auto-start (RTC)
RTC_DATA_ATTR unsigned long motionTime = 0;              // Motion wake counter (RTC)
RTC_DATA_ATTR bool buzzerEnabled = true;                 // Buzzer enabled (RTC)
RTC_DATA_ATTR bool vibroEnabled = true;                  // Vibro enabled (RTC)
RTC_DATA_ATTR float userQNH = 1013.25f;             // User-set QNH (RTC)
RTC_DATA_ATTR int userAltM = 0;                     // User field elevation in meters (RTC)
RTC_DATA_ATTR int varioSensitivity = 4;             // 0=fastest..9=smoothest, default 4

// --- FLIGHT TRACKER ---
// Ring buffer on the SPIFFS flash partition. Each record is 8 bytes.
// At 1 record/sec, a 3h flight = 10,800 records = 86.4 KB.
// 1.5 MB partition holds ~18 flights before wrap.
// Track state survives deep sleep (RTC_DATA_ATTR).
//
// Record format (8 bytes):
//   flightNum (uint16) | secSinceMidnight (uint32) | altitudeM (int16)
//
// The partition is never formatted as SPIFFS — we use esp_partition
// directly for raw sequential writes with ring-buffer wrap.
RTC_DATA_ATTR struct {
    uint32_t magic;             // 0x54524B55 ("TRK4") — validity marker
    uint16_t flightNum;         // current flight number, auto-increments
    uint32_t writeOffset;       // next write byte offset in partition
    uint16_t lastEraseSector;   // last 4KB sector erased (prevents double-erase)
    uint32_t totalRecords;      // total records written since first boot
    uint8_t flightStartDay;     // DD of current flight (for CSV export)
    uint8_t flightStartMon;     // MM of current flight (for CSV export)
} trackState = {0, 0, 0, 0, 0, 0, 0};

// Packed binary record format (12 bytes):
// flightNum(2) | date(2) | secSinceMidnight(4) | altM(2) | crc16(2)
struct __attribute__((packed)) TrackRecord {
    uint16_t flightNum;
    uint16_t date;              // (day << 8) | mon
    uint32_t secSinceMidnight;
    int16_t altM;
    uint16_t crc;               // CRC16 of bytes 0-9 (verifies data integrity)
};

// Flight metadata for web export table
struct FlightInfo {
    uint16_t num;
    uint16_t recCount;
    uint32_t startSec;
    uint32_t endSec;
    uint8_t day;
    uint8_t mon;
};

// Variometer task state machine — replaces all static locals
struct VarioFsm {
    int pulses = 0;
    bool pulseOn = false;
    unsigned long nextT = 0;
    bool pause = false;
    bool vibroActive = false;
    unsigned long vibroStopT = 0;
    bool bzOn = false;
    unsigned long bzNextT = 0;
    unsigned long lastUpdateMicros = 0;
    float vSmooth = 0.0f;
    bool running = true;
    int bmpFailCount = 0;
    unsigned long lastActivity = 0;
    unsigned long lastLogSec = 0;
    unsigned long flightStartSec = 0;  // seconds-since-midnight at flight start
    void reset() {
        pulses = 0; pulseOn = false; nextT = 0; pause = false;
        vibroActive = false; vibroStopT = 0;
        bzOn = false; bzNextT = 0;
        lastUpdateMicros = micros();
        vSmooth = 0.0f; lastActivity = 0; lastLogSec = 0; flightStartSec = 0; running = true; bmpFailCount = 0;
    }
};

// Runtime FSM data (not RTC — reset on each wake)
struct FsmRuntime {
    FsmState state;
    bool lastBtn[4];
    int settingsRow;       // 0-5: buzzer, vibro, time, alt, wifi, sensitivity
    int editPhase;         // 0=idle, 1=editing hours/high byte, 2=editing minutes/low byte, 3=editing sensitivity
} fsm;

VarioFsm vfsm;

struct SysData {
  float startAlt, alt, vel, maxV, minV, temp;
  float ax, ay, az;
  float gMagRef;
  bool track, sensInit, accInit, bmpFail;
  unsigned long tStart, tScreen;
} data;

int rtc_s, rtc_h, rtc_m, rtc_d, rtc_mon;
Adafruit_BMP3XX bmp;
GxEPD2_BW<GxEPD2_154_D67, 200> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RES, EPD_BUSY));
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t vTaskH = NULL;

VarioEMA varioEMA;

bool showTestResult = false;                         // Flag: display test result on settings screen
String testResult;                                   // Last self-test result text

// Read altitude using user-set QNH. Wraps BMP library call.
float readAlt() { return bmp.readAltitude(userQNH); }

// Map sensitivity 0-9 to complementary filter tau (0.5s..8.0s)
float sensToTau(int sens) {
    if (sens < 0) sens = 0;
    if (sens > 9) sens = 9;
    return 0.5f + (float)sens * 0.8333f;  // sens 4 -> ~3.83s (close to default 3.0)
}

// --- HELPER FUNCTIONS ---
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t bcd2dec(uint8_t v) { return ((v/16*10) + (v%16)); }

void readRTC() {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 7);
  if(Wire.available()) {
     rtc_s = bcd2dec(Wire.read() & 0x7F);
     rtc_m = bcd2dec(Wire.read() & 0x7F);
     rtc_h = bcd2dec(Wire.read() & 0x3F);
     rtc_d = bcd2dec(Wire.read() & 0x3F);
     Wire.read(); // skip weekday
     rtc_mon = bcd2dec(Wire.read() & 0x1F);
     Wire.read(); // skip year/century
  }
}

// Set RTC alarm N seconds from now
// BCD (Binary-Coded Decimal) registers: each nibble = one decimal digit.
// Alarm registers 0x0A-0x0E: 0x80 means "don't care" for that field.
// Setting minute alarm with 0x80 bit = match only minutes, ignore seconds.
void setRTCAlarmSec(int secFromNow) {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 6);
  uint8_t s=0, m=0, h=0, d=0, wd=0, mon=0;
  if(Wire.available()) {
     s = Wire.read() & 0x7F;  // seconds
     m = Wire.read() & 0x7F;  // minutes
     h = Wire.read() & 0x3F;  // hours
     d = Wire.read() & 0x3F;  // day
     wd = Wire.read() & 0x07; // weekday
     mon = Wire.read() & 0x1F;// month
  }
  // Add offset seconds
  uint8_t dec_s = (s >> 4) * 10 + (s & 0x0F);
  uint8_t dec_m = (m >> 4) * 10 + (m & 0x0F);
  uint8_t dec_h = (h >> 4) * 10 + (h & 0x0F);
  unsigned long totalSec = dec_h * 3600UL + dec_m * 60UL + dec_s + secFromNow;
  // Handle day overflow
  if (totalSec >= 86400UL) totalSec -= 86400UL;
  uint8_t newH = totalSec / 3600;
  uint8_t newM = (totalSec % 3600) / 60;
  uint8_t newS = totalSec % 60;
  // BCD
  uint8_t bcdS = ((newS / 10) << 4) | (newS % 10);
  uint8_t bcdM = ((newM / 10) << 4) | (newM % 10);
  uint8_t bcdH = ((newH / 10) << 4) | (newH % 10);
  // Write alarm registers
  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x09); // control/status2
  Wire.write(0x02); // enable alarm, clear AF flag
  Wire.write(0x00); // 0x0A: seconds alarm (0x00 = dont care for seconds)
  Wire.write(bcdM); // 0x0B: minute alarm (0x80 = enable, match minute)
  Wire.write(bcdH); // 0x0C: hour alarm
  Wire.write(0x80); // 0x0D: day alarm (0x80 = disable)
  Wire.write(0x80); // 0x0E: weekday alarm (disable)
  Wire.endTransmission();
  // Enable alarm interrupt in control/status1
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x00); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 1);
  uint8_t cs1 = 0;
  if(Wire.available()) cs1 = Wire.read();
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x00);
  Wire.write(cs1 | 0x02); // set AIE bit
  Wire.endTransmission();
}

// Initialize BMA423 any-motion interrupt for wake from sleep
// BMA423 register 0x7C = feature config (0x00 = off, 0x04 = any-motion).
// Reg 0x40 = INT1 config (0x38 = push-pull active low).
// Reg 0x41 = INT1 mapping (0x01 = any-motion on INT1).
void initBMAMotionWake() {
  // Enable any-motion detection: config off → set INT1 → enable
  // If any I2C write fails, motion wake won't work — self-test catches it on next button wake
  if (!i2cWrite(ADDR_BMA, 0x7C, 0x00)) return;  // feature config off
  delay(5);
  i2cWrite(ADDR_BMA, 0x40, 0x38); // INT1 config
  i2cWrite(ADDR_BMA, 0x41, 0x01); // INT1 map: any-motion → INT1
  i2cWrite(ADDR_BMA, 0x7C, 0x04); // enable any-motion
  delay(10);
}

void initSensors() {
    if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        data.sensInit = true;
    }
    // Configure BMA423 accelerometer (Reset, Config, Enable)
    i2cWrite(ADDR_BMA, 0x7E, 0xB6); delay(20);
    i2cWrite(ADDR_BMA, 0x7C, 0x00); delay(10);
    i2cWrite(ADDR_BMA, 0x40, 0x28);
    i2cWrite(ADDR_BMA, 0x41, 0x01);
    i2cWrite(ADDR_BMA, 0x7D, 0x04); delay(50);
    // Verify BMA423 presence: chip ID register 0x00 should be 0x11
    Wire.beginTransmission(ADDR_BMA); Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
        Wire.requestFrom(ADDR_BMA, 1);
        data.accInit = Wire.available() && (Wire.read() == 0x11);
    }
}

void drawItem(int x, int y, const GFXfont* f, String txt) {
    display.setFont(f);
    if (x < 0) { // Center horizontally
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
        x = (display.width() - w) / 2;
    }
    display.setCursor(x, y); display.print(txt);
}

// --- VARIOMETER TASK (FreeRTOS) ---
// All state in vfsm (VarioFsm), no static locals
void varioTask(void *p) {
    esp_task_wdt_add(NULL);  // subscribe core 0 task to WDT
    for(;;) {
        if (fsmStateRTC == FSM_RUNNING && data.sensInit && vfsm.running) {
            float ax = 0.0f, ay = 0.0f, az = 0.0f;
            float acc_lin_ms2 = 0.0f;

            unsigned long nowMicros = micros();
            float dt = (nowMicros - vfsm.lastUpdateMicros) / 1000000.0f;
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 0.1f)   dt = 0.1f;
            vfsm.lastUpdateMicros = nowMicros;

            bool accSafe = !vfsm.vibroActive && (millis() - vfsm.vibroStopT > VIBRO_COOLDOWN_MS);

            if (data.accInit && accSafe) {
                Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
                Wire.requestFrom(ADDR_BMA, 6);
                if (Wire.available() >= 6) {
                    int16_t rx = Wire.read() | (Wire.read() << 8);
                    int16_t ry = Wire.read() | (Wire.read() << 8);
                    int16_t rz = Wire.read() | (Wire.read() << 8);
                    ax = rx / 8192.0f;
                    ay = ry / 8192.0f;
                    az = rz / 8192.0f;

                    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
                    float acc_linear_g = acc_mag - data.gMagRef;
                    if (fabsf(acc_linear_g) < 0.02f) acc_linear_g = 0.0f;
                    acc_lin_ms2 = acc_linear_g * GRAVITY_G;
                }
            }

            if (bmp.performReading()) {
                vfsm.bmpFailCount = 0;
                portENTER_CRITICAL(&mux); data.bmpFail = false; portEXIT_CRITICAL(&mux);
                float baro_alt = readAlt();
                float v = varioEMA.update(ax, ay, az, acc_lin_ms2, baro_alt, dt);

                portENTER_CRITICAL(&mux);
                data.alt  = varioEMA.getAltitude();
                data.vel  = v;
                data.temp = bmp.temperature;
                if (accSafe) { data.ax = ax; data.ay = ay; data.az = az; }
                portEXIT_CRITICAL(&mux);

                if (data.track) {
                    if (data.vel > data.maxV) data.maxV = data.vel;
                    if (data.vel < data.minV) data.minV = data.vel;
                }

                vfsm.vSmooth += (data.vel - vfsm.vSmooth) * 0.2f;

                // Brauneiger-style pulse generation:
                // reqP == -1: continuous vibration (hard sink)
                // reqP == 0: silence
                // reqP > 0: burst of N pulses, each V_PULSE ms on, V_GAP ms gap,
                //           V_PAUSE ms between bursts. Follows 1:1 on/off ratio.
                // --- PULSE GENERATION ---
                int reqP = 0;
                float v_check = vfsm.vSmooth;

                if (v_check <= SINK_TRH) reqP = -1;
                else {
                    for (int i = 0; i < 4; i++)
                        if (v_check >= LIFT_TH[i] && v_check < LIFT_TH[i+1])
                            reqP = LIFT_PULSES[i];
                }

                unsigned long now = millis();
                bool setVibro = false;

                if (reqP == -1) {
                    setVibro = true; vfsm.pulses = 0; vfsm.pulseOn = true; vfsm.pause = false;
                } else if (reqP == 0) {
                    setVibro = false; vfsm.pulses = 0; vfsm.pulseOn = false; vfsm.pause = false;
                } else {
                    if (vfsm.pulses == 0 && !vfsm.pulseOn && !vfsm.pause) {
                         vfsm.pulses = reqP; setVibro = true; vfsm.pulseOn = true;
                         vfsm.nextT = now + V_PULSE;
                    }
                    if (now >= vfsm.nextT) {
                        if (vfsm.pulseOn) {
                            setVibro = false; vfsm.pulseOn = false; vfsm.pulses--;
                            vfsm.nextT = now + (vfsm.pulses > 0 ? V_GAP : V_PAUSE);
                            if(vfsm.pulses <= 0) vfsm.pause = true;
                        } else {
                            if (vfsm.pause) vfsm.pause = false;
                            else { setVibro = true; vfsm.pulseOn = true; vfsm.nextT = now + V_PULSE; }
                        }
                    } else { setVibro = vfsm.pulseOn; }
                }

                // Vibro control
                if (setVibro && vibroEnabled) {
                    digitalWrite(PIN_VIBRO, 1); vfsm.vibroActive = true;
                } else {
                    digitalWrite(PIN_VIBRO, 0);
                    if (vfsm.vibroActive) vfsm.vibroStopT = millis();
                    vfsm.vibroActive = false;
                }

                // Brauneiger IQ tone: pitch proportional to climb rate,
                // beat cadence accelerates with climb. 1:1 pulse/pause ratio.
                // Sink alarm: continuous tone below BZ_SINK_ALARM (-3 m/s).
                // Silent zone: ±0.3 m/s deadband.
                // --- BUZZER ---
                unsigned long bzNow = millis();
                float bzV = vfsm.vSmooth;
                if (buzzerEnabled) {
                    if (bzV >= BZ_SILENT_MAX) {
                        float climbRate = bzV;
                        if (climbRate > 8.0f) climbRate = 8.0f;
                        int freq = BZ_CLIMB_BASE + (int)(climbRate * BZ_CLIMB_MOD);
                        if (freq > BZ_CLIMB_MAX) freq = BZ_CLIMB_MAX;
                        int beatMs = BZ_BEAT_BASE / (1.0f + (climbRate - BZ_SILENT_MAX) * BZ_BEAT_RATE);
                        if (beatMs < BZ_BEAT_MIN) beatMs = BZ_BEAT_MIN;
                        if (bzNow >= vfsm.bzNextT) {
                            vfsm.bzOn = !vfsm.bzOn;
                            vfsm.bzNextT = bzNow + beatMs / 2;
                            if (vfsm.bzOn) {
                                ledcChangeFrequency(BUZZER_CHANNEL, freq, 8);
                                ledcWrite(BUZZER_CHANNEL, 128);
                            } else {
                                ledcWrite(BUZZER_CHANNEL, 0);
                            }
                        }
                    } else if (bzV <= BZ_SILENT_MIN) {
                        float sinkRate = -bzV;
                        if (sinkRate > 5.0f) sinkRate = 5.0f;
                        int freq = BZ_SINK_FREQ;
                        if (sinkRate >= (-BZ_SINK_ALARM)) {
                            ledcChangeFrequency(BUZZER_CHANNEL, freq, 8);
                            ledcWrite(BUZZER_CHANNEL, 128);
                            vfsm.bzOn = true;
                            vfsm.bzNextT = 0;
                        } else {
                            int beatMs = 800 - (int)(sinkRate * 100);
                            if (beatMs < 200) beatMs = 200;
                            if (bzNow >= vfsm.bzNextT) {
                                vfsm.bzOn = !vfsm.bzOn;
                                vfsm.bzNextT = bzNow + beatMs / 2;
                                if (vfsm.bzOn) {
                                    ledcChangeFrequency(BUZZER_CHANNEL, freq, 8);
                                    ledcWrite(BUZZER_CHANNEL, 128);
                                } else {
                                    ledcWrite(BUZZER_CHANNEL, 0);
                                }
                            }
                        }
                    } else {
                        if (vfsm.bzOn) { ledcWrite(BUZZER_CHANNEL, 0); vfsm.bzOn = false; }
                        vfsm.bzNextT = 0;
                    }
                } else {
                    if (vfsm.bzOn) { ledcWrite(BUZZER_CHANNEL, 0); vfsm.bzOn = false; }
                    vfsm.bzNextT = 0;
                }
                // BMP fail detection
            } else {
                vfsm.bmpFailCount++;
                if (vfsm.bmpFailCount > 50) {
                    portENTER_CRITICAL(&mux); data.bmpFail = true; portEXIT_CRITICAL(&mux);
                }
            }

            // Log track record: 1 Hz, only during active flight
            unsigned long tickSec = millis() / 1000;
            if (tickSec != vfsm.lastLogSec) {
                vfsm.lastLogSec = tickSec;
                logRecord();
            }
        } else {
            digitalWrite(PIN_VIBRO, 0);
            ledcWrite(BUZZER_CHANNEL, 0);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000/LOOP_HZ));
    }
}

void drawMain() {
    float dAlt, dVel, dMax, dMin, dTemp;
    portENTER_CRITICAL(&mux);
    dAlt = data.alt; dVel = data.vel; dMax = data.maxV; dMin = data.minV; dTemp = data.temp;
    portEXIT_CRITICAL(&mux);

    char buf[40];
    display.setPartialWindow(0, 0, 200, 200);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);

        // Corner labels (same as clock screen)
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(2, 10);   display.print("ex");  // exit (BTN4)
        display.setCursor(188, 15); display.print("^");   // confirm (BTN2)
        display.setCursor(2, 196);  display.print("ok");  // down (BTN1)
        display.setCursor(188, 196);display.print("v");   // back (BTN3)

        // Flight time
        unsigned long t = stopwatchElapsed + (fsmStateRTC == FSM_RUNNING ? (millis()-data.tStart)/1000 : 0);
        sprintf(buf, "%02lu:%02lu:%02lu", t/3600, (t%3600)/60, t%60);
        drawItem(10, 20, &FreeSansBold9pt7b, buf);

        sprintf(buf, "%.1fc", dTemp); drawItem(80, 20, &FreeSansBold9pt7b, buf);

        // Battery charge
        // LiPo linear approximation: 3.3V=0%, 4.2V=100%. Inaccurate mid-range but sufficient.
        // analogReadMilliVolts returns mV, /1000 = V, *2 = voltage divider correction (/2 on PCB).
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        sprintf(buf, "%d%%", v>=4.2?100 : (v<=3.3?0 : (int)((v-3.3)*111.1)));
        drawItem(140, 20, &FreeSansBold9pt7b, buf);

        // Sensor fail warning
        if (data.bmpFail && (fsmStateRTC == FSM_RUNNING || fsmStateRTC == FSM_STOPPED)) {
            display.setTextColor(GxEPD_WHITE);
            display.fillRect(0, 40, 200, 18, GxEPD_BLACK);
            display.setCursor(10, 55);
            display.setFont(&FreeSansBold9pt7b);
            display.print("SENSOR FAIL");
            display.setTextColor(GxEPD_BLACK);
        }
        if(fsmStateRTC == FSM_RUNNING || fsmStateRTC == FSM_STOPPED) {
            drawItem(25, 50, &FreeSansBold9pt7b, "Start, m   Sea, m");
            sprintf(buf, "%+d", (int)(dAlt - data.startAlt));
            drawItem(30, 90, &FreeSansBold18pt7b, buf);
            sprintf(buf, "%d", (int)dAlt);
            drawItem(130, 90, &FreeSansBold18pt7b, buf);
        }

        drawItem(10, 135, &FreeSansBold9pt7b, "Vario");
        if(data.track) {
            sprintf(buf, "max%+.1f min%+.1f", dMax, dMin);
            drawItem(60, 135, &FreeSansBold9pt7b, buf);
        }
        sprintf(buf, "%+.1f", dVel);
        drawItem(10, 190, &FreeMonoBold36pt7b, buf);
    } while (display.nextPage());
}

void drawClock(bool fullInit) {
    if(fullInit) { display.init(115200, true, 2, false); display.setFullWindow(); }
    else display.setPartialWindow(0,0,200,200);

    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        char buf[20]; sprintf(buf, "%.2fV %d%%", v, (int)((v-3.3)*111.1));
        drawItem(10, 20, &FreeSansBold9pt7b, buf);

        sprintf(buf, "%02d:%02d", rtc_h, rtc_m);
        drawItem(-1, 110, &FreeSansBold24pt7b, buf);
        sprintf(buf, "%02d.%02d", rtc_d, rtc_mon);
        drawItem(-1, 160, &FreeSansBold18pt7b, buf);

        // Button labels in corners (tiny 9pt)
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(2, 10);   display.print("ex");  // exit
        display.setCursor(188, 15); display.print("^");   // up
        display.setCursor(2, 196);  display.print("ok");  // confirm
        display.setCursor(188, 196);display.print("v");   // down
    } while (display.nextPage());
}

void drawSettings() {
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        drawItem(-1, 25, &FreeSansBold18pt7b, "settings");

        // Corner labels (same as clock screen)
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(2, 10);   display.print("ex");  // exit
        display.setCursor(188, 15); display.print("^");   // up
        display.setCursor(2, 196);  display.print("ok");  // confirm
        display.setCursor(188, 196);display.print("v");   // down

        char buf[32];

        // Row 0: Buzzer
        display.setCursor(10, 48);
        display.print(fsm.settingsRow == 0 ? ">" : " ");
        display.print("Buzzer: ");
        display.println(buzzerEnabled ? "[ON]" : "[OFF]");

        // Row 1: Vibro
        display.setCursor(10, 68);
        display.print(fsm.settingsRow == 1 ? ">" : " ");
        display.print("Vibro:  ");
        display.println(vibroEnabled ? "[ON]" : "[OFF]");

        // Row 2: Time
        display.setCursor(10, 88);
        display.print(fsm.settingsRow == 2 ? ">" : " ");
        display.print("Time:   ");
        if (fsm.editPhase == 1) sprintf(buf, "%02d>%02d", rtc_h, rtc_m);
        else if (fsm.editPhase == 2) sprintf(buf, "%02d:%02d<", rtc_h, rtc_m);
        else sprintf(buf, "%02d:%02d", rtc_h, rtc_m);
        display.println(buf);

        // Row 3: Alt (elevation → auto QNH)
        display.setCursor(10, 108);
        display.print(fsm.settingsRow == 3 ? ">" : " ");
        display.print("Alt:    ");
        if (fsm.editPhase >= 1) sprintf(buf, "%dm_", userAltM);
        else sprintf(buf, "%dm", userAltM);
        display.println(buf);

        // Row 4: WiFi export
        display.setCursor(10, 130);
        display.print(fsm.settingsRow == 4 ? ">" : " ");
        display.print("WiFi:   ");
        display.println(webExportActive ? "[ON]" : "[OFF]");

        // Row 5: Sensitivity
        display.setCursor(10, 150);
        display.print(fsm.settingsRow == 5 ? ">" : " ");
        display.print("Sens:   ");
        if (fsm.editPhase == 3) sprintf(buf, "[%d*]", varioSensitivity);
        else sprintf(buf, "[%d]", varioSensitivity);
        display.println(buf);

        // Row 6: Self-test
        display.setCursor(10, 170);
        display.print(fsm.settingsRow == 6 ? ">" : " ");
        display.print("Test:   ");
        display.println(showTestResult ? "" : "[RUN]");

        if (showTestResult) {
            display.setCursor(80, 170);
            display.println(testResult);
        }
    } while (display.nextPage());
}

// Write hours+minutes to PCF8563 RTC in BCD format, seconds zeroed
void writeTimeToRTC(int h, int m) {
    uint8_t bcdH = ((h / 10) << 4) | (h % 10);
    uint8_t bcdM = ((m / 10) << 4) | (m % 10);
    Wire.beginTransmission(ADDR_RTC);
    Wire.write(0x02); // seconds register
    Wire.write(0x00); // seconds = 0
    Wire.write(bcdM); // minutes
    Wire.write(bcdH); // hours
    Wire.endTransmission();
}

// Read BMP pressure and compute QNH for given field elevation (meters).
// Uses standard atmosphere reverse formula: QNH = P / (1 - h/44330.769)^5.255876
void computeQNHfromAlt(int altMeters) {
    digitalWrite(PIN_VARIO_EN, 1);
    delay(10);
    if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        if (bmp.performReading()) {
            float pAbs = bmp.pressure / 100.0f; // Pa to hPa
            float ratio = 1.0f - altMeters / ISA_H;
            if (ratio > 0.01f) {
                userQNH = pAbs / powf(ratio, ISA_POW);
                if (userQNH < 950.0f) userQNH = 950.0f;
                if (userQNH > 1050.0f) userQNH = 1050.0f;
            }
            digitalWrite(PIN_VARIO_EN, 0);
            return; // success
        }
    }
    digitalWrite(PIN_VARIO_EN, 0);
    // BMP unavailable — show brief warning on e-ink
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        drawItem(-1, 80, &FreeSansBold18pt7b, "BARO UNAVAIL");
        drawItem(-1, 130, &FreeSansBold9pt7b, "QNH unchanged");
    } while (display.nextPage());
    delay(2000);
}

void startFlight() {
    digitalWrite(PIN_VARIO_EN, 1); delay(50); initSensors();
    display.setPartialWindow(0,0,200,200); display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        drawItem(20, 90, &FreeSansBold18pt7b, "Calibrating...");
    } while(display.nextPage());

    if(data.sensInit) {
        if (bmp.performReading()) {
            varioEMA.init(readAlt());
        }
        float sumMag = 0.0f;
        const int samples = 100;
        for(int i=0; i<samples; i++) {
            float _ax = 0.0f, _ay = 0.0f, _az = 0.0f;
            Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
            Wire.requestFrom(ADDR_BMA, 6);
            if(Wire.available()>=6) {
                int16_t rx = Wire.read()|(Wire.read()<<8);
                int16_t ry = Wire.read()|(Wire.read()<<8);
                int16_t rz = Wire.read()|(Wire.read()<<8);
                _ax = rx/8192.0f;
                _ay = ry/8192.0f;
                _az = rz/8192.0f;
                sumMag += sqrtf(_ax*_ax + _ay*_ay + _az*_az);
            }
            if(bmp.performReading()) {
                float rawAlt = readAlt();
                varioEMA.update(_ax, _ay, _az, 0.0f, rawAlt, 0.02f);
            }
            delay(10);
        }
        data.gMagRef = sumMag / samples;
        if(data.gMagRef < 0.5f || data.gMagRef > 1.5f) data.gMagRef = 1.0f;
        float finalAlt = varioEMA.getAltitude();
        portENTER_CRITICAL(&mux);
        data.startAlt = finalAlt;
        data.alt      = finalAlt;
        data.vel      = 0.0f;
        data.maxV     = 0.0f;
        data.minV     = 0.0f;
        portEXIT_CRITICAL(&mux);
        data.track = false;
        stopwatchElapsed = 0;
        data.tStart = millis();
        if(!vTaskH) xTaskCreatePinnedToCore(varioTask, "V", 4096, NULL, 10, &vTaskH, 0);
        vfsm.reset();
        // Apply sensitivity setting
        varioEMA.setTauComp(sensToTau(varioSensitivity));
        // Init flight tracker for this flight
        trackState.flightNum++;
        readRTC();
        trackState.flightStartDay = (uint8_t)rtc_d;
        trackState.flightStartMon = (uint8_t)rtc_mon;
        vfsm.flightStartSec = (unsigned long)rtc_h * 3600UL + (unsigned long)rtc_m * 60UL + (unsigned long)rtc_s;
        vfsm.lastLogSec = 0;
        fsm.state = FSM_RUNNING;
        fsmStateRTC = FSM_RUNNING;
    }
}

void runSelfTest() {
    // Check buttons: the wake button (UP) is expected to be pressed.
    // Only flag other buttons as stuck if they are also held.
    // Buttons are active LOW (pullup, short to GND when pressed).
    bool btnStuck[4] = { false, false, false, false };
    delay(10);  // settle after wake
    // Skip BTN_UP [2] — that's the wake button, expected to be held
    if (!digitalRead(BTN_CALIBRATE)) btnStuck[0] = true;
    if (!digitalRead(BTN_OK))   btnStuck[1] = true;
    if (!digitalRead(BTN_EDIT)) btnStuck[3] = true;

    // Check sensors
    bool baroOK = false, accOK = false;
    digitalWrite(PIN_VARIO_EN, 1);
    delay(10);
    if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
        baroOK = true;
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    }
    Wire.beginTransmission(ADDR_BMA); Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
        Wire.requestFrom(ADDR_BMA, 1);
        accOK = Wire.available() && (Wire.read() == 0x11);
    }
    digitalWrite(PIN_VARIO_EN, 0);

    // Build message
    String msg;
    if (btnStuck[0] || btnStuck[1]) {
        msg += "BTN:";
        if (btnStuck[0]) msg += " DOWN";
        if (btnStuck[1]) msg += " OK";
    }
    if (!baroOK)   { if (msg.length()) msg += " "; msg += "BARO FAIL"; }
    if (!accOK)    { if (msg.length()) msg += " "; msg += "ACCEL FAIL"; }

    // Check battery
    float battV = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
    if (battV < 3.4f) { if (msg.length()) msg += " "; msg += "LOW BATT"; }

    // Show warning if any issue found
    if (msg.length() > 0) {
        display.setFullWindow();
        display.firstPage();
        do {
            display.fillScreen(GxEPD_WHITE);
            display.setTextColor(GxEPD_BLACK);
            drawItem(-1, 30, &FreeSansBold18pt7b, "SELF-TEST");
            display.setCursor(10, 70);
            display.setFont(&FreeSansBold9pt7b);
            // Split message into lines of max ~28 chars
            int from = 0, y = 70;
            while (from < (int)msg.length()) {
                int to = msg.indexOf(' ', from + 28);
                if (to < 0 || to > from + 35) to = msg.indexOf(' ', from + 20);
                if (to < 0) to = msg.length();
                display.setCursor(10, y);
                display.print(msg.substring(from, to));
                y += 20;
                from = to + 1;
            }
            drawItem(-1, 170, &FreeSansBold9pt7b, "Check device");
        } while (display.nextPage());
        // Hold warning for 5 seconds
        unsigned long startMs = millis();
        while (millis() - startMs < 5000) { delay(10); }
    }
}

// Self-test returning result string (for settings menu display)
String runSelfTestStr() {
    bool btnStuck[3] = { false, false, false };
    delay(10);
    if (!digitalRead(BTN_CALIBRATE)) btnStuck[0] = true;
    if (!digitalRead(BTN_OK))   btnStuck[1] = true;

    bool baroOK = false, accOK = false;
    digitalWrite(PIN_VARIO_EN, 1);
    delay(10);
    if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
        baroOK = true;
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    }
    Wire.beginTransmission(ADDR_BMA); Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
        Wire.requestFrom(ADDR_BMA, 1);
        accOK = Wire.available() && (Wire.read() == 0x11);
    }
    digitalWrite(PIN_VARIO_EN, 0);

    String msg;
    if (!baroOK) msg += "BARO";
    if (!accOK)  { if (msg.length()) msg += " "; msg += "ACC"; }
    float battV = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
    if (battV < 3.4f) { if (msg.length()) msg += " "; msg += "BAT"; }
    if (msg.length() == 0) msg = "OK";
    return msg;
}

// --- FLIGHT TRACKER ---
// CRC16-CCITT (polynomial 0x8005, reflected 0xA001)
uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// Initialise the ring buffer on the SPIFFS flash partition.
// On first boot (magic mismatch), erase the entire partition.
const esp_partition_t* trackPart = NULL;

void initTracker() {
    trackPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                          ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    if (!trackPart) return;  // no partition — tracking disabled

    if (trackState.magic != 0x54524B55) {
        // First boot — erase entire partition
        esp_partition_erase_range(trackPart, 0, trackPart->size);
        trackState.magic = 0x54524B55;
        trackState.flightNum = 0;
        trackState.writeOffset = 0;
        trackState.lastEraseSector = 0;
        trackState.totalRecords = 0;
        trackState.flightStartDay = 0;
        trackState.flightStartMon = 0;
    }
}

// Write one track record to the ring buffer.
// Called from varioTask() every ~1 second during flight.
void logRecord() {
    if (!trackPart) return;

    TrackRecord rec;
    rec.flightNum = trackState.flightNum;
    rec.altM = (int16_t)roundf(data.alt);
    rec.date = ((uint16_t)trackState.flightStartDay << 8) | trackState.flightStartMon;
    // Compute CRC16 over first 10 bytes, store in last 2
    rec.crc = crc16((const uint8_t*)&rec, 10);
    // Compute seconds-since-midnight from flight start
    unsigned long elapsed = millis() / 1000;
    unsigned long secOfDay = (vfsm.flightStartSec + elapsed) % 86400UL;
    rec.secSinceMidnight = (uint32_t)secOfDay;

    // Ring buffer: wrap to 0 if not enough space
    if (trackState.writeOffset + sizeof(rec) > trackPart->size) {
        trackState.writeOffset = 0;
    }

    // Erase the 4KB sector if entering a new one
    uint16_t sector = trackState.writeOffset / 4096;
    if (sector != trackState.lastEraseSector) {
        size_t eraseOff = sector * 4096;
        size_t eraseSz = 4096;
        if (eraseOff + eraseSz > trackPart->size) {
            eraseSz = trackPart->size - eraseOff;
        }
        esp_partition_erase_range(trackPart, eraseOff, eraseSz);
        trackState.lastEraseSector = sector;
    }

    esp_partition_write(trackPart, trackState.writeOffset, &rec, sizeof(rec));
    trackState.writeOffset += sizeof(rec);
    trackState.totalRecords++;
}

// --- WEB EXPORT ---
WiFiServer webServer(80);
unsigned long webExportStartTime = 0;
bool webExportActive = false;

// Scan ring buffer and extract flight list (up to maxFlights).
// Returns number of flights found.
int scanFlights(FlightInfo *flights, int maxFlights) {
    if (!trackPart || trackState.totalRecords == 0) return 0;

    size_t partSz = trackPart->size;
    size_t recSz = sizeof(TrackRecord);
    size_t numSlots = partSz / recSz;

    size_t r1_off, r1_len, r2_off, r2_len;
    if (trackState.totalRecords <= numSlots) {
        r1_off = 0; r1_len = trackState.writeOffset;
        r2_off = 0; r2_len = 0;
    } else {
        r1_off = trackState.writeOffset;
        r1_len = partSz - trackState.writeOffset;
        r2_off = 0; r2_len = trackState.writeOffset;
    }

    int count = 0;
    uint16_t curF = 0;
    FlightInfo *cur = NULL;

    auto readRange = [&](size_t off, size_t len) {
        size_t cnt = len / recSz;
        for (size_t i = 0; i < cnt; i++) {
            TrackRecord rec;
            esp_partition_read(trackPart, off + i * recSz, &rec, recSz);
            if (rec.flightNum == 0) continue;
            // CRC check: skip corrupted records
            if (rec.crc != crc16((const uint8_t*)&rec, 10)) continue;
            if (rec.flightNum != curF) {
                curF = rec.flightNum;
                if (count < maxFlights) {
                    cur = &flights[count++];
                    cur->num = rec.flightNum;
                    cur->startSec = rec.secSinceMidnight;
                    cur->endSec = rec.secSinceMidnight;
                    cur->recCount = 1;
                    cur->day = rec.date >> 8;
                    cur->mon = rec.date & 0xFF;
                }
            } else if (cur) {
                cur->endSec = rec.secSinceMidnight;
                cur->recCount++;
            }
        }
    };

    if (r1_len) readRange(r1_off, r1_len);
    if (r2_len) readRange(r2_off, r2_len);
    return count;
}

void drawWebExport() {
    // Count flights on-screen
    FlightInfo flist[30];
    int nFlights = scanFlights(flist, 30);

    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);

        // Corner labels (same as clock screen)
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(2, 10);   display.print("ex");  // exit (BTN4/UP)
        display.setCursor(188, 15); display.print("^");   // (BTN2 — no-op)
        display.setCursor(2, 196);  display.print("ok");  // (BTN1 — no-op)
        display.setCursor(188, 196);display.print("v");   // exit (BTN3/BACK)

        drawItem(-1, 12, &FreeSansBold18pt7b, "WiFi Export");
        drawItem(-1, 38, &FreeSansBold9pt7b, "SSID: VibroVario");
        drawItem(-1, 55, &FreeSansBold9pt7b, "IP: 192.168.4.1");
        char buf[32];
        int remain = (15*60 - (int)((millis() - webExportStartTime)/1000));
        if (remain < 0) remain = 0;
        sprintf(buf, "Timer: %02d:%02d  Flights: %d", remain/60, remain%60, nFlights);
        drawItem(-1, 72, &FreeSansBold9pt7b, buf);
        drawItem(-1, 100, &FreeSansBold9pt7b, "Open browser to 192.168.4.1");
        drawItem(-1, 120, &FreeSansBold9pt7b, "/export   - CSV all flights");
        drawItem(-1, 140, &FreeSansBold9pt7b, "/settings - set time/alt");
    } while (display.nextPage());
}

void startWebExport() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("VibroVario");
    webServer.begin();
    webExportStartTime = millis();
    webExportActive = true;
    drawWebExport();
}

static void fmtTime(char* buf, uint32_t sec) {
    uint8_t h = sec / 3600;
    uint8_t m = (sec % 3600) / 60;
    uint8_t s = sec % 60;
    sprintf(buf, "%02d:%02d:%02d", h, m, s);
}

static void fmtDuration(char* buf, uint32_t startSec, uint32_t endSec) {
    if (endSec < startSec) endSec += 86400;  // midnight cross
    uint32_t d = endSec - startSec;
    sprintf(buf, "%02d:%02d:%02d", d/3600, (d%3600)/60, d%60);
}

// === HTTP HANDLERS ===

void send_http_ok(WiFiClient &c) {
    c.println("HTTP/1.1 200 OK");
    c.println("Connection: close");
}

void send_head(WiFiClient &c) {
    c.println("<!DOCTYPE html><html><head>");
    c.println("<meta charset='utf-8'><meta name='viewport' content='width=device-width'>");
    c.println("<style>body{font:14px/1.4 sans-serif;margin:20px;max-width:600px}");
    c.println("a{color:#06f}th{text-align:left;padding:2px 8px}td{padding:2px 8px}");
    c.println("input{font:14px;width:80px;margin:2px}.btn{display:inline-block;padding:6px 14px;background:#06f;color:#fff;text-decoration:none;border-radius:4px}");
    c.println("</style></head><body>");
}

void send_foot(WiFiClient &c) {
    c.println("</body></html>");
}

void handleRoot(WiFiClient &c) {
    send_http_ok(c);
    c.println("Content-Type: text/html; charset=utf-8");
    c.println();
    send_head(c);
    c.println("<h2>VibroVario</h2>");
    char buf[128];
    int remain = (15*60 - (int)((millis() - webExportStartTime)/1000));
    if (remain < 0) remain = 0;
    sprintf(buf, "<p>Active: <b>%02d:%02d</b></p>", remain/60, remain%60);
    c.println(buf);

    c.println("<p><a class='btn' href='/export'>Download all flights (CSV)</a></p>");

    // Flight table
    FlightInfo flist[30];
    int nFlights = scanFlights(flist, 30);
    if (nFlights > 0) {
        c.println("<table><tr><th>Flight</th><th>Date</th><th>Start</th><th>Duration</th><th>Points</th><th></th></tr>");
        for (int i = nFlights - 1; i >= 0; i--) {  // newest first
            FlightInfo &f = flist[i];
            int day = f.day ? f.day : rtc_d;
            int mon = f.mon ? f.mon : rtc_mon;
            char ts[9], dur[9];
            fmtTime(ts, f.startSec);
            fmtDuration(dur, f.startSec, f.endSec);
            sprintf(buf, "<tr><td>%d</td><td>%02d.%02d</td><td>%s</td><td>%s</td><td>%d</td>",
                    f.num, day, mon, ts, dur, f.recCount);
            c.println(buf);
            sprintf(buf, "<td><a href='/export?f=%d'>CSV</a></td></tr>", f.num);
            c.println(buf);
        }
        c.println("</table>");
    } else {
        c.println("<p>No flights recorded.</p>");
    }

    c.println("<hr><p><a href='/settings'>Settings: time & altitude</a></p>");
    send_foot(c);
}

void handleExport(WiFiClient &c, int flightFilter) {
    if (!trackPart || trackState.totalRecords == 0) {
        send_http_ok(c);
        c.println("Content-Type: text/plain; charset=utf-8");
        c.println();
        c.println("No track data");
        c.stop();
        return;
    }

    send_http_ok(c);
    c.println("Content-Type: text/csv; charset=utf-8");
    c.println("Content-Disposition: attachment; filename=vibrovario.csv");
    c.println();

    // Export CSV
    size_t partSz = trackPart->size;
    size_t recSz = sizeof(TrackRecord);
    size_t numSlots = partSz / recSz;

    size_t r1_off, r1_len, r2_off, r2_len;
    if (trackState.totalRecords <= numSlots) {
        r1_off = 0; r1_len = trackState.writeOffset;
        r2_off = 0; r2_len = 0;
    } else {
        r1_off = trackState.writeOffset;
        r1_len = partSz - trackState.writeOffset;
        r2_off = 0; r2_len = trackState.writeOffset;
    }

    c.println("FLT;DATE;TIME;ALT_M");
    uint16_t curFlight = 0xFFFF;

    auto sendRange = [&](size_t off, size_t len) {
        size_t cnt = len / recSz;
        for (size_t i = 0; i < cnt; i++) {
            TrackRecord rec;
            esp_partition_read(trackPart, off + i * recSz, &rec, recSz);
            if (flightFilter > 0 && rec.flightNum != (uint16_t)flightFilter) continue;
            if (rec.flightNum == 0) continue;
            // CRC check: skip corrupted records
            if (rec.crc != crc16((const uint8_t*)&rec, 10)) continue;

            // Each record carries its own date
            int day = rec.date >> 8;
            int mon = rec.date & 0xFF;
            if (day == 0 || mon == 0) { day = rtc_d; mon = rtc_mon; }

            char ts[9];
            fmtTime(ts, rec.secSinceMidnight);
            char buf[64];
            sprintf(buf, "%d;%02d.%02d;%s;%d", rec.flightNum, day, mon, ts, rec.altM);
            c.println(buf);
            if (!c) return false;
        }
        return true;
    };

    if (r1_len) if (!sendRange(r1_off, r1_len)) return;
    if (r2_len) sendRange(r2_off, r2_len);
}

void handleSettings(WiFiClient &c) {
    // Read current RTC time
    readRTC();
    send_http_ok(c);
    c.println("Content-Type: text/html; charset=utf-8");
    c.println();
    send_head(c);
    c.println("<h2>Settings</h2>");
    c.println("<form action='/set' method='get'>");
    char buf[128];

    // Time
    c.println("<h3>Clock</h3>");
    sprintf(buf, "<p>Hour: <input type='number' name='h' min='0' max='23' value='%d'></p>", rtc_h);
    c.println(buf);
    sprintf(buf, "<p>Min:  <input type='number' name='m' min='0' max='59' value='%d'></p>", rtc_m);
    c.println(buf);

    // Altitude / QNH
    c.println("<h3>Field Altitude (auto QNH)</h3>");
    sprintf(buf, "<p>Alt (m): <input type='number' name='alt' min='0' max='5000' value='%d'></p>", userAltM);
    c.println(buf);
    sprintf(buf, "<p>Current QNH: %.2f hPa</p>", userQNH);
    c.println(buf);

    c.println("<p><input type='submit' value='Save'></p>");
    c.println("</form>");
    c.println("<p><a href='/'>Back</a></p>");
    send_foot(c);
}

void handleSet(WiFiClient &c, String &query) {
    // Parse GET parameters: ?h=14&m=30 or ?alt=300 or combination
    int newH = -1, newM = -1, newAlt = -1;
    int pos;

    pos = query.indexOf("h=");
    if (pos >= 0) newH = query.substring(pos + 2).toInt();
    // Check if there's a '&' after h value
    int amp = query.indexOf('&', pos + 2);
    if (amp > 0 && newH >= 0) newH = query.substring(pos + 2, amp).toInt();

    pos = query.indexOf("m=");
    if (pos >= 0) {
        amp = query.indexOf('&', pos + 2);
        if (amp > 0) newM = query.substring(pos + 2, amp).toInt();
        else newM = query.substring(pos + 2).toInt();
    }

    pos = query.indexOf("alt=");
    if (pos >= 0) {
        amp = query.indexOf('&', pos + 4);
        if (amp > 0) newAlt = query.substring(pos + 4, amp).toInt();
        else newAlt = query.substring(pos + 4).toInt();
    }

    // Validate
    if (newH >= 0 && newH < 24 && newM >= 0 && newM < 60) {
        writeTimeToRTC(newH, newM);
        rtc_h = newH; rtc_m = newM;
    }
    if (newAlt >= 0 && newAlt <= 5000) {
        userAltM = newAlt;
        computeQNHfromAlt(newAlt);
    }

    // Redirect back to settings page
    c.println("HTTP/1.1 303 See Other");
    c.println("Location: /settings");
    c.println("Connection: close");
    c.println();
}

void handleWebClient() {
    WiFiClient client = webServer.available();
    if (!client) return;

    // Read request line with 2s timeout to prevent hanging on broken connections
    client.setTimeout(2000);
    String request = client.readStringUntil('\n');
    request.trim();

    // Consume the rest of headers
    while (client.connected()) {
        String h = client.readStringUntil('\n');
        h.trim();
        if (h.length() == 0) break;
    }

    // Parse path
    int flightFilter = -1;
    bool handled = false;

    if (request.indexOf("GET / ") == 0 || request.indexOf("GET / HTTP") == 0) {
        handleRoot(client);
        handled = true;
    } else if (request.indexOf("GET /export") == 0) {
        int fPos = request.indexOf("?f=");
        if (fPos > 0) {
            String val = request.substring(fPos + 3);
            int amp = val.indexOf(' ');
            if (amp > 0) val = val.substring(0, amp);
            flightFilter = val.toInt();
        }
        handleExport(client, flightFilter);
        handled = true;
    } else if (request.indexOf("GET /settings") == 0) {
        handleSettings(client);
        handled = true;
    } else if (request.indexOf("GET /set") == 0) {
        int qPos = request.indexOf('?');
        if (qPos > 0) {
            String query = request.substring(qPos + 1);
            int sp = query.indexOf(' ');
            if (sp > 0) query = query.substring(0, sp);
            handleSet(client, query);
        } else {
            client.println("HTTP/1.1 400 Bad Request");
            client.println("Connection: close");
            client.println();
        }
        handled = true;
    }

    if (!handled) {
        client.println("HTTP/1.1 404 Not Found");
        client.println("Connection: close");
        client.println();
    }
    client.stop();
}

void goDeepSleep() {
    fsmStateRTC = fsm.state;
    if(vTaskH) { vfsm.running = false; delay(50); vTaskDelete(vTaskH); vTaskH = NULL; }
    digitalWrite(PIN_VARIO_EN, 0); digitalWrite(PIN_VIBRO, 0);
    ledcWrite(BUZZER_CHANNEL, 0);
    ledcDetachPin(BUZZER_PIN);
    display.hibernate(); Wire.end(); WiFi.mode(WIFI_OFF);

    // Wake sources: all 4 buttons (active HIGH, default 0, pressed 1)
    const uint64_t wakeMask = BIT64(BTN_UP) | BIT64(BTN_OK) | BIT64(BTN_CALIBRATE) | BIT64(BTN_EDIT);
    esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_task_wdt_deinit();
    esp_deep_sleep_start();
}

void setup() {
    pinMode(BTN_CALIBRATE, INPUT);
    pinMode(BTN_OK, INPUT);
    pinMode(BTN_UP, INPUT);
    pinMode(BTN_EDIT, INPUT);
    pinMode(PIN_VARIO_EN, OUTPUT);
    pinMode(PIN_VIBRO, OUTPUT);
    pinMode(PIN_BATT, INPUT);
    digitalWrite(PIN_VARIO_EN, 0);
    digitalWrite(PIN_VIBRO, 0);

    // Initialize buzzer (PWM)
    ledcSetup(BUZZER_CHANNEL, BZ_CLIMB_BASE, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, 0);                   // silent at startup

    Wire.begin();
    setCpuFrequencyMhz(80);
    WiFi.mode(WIFI_OFF);

    readRTC();

    // Init watchdog: 10 second timeout, panic on expiry (Core 2.x API)
    esp_task_wdt_init(10, true);                     // timeout in seconds, panic on expiry
    esp_task_wdt_add(NULL);                          // subscribe loop() task (core 1)

    initTracker();  // init flight log ring buffer on flash

    // Restore runtime FSM state from RTC
    fsm.state = fsmStateRTC;
    fsm.settingsRow = 0; fsm.editPhase = 0;

    // Determine wake cause
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    uint64_t wakePin = 0;
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
        wakePin = esp_sleep_get_ext1_wakeup_status();
    }

    // Init display only for button wake events (active HIGH)
    bool needDisplay = (cause == ESP_SLEEP_WAKEUP_UNDEFINED) ||
                       (cause == ESP_SLEEP_WAKEUP_EXT1) ||
                       (cause != ESP_SLEEP_WAKEUP_UNDEFINED &&
                        (fsmStateRTC == FSM_RUNNING || fsmStateRTC == FSM_STOPPED));

    if (needDisplay) {
        display.init(115200, true, 2, (cause == ESP_SLEEP_WAKEUP_UNDEFINED));
        display.setRotation(1);
    }

    if (fsmStateRTC == FSM_RUNNING && cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        // Wake from sleep during flight — anomaly, fall back to CLOCK
        fsmStateRTC = FSM_CLOCK;
        fsm.state = FSM_CLOCK;
    }

    if (fsmStateRTC == FSM_CLOCK) {
        if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
            // First boot — show clock
            drawClock(true);
            fsmStateRTC = FSM_CLOCK;
            fsm.state = FSM_CLOCK;
            // Go to loop()
        }
        else if (cause == ESP_SLEEP_WAKEUP_EXT1) {
            // Wake by any button — show clock
            drawClock(true);
            fsmStateRTC = FSM_CLOCK;
            fsm.state = FSM_CLOCK;
        }
    } else if (fsmStateRTC == FSM_STOPPED) {
        drawMain();
    }

    if (fsmStateRTC == FSM_RUNNING) {
        // Restored from deep sleep in RUNNING — fall back to CLOCK
        fsm.state = FSM_CLOCK;
        fsmStateRTC = FSM_CLOCK;
        drawClock(true);
    }
}

void loop() {
    esp_task_wdt_reset();
    // === FSM TICK ===
    // Buttons: active HIGH (default LOW=0, pressed HIGH=1)
    // [0]=BTN1 (GPIO26), [1]=BTN2 (GPIO25), [2]=BTN3 (GPIO35), [3]=BTN4 (GPIO4)
    bool btn[4] = { digitalRead(BTN1), digitalRead(BTN2), digitalRead(BTN3), digitalRead(BTN4) };
    bool anyBtn = btn[0] || btn[1] || btn[2] || btn[3];
    unsigned long now = millis();

    // Rising-edge detection with 50ms debounce delay.
    bool press[4] = { false, false, false, false };
    for (int i = 0; i < 4; i++) {
        if (btn[i] && !fsm.lastBtn[i]) {
            delay(50);
            int p = (i==0 ? BTN1 : (i==1 ? BTN2 : (i==2 ? BTN3 : BTN4)));
            if (digitalRead(p)) { press[i] = true; }
        }
        fsm.lastBtn[i] = btn[i];
    }

    // FSM dispatcher
    switch (fsm.state) {

    case FSM_CLOCK: {
        // [0]=DOWN (BTN1/G26/bottom-left) → fly
        // [1]=CONFIRM (BTN2/G25/top-right) → settings
        // [2]=BACK (BTN3/G35/bottom-right) → sleep
        // [3]=UP (BTN4/G4/top-left) → no-op
        if (press[1]) {  // CONFIRM → settings
            fsm.state = FSM_SETTINGS;
            fsm.settingsRow = 0; fsm.editPhase = 0;
            drawSettings();
            return;
        }
        if (press[2]) {  // BACK → sleep
            setRTCAlarmSec(CLOCK_SLEEP_STILL);
            goDeepSleep();
        }
        if (press[0]) {  // DOWN → start flight
            fsm.state = FSM_CALIBRATING;
            startFlight();
            return;
        }
        // No button → sleep after 15s idle
        if (!anyBtn) {
            // Stay awake 15s after wake so user can press a button
            if (millis() < 15000) { return; }
            setRTCAlarmSec(CLOCK_SLEEP_STILL);
            goDeepSleep();
        } else {
            // Button was pressed but not handled (shouldn't happen) — sleep anyway
            setRTCAlarmSec(CLOCK_SLEEP_MOTION);
            goDeepSleep();
        }
        break;
    }

    case FSM_SETTINGS: {
        // Edit mode (time or QNH adjustment active)
        if (fsm.editPhase > 0) {
            // UP (press[3], BTN4/top-left) → increase
            if (press[3]) {
                if (fsm.settingsRow == 2) {
                    if (fsm.editPhase == 1) rtc_h = (rtc_h + 1) % 24;
                    else rtc_m = (rtc_m + 1) % 60;
                } else if (fsm.settingsRow == 3) {
                    userAltM += 5;
                    if (userAltM > 5000) userAltM = 5000;
                } else if (fsm.settingsRow == 5) {
                    if (varioSensitivity < 9) varioSensitivity++;
                }
                drawSettings(); return;
            }
            // DOWN (press[0], BTN1/bottom-left) → decrease
            if (press[0]) {
                if (fsm.settingsRow == 2) {
                    if (fsm.editPhase == 1) rtc_h = (rtc_h + 23) % 24;
                    else rtc_m = (rtc_m + 59) % 60;
                } else if (fsm.settingsRow == 3) {
                    userAltM -= 5;
                    if (userAltM < 0) userAltM = 0;
                } else if (fsm.settingsRow == 5) {
                    if (varioSensitivity > 0) varioSensitivity--;
                }
                drawSettings(); return;
            }
            // CONFIRM (press[1], BTN2/top-right) → save
            if (press[1]) {
                if (fsm.settingsRow == 2) {
                    if (fsm.editPhase == 1) fsm.editPhase = 2;
                    else { writeTimeToRTC(rtc_h, rtc_m); fsm.editPhase = 0; }
                } else if (fsm.settingsRow == 3) {
                    computeQNHfromAlt(userAltM);
                    fsm.editPhase = 0;
                } else {
                    fsm.editPhase = 0;
                }
                drawSettings(); return;
            }
            // BACK (press[2], BTN3/bottom-right) → cancel edit
            if (press[2]) {
                fsm.editPhase = 0;
                drawSettings(); return;
            }
            delay(50); break;
        }

        // Normal settings navigation
        if (press[1]) { // CONFIRM (BTN2/top-right) → select current row
            if (fsm.settingsRow == 0) { buzzerEnabled = !buzzerEnabled; drawSettings(); return; }
            if (fsm.settingsRow == 1) { vibroEnabled = !vibroEnabled; drawSettings(); return; }
            if (fsm.settingsRow == 2) { fsm.editPhase = 1; drawSettings(); return; }
            if (fsm.settingsRow == 3) { fsm.editPhase = 1; drawSettings(); return; }
            if (fsm.settingsRow == 4) {
                fsm.state = FSM_WEB_EXPORT;
                startWebExport();
                return;
            }
            if (fsm.settingsRow == 5) { fsm.editPhase = 3; drawSettings(); return; }
            if (fsm.settingsRow == 6) {
                // Run self-test and show result
                showTestResult = true;
                testResult = runSelfTestStr();
                drawSettings(); return;
            }
        }
        if (press[0]) { // DOWN (BTN1/bottom-left) → next row
            fsm.settingsRow = (fsm.settingsRow + 1) % 7;  // 0..6
            showTestResult = false;
            drawSettings(); return;
        }
        if (press[2]) { // BACK (BTN3/bottom-right) → back to clock
            readRTC(); drawClock(true);
            fsm.state = FSM_CLOCK;
            fsm.editPhase = 0;
            showTestResult = false;
            return;
        }
        if (press[3]) { // UP (BTN4/top-left) → previous row
            fsm.settingsRow = (fsm.settingsRow + 6) % 7;  // 0..6, wraps
            showTestResult = false;
            drawSettings(); return;
        }
        delay(50);
        break;
    }

    case FSM_CALIBRATING:
        // startFlight() handled calibration and set RUNNING.
        // If we're still here, calibration failed or was never called.
        fsm.state = FSM_CLOCK;
        drawClock(true);
        break;

    case FSM_RUNNING: {
        // RUNNING: varioTask is active on core 0, loop handles UI and auto-landing.
        // BACK (press[2], BTN3/bottom-right)=emergency stop, CONFIRM (press[1], BTN2/top-right)=pause+stats, DOWN( press[0])=ignored.
        // Track flag enables max/min recording after 5s stabilization.
        // Ensure vario task exists
        if (!vTaskH) {
            fsm.state = FSM_CLOCK;
            drawClock(true);
            return;
        }
        // BACK (press[2], BTN3/bottom-right) → stop flight, back to clock
        if (press[2]) {
            if(vTaskH) { vfsm.running = false; delay(50); vTaskDelete(vTaskH); vTaskH = NULL; }
            digitalWrite(PIN_VARIO_EN, 0);
            digitalWrite(PIN_VIBRO, 0);
            ledcWrite(BUZZER_CHANNEL, 0);
            vfsm.reset();
            readRTC(); drawClock(true);
            fsm.state = FSM_CLOCK;
            return;
        }
        // CONFIRM (press[1], BTN2/top-right) → stop flight, show stats
        if (press[1]) {
            stopwatchElapsed += (now - data.tStart) / 1000;
            if(vTaskH) { vfsm.running = false; delay(50); vTaskDelete(vTaskH); vTaskH = NULL; }
            digitalWrite(PIN_VARIO_EN, 0);
            digitalWrite(PIN_VIBRO, 0);
            ledcWrite(BUZZER_CHANNEL, 0);
            vfsm.reset();
            fsm.state = FSM_STOPPED;
            digitalWrite(PIN_VIBRO, 0);
            return;
        }
        // Auto landing detection
        if (data.track && (now - data.tStart > 10000)) {
            float vel = 0, ax = 0, ay = 0;
            portENTER_CRITICAL(&mux);
            vel = data.vel; ax = data.ax; ay = data.ay;
            portEXIT_CRITICAL(&mux);
            if (fabsf(vel) < LANDING_DETECT_VZ) {
                // Use accel for stillness check only when vibro cooldown has passed
                bool still = !vfsm.vibroActive && (now - vfsm.vibroStopT > VIBRO_COOLDOWN_MS)
                             && fabsf(ax) < 0.1f && fabsf(ay) < 0.1f;
                if (still) {
                    if (vfsm.lastActivity == 0) vfsm.lastActivity = now;
                    if ((now - vfsm.lastActivity) > (LANDING_DETECT_SEC * 1000UL)) {
                        stopwatchElapsed += (now - data.tStart) / 1000;
                        if(vTaskH) { vfsm.running = false; delay(50); vTaskDelete(vTaskH); vTaskH = NULL; }
                        digitalWrite(PIN_VARIO_EN, 0);
                        digitalWrite(PIN_VIBRO, 0);
                        ledcWrite(BUZZER_CHANNEL, 0);
                        vfsm.reset();
                        readRTC(); drawClock(true);
                        fsm.state = FSM_CLOCK;
                        return;
                    }
                } else {
                    vfsm.lastActivity = 0;
                }
            }
        }
        // Enable tracking after 5 sec
        if (!data.track && (now - data.tStart > 5000)) data.track = true;
        // Screen refresh
        if (now - data.tScreen >= REFRESH_MS) {
            data.tScreen = now; drawMain();
        }
        delay(10);
        break;
    }

    case FSM_STOPPED:
        // STOPPED: flight timer frozen, stats displayed. BACK(press[2])=clock, CONFIRM(press[1])=reset RTC, DOWN(press[0])=restart.
        // BACK (press[2], BTN3/bottom-right) → clock
        if (press[2]) {
            if(vTaskH) { vfsm.running = false; delay(50); vTaskDelete(vTaskH); vTaskH = NULL; }
            digitalWrite(PIN_VARIO_EN, 0);
            digitalWrite(PIN_VIBRO, 0);
            ledcWrite(BUZZER_CHANNEL, 0);
            vfsm.reset();
            readRTC(); drawClock(true);
            fsm.state = FSM_CLOCK;
            return;
        }
        // CONFIRM (press[1], BTN2/top-right) → reset RTC to 00:00
        if (press[1]) {
            i2cWrite(ADDR_RTC, 0x02, 0);
            i2cWrite(ADDR_RTC, 0x03, 0);
            i2cWrite(ADDR_RTC, 0x04, 0);
            rtc_h = 0; rtc_m = 0;
            drawClock(false);
            fsm.state = FSM_CLOCK;
            return;
        }
        if (press[0]) { // DOWN (BTN1/bottom-left) → new flight
            startFlight();
            return;
        }
        // Screen refresh
        if (now - data.tScreen >= REFRESH_MS) {
            data.tScreen = now; drawMain();
        }
        delay(10);
        break;

    case FSM_WEB_EXPORT: {
        handleWebClient();
        // Check 15-minute timeout
        if (webExportActive && (millis() - webExportStartTime > 15*60*1000UL)) {
            webExportActive = false;
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_OFF);
            readRTC(); drawClock(true);
            fsm.state = FSM_CLOCK;
            return;
        }
        // BACK (press[2], BTN3/bottom-right) → exit early
        // UP (press[3], BTN4/top-left) → exit early (matches "UP - exit" on screen)
        if (press[2] || press[3]) {
            webExportActive = false;
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_OFF);
            readRTC(); drawClock(true);
            fsm.state = FSM_CLOCK;
            return;
        }
        delay(10);
        break;
    }
    }
}
