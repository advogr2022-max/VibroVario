/* Версия 1.5b — English cross-reference comments, FSM transition docs
 * Прошивка ESP32 Вариометра
 * Форк/порт оригинального VibroVario (github.com/isemaster/VibroVario)
 * Основной функционал:
 * - Считывание данных с барометра BMP3XX и акселерометра (BMA).
 * - Fusion-фильтрация (EMA) данных для быстрого отклика вариометра.
 * - Вывод информации на E-Ink дисплей.
 * - Звуковая/Вибро индикация подъема и спуска.
 * - Управление питанием (Deep Sleep).
 */

#include <esp_sleep.h>
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

// --- НАСТРОЙКИ ФИЛЬТРОВ (Fusion) ---
float CFG_TAU_BARO_ALT        = 0.1f;   // Постоянная времени сглаживания высоты (сек)
float CFG_TAU_BARO_VARIO_BASE = 0.3f;   // Базовая постоянная времени вариометра (в спокойном воздухе)
float CFG_TAU_BARO_VARIO_TURB = 0.3f;   // Постоянная времени при турбулентности (динамическая настройка)
float CFG_TAU_ACCEL           = 0.1f;   // Постоянная времени фильтра акселерометра
float CFG_ACCEL_TURB_REF      = 2.0f;   // Эталонное ускорение (м/с²) для определения уровня турбулентности
float CFG_VARIO_SENS          = 1.0f;   // Масштабный коэффициент чувствительности вариометра
float CFG_TAU_COMPLEMENTARY   = 3.0f;   // Постоянная времени комплементарного фильтра (сек)
                                         // > 3c: баро доминирует, < 3c: IMU доминирует
float CFG_TAU_GRAVITY_VEC     = 2.0f;   // Постоянная времени оценки направления гравитации (сек)
                                         // Определяет "вертикаль" независимо от ориентации часов

// --- СИСТЕМНЫЕ НАСТРОЙКИ ---
#define SEALEVELPRESSURE_HPA 1013.25     // Давление на уровне моря для расчета высоты
#define REFRESH_MS 1000                 // Период обновления экрана (мс)
#define LOOP_HZ 50                      // Частота основного цикла обработки (Гц)
#define SINK_TRH -5.0f                  // Порог срабатывания сигнала снижения (м/с)

// --- НАСТРОЙКИ ЭНЕРГОСБЕРЕЖЕНИЯ ---
#define CLOCK_SLEEP_MOTION 60           // Сон с движением (сек)
#define CLOCK_SLEEP_STILL  86400        // Сон без движения (24ч)
#define FLIGHT_DETECT_VZ  0.75f          // Порог Vz для автостарта полёта (м/с)
#define FLIGHT_DETECT_SEC 3             // Сколько секунд Vz > порога для старта
#define LANDING_DETECT_VZ  0.75f         // Порог Vz для автоопределения посадки (м/с)
#define LANDING_DETECT_SEC 300          // 5 мин без движения = посадка

// --- НАСТРОЙКИ АКСЕЛЕРОМЕТРА ---
#define GRAVITY_G 9.80665f              // Ускорение свободного падения

// --- ПОРОГИ ЗВУКА И ИМПУЛЬСЫ ---
// Пороги вертикальной скорости (м/с) и соответствующее количество импульсов вибро
const float LIFT_TH[]    = {0.15f, 0.4f, 1.0f, 2.0f, 1000.0f}; 
const int   LIFT_PULSES[] = {1, 2, 3, 4};

// --- НАСТРОЙКИ ВИБРАЦИИ ---
#define V_PULSE 30                      // Длительность импульса вибрации (мс)
#define V_GAP   100                     // Пауза между импульсами в пачке (мс)
#define V_PAUSE 500                     // Длинная пауза между пачками (мс)
#define VIBRO_COOLDOWN_MS 200           // Время "остывания" после вибрации перед замером акселерометра (защита от шума)

// --- ПИНЫ (Hardware) ---
#define BTN_OK 4                        // Ниж-лев: подтверждение/стоп
#define PIN_VARIO_EN 26                 // Пин управления питанием сенсоров
#define BTN_DOWN 35                     // Ниж-прав: старт/вниз
#define BTN_UP 25                       // Верх-прав: назад к часам/вверх
#define PIN_VIBRO 13                    
#define PIN_BATT 34                     // АЦП батареи

// --- БУЗЗЕР (Brauneiger-style) ---
// Свободный пин на Watchy V2: GPIO 16, 17, или 32.
// Внимание: на V1/V1.5 GPIO 32 = UP_BTN, используй 16 или 17.
#define BUZZER_PIN     32               // Пин буззера (PWM)
#define BUZZER_CH      1                // LEDC канал

// Настройки тональности (Brauneiger IQ style)
#define BZ_SILENT_MIN  -0.3f            // Нижняя граница тихой зоны (м/с)
#define BZ_SILENT_MAX   0.3f            // Верхняя граница тихой зоны (м/с)
#define BZ_CLIMB_BASE   700              // Базовая частота подъёма (Гц)
#define BZ_CLIMB_MOD    200              // Прибавка частоты на м/с (Гц)
#define BZ_CLIMB_MAX    2200             // Макс частота подъёма (Гц)
#define BZ_SINK_FREQ    500              // Частота снижения (Гц)
#define BZ_SINK_ALARM  -3.0f             // Порог аварийного снижения (м/с)
#define BZ_BEAT_BASE    600              // Базовая длительность такта (мс) при vz=0
#define BZ_BEAT_MIN     80               // Мин длительность такта (мс)
#define BZ_BEAT_RATE    2.0f             // Коэф ускорения бипов от vz

// --- ДИСПЛЕЙ И I2C ---
#define EPD_CS 5                        
#define EPD_RES 9                       
#define EPD_DC 10                       
#define EPD_BUSY 19                     
#define ADDR_BMA 0x18                   
#define ADDR_RTC 0x51
#define ACC_INT_1_PIN 14                // BMA423 INT1 (any-motion wake)
#define RTC_INT_PIN 27                  // PCF8563 INT (alarm wake)

// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ---
enum FsmState { FSM_CLOCK, FSM_SETTINGS, FSM_CALIBRATING, FSM_RUNNING, FSM_STOPPED };
RTC_DATA_ATTR FsmState fsmStateRTC = FSM_CLOCK;          // FSM состояние через deep sleep
RTC_DATA_ATTR unsigned long stopwatchElapsed = 0;        // Накопленное время полета
RTC_DATA_ATTR int lastWakeMinute = -1;                   // Минута последнего пробуждения часов
RTC_DATA_ATTR int lastWakeDay = -1;                      // День последнего пробуждения (для 1р/день)
RTC_DATA_ATTR float altCheck = 0.0f;                     // Последняя высота для автостарта (RTC)
RTC_DATA_ATTR unsigned long motionTime = 0;              // Счётчик wake-ов с движением (RTC)
RTC_DATA_ATTR bool buzzerEnabled = true;                 // Буззер включён (RTC)
RTC_DATA_ATTR bool vibroEnabled = true;                  // Вибро включён (RTC)

// Состояние вариометрной задачи — заменяет все static-переменные
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
    unsigned long lastActivity = 0;
    void reset() {
        pulses = 0; pulseOn = false; nextT = 0; pause = false;
        vibroActive = false; vibroStopT = 0;
        bzOn = false; bzNextT = 0;
        lastUpdateMicros = micros();
        vSmooth = 0.0f; lastActivity = 0;
    }
};

// Runtime FSM данные (не RTC — сбрасываются при каждом wake)
struct FsmRuntime {
    FsmState state;
    bool lastBtn[3];
} fsm;

VarioFsm vfsm;

struct SysData {
  float startAlt, alt, vel, maxV, minV, temp;
  float ax, ay, az;
  float gMagRef;
  bool track, sensInit, accInit;
  unsigned long tStart, tScreen;
  bool lastSt[3];
} data;

int rtc_h, rtc_m, rtc_d, rtc_mon;
Adafruit_BMP3XX bmp;
GxEPD2_BW<GxEPD2_154_D67, 200> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RES, EPD_BUSY));
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t vTaskH = NULL;

// --- КЛАСС ФИЛЬТРАЦИИ ВАРИОМЕТРА (Gravity-aligned IMU + Baro) ---
// Вертикаль определяется по фильтрованному вектору ускорения (gravity vector).
// Проекция текущего ускорения на него даёт альтитудное ускорение независимо
// от ориентации часов. Комплементарный фильтр: IMU(быстрый) + Baro(точный).
// Турбулентность — по магнитуде (все оси).
// Complementary filter: gravity-vector estimation from accelerometer LPF,
// projected acceleration gives vertical acceleration regardless of watch orientation.
// Turbulence magnitude computed from all 3 axes (orientation-independent).
// Fusion: IMU (fast, drifts) high-pass + Baro (slow, accurate) low-pass.
class VarioEMA {
  float altFilt_        = 0.0f;
  float altPrev_        = 0.0f;
  float varioFilt_      = 0.0f;
  float accelLinFilt_   = 0.0f;
  float velInertial_    = 0.0f;   // Интегрированная скорость от акселерометра
  float velInertialLP_  = 0.0f;   // Low-pass для удаления дрейфа
  float varioBaroLP_    = 0.0f;   // Low-pass baro-варио

  // Gravity vector estimation (LPF on raw accel)
  float gxEst_ = 0.0f, gyEst_ = 0.0f, gzEst_ = 0.0f;  // Низкочастотная оценка гравитации (G)

  bool  inited_       = false;

  float alphaFromTau(float dt, float tau) {
      if (tau <= 0.0f) return 1.0f;
      float a = dt / (tau + dt);
      if (a < 0.0f) a = 0.0f;
      if (a > 1.0f) a = 1.0f;
      return a;
  }

public:
  void init(float initialAlt) {
      altFilt_       = initialAlt;
      altPrev_       = initialAlt;
      varioFilt_     = 0.0f;
      accelLinFilt_  = 0.0f;
      velInertial_   = 0.0f;
      velInertialLP_ = 0.0f;
      varioBaroLP_   = 0.0f;
      gxEst_ = 0.0f; gyEst_ = 0.0f; gzEst_ = 0.0f;
      inited_        = true;
  }

  // Основной метод обновления фильтра.
  // ax_raw, ay_raw, az_raw — сырые показания акселерометра (единицы: G, ±8g диапазон)
  // accelMagMs2            — магнитуда линейного ускорения (м/с²), для турбулентности
  // baroAlt                — сырая высота с барометра
  float update(float ax_raw, float ay_raw, float az_raw,
               float accelMagMs2, float baroAlt, float dt) {
      if (dt < 0.001f) dt = 0.001f;
      if (dt > 0.1f)   dt = 0.1f;

      if (!inited_) {
          init(baroAlt);
      }

      // 1. Оценка направления гравитации (LPF вектора акселерометра)
      //    При любом положении часов этот вектор указывает "вниз"
      float aGrav = alphaFromTau(dt, CFG_TAU_GRAVITY_VEC);
      gxEst_ += aGrav * (ax_raw - gxEst_);
      gyEst_ += aGrav * (ay_raw - gyEst_);
      gzEst_ += aGrav * (az_raw - gzEst_);

      // Нормализация gravity vector
      float gNorm = sqrtf(gxEst_*gxEst_ + gyEst_*gyEst_ + gzEst_*gzEst_);
      if (gNorm > 0.001f) {
          float invG = 1.0f / gNorm;
          gxEst_ *= invG;
          gyEst_ *= invG;
          gzEst_ *= invG;
      } else {
          gxEst_ = 0.0f; gyEst_ = 0.0f; gzEst_ = 1.0f;
      }

      // 2. Проекция текущего ускорения на гравитацию => линейное ускорение по вертикали
      float accelVerticalG = ax_raw*gxEst_ + ay_raw*gyEst_ + az_raw*gzEst_ - 1.0f;
      if (fabsf(accelVerticalG) < 0.02f) accelVerticalG = 0.0f; // deadzone
      float azMs2 = accelVerticalG * GRAVITY_G;

      // 3. Фильтрация магнитуды ускорения (для турбулентности)
      float aAcc = alphaFromTau(dt, CFG_TAU_ACCEL);
      accelLinFilt_ += aAcc * (accelMagMs2 - accelLinFilt_);

      // 4. Фильтрация высоты
      float aAlt = alphaFromTau(dt, CFG_TAU_BARO_ALT);
      altFilt_ += aAlt * (baroAlt - altFilt_);

      // 5. Baro-варио (производная высоты)
      float varioRaw = (altFilt_ - altPrev_) / dt;
      altPrev_ = altFilt_;

      // 6. Адаптация tau варио по турбулентности
      float turb = fabsf(accelLinFilt_);
      float turbNorm = 0.0f;
      if (CFG_ACCEL_TURB_REF > 0.0f) {
          turbNorm = turb / CFG_ACCEL_TURB_REF;
          if (turbNorm > 1.0f) turbNorm = 1.0f;
      }
      float tauVario = CFG_TAU_BARO_VARIO_BASE + turbNorm * CFG_TAU_BARO_VARIO_TURB;
      float aVario = alphaFromTau(dt, tauVario);

      // 7. Low-pass baro-варио (для комплементарного фильтра)
      varioBaroLP_ += aVario * (varioRaw - varioBaroLP_);

      // 8. Комплементарный фильтр: IMU + Baro
      velInertial_ += azMs2 * dt;                    // Интеграция вертикального ускорения

      float aComp = alphaFromTau(dt, CFG_TAU_COMPLEMENTARY);
      velInertialLP_ += aComp * (velInertial_ - velInertialLP_);  // Дрейф

      float velInertialHP = velInertial_ - velInertialLP_;        // HP = без дрейфа

      varioFilt_ = velInertialHP + varioBaroLP_;                  // Fusion

      if (varioFilt_ >  25.0f) varioFilt_ =  25.0f;
      if (varioFilt_ < -25.0f) varioFilt_ = -25.0f;

      return varioFilt_ * CFG_VARIO_SENS;
  }

  float getAltitude() const { return altFilt_; }
  float getVario()    const { return varioFilt_ * CFG_VARIO_SENS; }
  float getAccelLin() const { return accelLinFilt_; }

  // Доступ к оценке гравитации (для отладки)
  void getGravityEst(float &gx, float &gy, float &gz) const {
      gx = gxEst_; gy = gyEst_; gz = gzEst_;
  }
};

VarioEMA varioEMA;

// --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t bcd2dec(uint8_t v) { return ((v/16*10) + (v%16)); }

void readRTC() {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 6);
  if(Wire.available()) {
     Wire.read(); // skip seconds
     rtc_m = bcd2dec(Wire.read()&0x7F);
     rtc_h = bcd2dec(Wire.read()&0x3F);
     rtc_d = bcd2dec(Wire.read()&0x3F);
     Wire.read(); // skip
     rtc_mon = bcd2dec(Wire.read()&0x1F);
  }
}

// Установка RTC alarm на +N секунд от текущего времени
// BCD (Binary-Coded Decimal) registers: each nibble = one decimal digit.
// Alarm registers 0x0A-0x0E: 0x80 means "don't care" for that field.
// Setting minute alarm with 0x80 bit = match only minutes, ignore seconds.
void setRTCAlarmSec(int secFromNow) {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 6);
  uint8_t s=0, m=0, h=0, d=0, wd=0, mon=0;
  if(Wire.available()) {
     s = Wire.read() & 0x7F;  // секунды
     m = Wire.read() & 0x7F;  // минуты
     h = Wire.read() & 0x3F;  // часы
     d = Wire.read() & 0x3F;  // день
     wd = Wire.read() & 0x07; // день недели
     mon = Wire.read() & 0x1F;// месяц
  }
  // Добавляем секунды
  uint8_t dec_s = (s >> 4) * 10 + (s & 0x0F);
  uint8_t dec_m = (m >> 4) * 10 + (m & 0x0F);
  uint8_t dec_h = (h >> 4) * 10 + (h & 0x0F);
  unsigned long totalSec = dec_h * 3600UL + dec_m * 60UL + dec_s + secFromNow;
  // Обработка переполнения через сутки
  if (totalSec >= 86400UL) totalSec -= 86400UL;
  uint8_t newH = totalSec / 3600;
  uint8_t newM = (totalSec % 3600) / 60;
  uint8_t newS = totalSec % 60;
  // BCD
  uint8_t bcdS = ((newS / 10) << 4) | (newS % 10);
  uint8_t bcdM = ((newM / 10) << 4) | (newM % 10);
  uint8_t bcdH = ((newH / 10) << 4) | (newH % 10);
  // Записываем alarm
  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x09); // control/status2
  Wire.write(0x02); // enable alarm, clear AF flag
  Wire.write(0x00); // 0x0A: seconds alarm (0x00 = dont care for seconds)
  Wire.write(bcdM); // 0x0B: minute alarm (0x80 = enable, match minute)
  Wire.write(bcdH); // 0x0C: hour alarm
  Wire.write(0x80); // 0x0D: day alarm (0x80 = disable)
  Wire.write(0x80); // 0x0E: weekday alarm (disable)
  Wire.endTransmission();
  // Включаем alarm interrupt в control/status1
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x00); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 1);
  uint8_t cs1 = 0;
  if(Wire.available()) cs1 = Wire.read();
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x00);
  Wire.write(cs1 | 0x02); // set AIE bit
  Wire.endTransmission();
}

// Инициализация BMA423 any-motion interrupt для пробуждения из сна
// BMA423 register 0x7C = feature config (0x00 = off, 0x04 = any-motion).
// Reg 0x40 = INT1 config (0x38 = push-pull active low).
// Reg 0x41 = INT1 mapping (0x01 = any-motion on INT1).
void initBMAMotionWake() {
  // Устанавливаем any-motion детекцию
  i2cWrite(ADDR_BMA, 0x7C, 0x00); // feature config off
  delay(5);
  // Конфиг any-motion через feature config
  i2cWrite(ADDR_BMA, 0x40, 0x38); // INT1: active low, push-pull
  i2cWrite(ADDR_BMA, 0x41, 0x01); // INT1 map: any-motion → INT1
  // Включаем any-motion detection
  i2cWrite(ADDR_BMA, 0x7C, 0x04);
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
    // Настройка акселерометра BMA (Reset, Config, Enable)
    i2cWrite(ADDR_BMA, 0x7E, 0xB6); delay(20);
    i2cWrite(ADDR_BMA, 0x7C, 0x00); delay(10);
    i2cWrite(ADDR_BMA, 0x40, 0x28);
    i2cWrite(ADDR_BMA, 0x41, 0x01);
    i2cWrite(ADDR_BMA, 0x7D, 0x04); delay(50);
    data.accInit = true;
}

void drawItem(int x, int y, const GFXfont* f, String txt) {
    display.setFont(f);
    if (x < 0) { // Центрирование по горизонтали
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
        x = (display.width() - w) / 2;
    }
    display.setCursor(x, y); display.print(txt);
}

// --- ЗАДАЧА ВАРИОМЕТРА (FreeRTOS Task) ---
// Все состояния хранятся в vfsm (VarioFsm), не в static locals
void varioTask(void *p) {
    for(;;) {
        if (fsmStateRTC == FSM_RUNNING && data.sensInit) {
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
                float baro_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
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
                // --- ЛОГИКА ГЕНЕРАЦИИ ИМПУЛЬСОВ ---
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

                // Управление вибро
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
                // --- БУЗЗЕР ---
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
                                ledcChangeFrequency(BUZZER_PIN, freq, 8);
                                ledcWrite(BUZZER_PIN, 128);
                            } else {
                                ledcWrite(BUZZER_PIN, 0);
                            }
                        }
                    } else if (bzV <= BZ_SILENT_MIN) {
                        float sinkRate = -bzV;
                        if (sinkRate > 5.0f) sinkRate = 5.0f;
                        int freq = BZ_SINK_FREQ;
                        if (sinkRate >= (-BZ_SINK_ALARM)) {
                            ledcChangeFrequency(BUZZER_PIN, freq, 8);
                            ledcWrite(BUZZER_PIN, 128);
                            vfsm.bzOn = true;
                            vfsm.bzNextT = 0;
                        } else {
                            int beatMs = 800 - (int)(sinkRate * 100);
                            if (beatMs < 200) beatMs = 200;
                            if (bzNow >= vfsm.bzNextT) {
                                vfsm.bzOn = !vfsm.bzOn;
                                vfsm.bzNextT = bzNow + beatMs / 2;
                                if (vfsm.bzOn) {
                                    ledcChangeFrequency(BUZZER_PIN, freq, 8);
                                    ledcWrite(BUZZER_PIN, 128);
                                } else {
                                    ledcWrite(BUZZER_PIN, 0);
                                }
                            }
                        }
                    } else {
                        if (vfsm.bzOn) { ledcWrite(BUZZER_PIN, 0); vfsm.bzOn = false; }
                        vfsm.bzNextT = 0;
                    }
                } else {
                    if (vfsm.bzOn) { ledcWrite(BUZZER_PIN, 0); vfsm.bzOn = false; }
                    vfsm.bzNextT = 0;
                }
            }
        } else {
            digitalWrite(PIN_VIBRO, 0);
            ledcWrite(BUZZER_PIN, 0);
        }

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

        // Время полета
        unsigned long t = stopwatchElapsed + (fsmStateRTC == FSM_RUNNING ? (millis()-data.tStart)/1000 : 0);
        sprintf(buf, "%02lu:%02lu:%02lu", t/3600, (t%3600)/60, t%60);
        drawItem(10, 20, &FreeSansBold9pt7b, buf);

        sprintf(buf, "%.1fc", dTemp); drawItem(80, 20, &FreeSansBold9pt7b, buf);

        // Заряд батареи
        // LiPo linear approximation: 3.3V=0%, 4.2V=100%. Inaccurate mid-range but sufficient.
        // analogReadMilliVolts returns mV, /1000 = V, *2 = voltage divider correction (/2 on PCB).
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        sprintf(buf, "%d%%", v>=4.2?100 : (v<=3.3?0 : (int)((v-3.3)*111.1)));
        drawItem(140, 20, &FreeSansBold9pt7b, buf);

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

void drawClock(bool deep) {
    if(deep) { display.init(115200, true, 2, false); display.setFullWindow(); }
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
    } while (display.nextPage());
}

void drawSettings() {
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        drawItem(-1, 20, &FreeSansBold18pt7b, "Settings");

        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(10, 60);
        display.print("Buzzer: ");
        display.print(buzzerEnabled ? "[ON]" : "[OFF]");

        display.setCursor(10, 90);
        display.print("Vibro: ");
        display.print(vibroEnabled ? "[ON]" : "[OFF]");

        drawItem(-1, 150, &FreeSansBold9pt7b, "OK-toggle  UP-exit");
    } while (display.nextPage());
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
            varioEMA.init(bmp.readAltitude(SEALEVELPRESSURE_HPA));
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
                float rawAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
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
        fsm.state = FSM_RUNNING;
        fsmStateRTC = FSM_RUNNING;
    }
}

void goDeepSleep() {
    fsmStateRTC = fsm.state;
    if(vTaskH) vTaskDelete(vTaskH);
    digitalWrite(PIN_VARIO_EN, 0); digitalWrite(PIN_VIBRO, 0);
    ledcWrite(BUZZER_PIN, 0);
    ledcDetach(BUZZER_PIN);
    display.hibernate(); Wire.end(); WiFi.mode(WIFI_OFF);

    // Пробуждение: кнопка BACK (GPIO25), RTC alarm (GPIO27), BMA motion (GPIO14)
    // Все активны при LOW (кнопка → GND, RTC INT → open-drain, BMA INT → active low)
    // EXT1 wake sources: BTN_UP, RTC alarm, BMA motion.
    // All active LOW. ESP_EXT1_WAKEUP_ALL_LOW = all selected pins must be LOW.
    // BTN_OK and BTN_DOWN cannot wake the device from deep sleep.
    const uint64_t wakeMask = BIT64(BTN_UP) | BIT64(RTC_INT_PIN) | BIT64(ACC_INT_1_PIN);
    esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
}

void setup() {
    pinMode(BTN_DOWN, INPUT);
    pinMode(BTN_OK, INPUT);
    pinMode(BTN_UP, INPUT);
    pinMode(PIN_VARIO_EN, OUTPUT);
    pinMode(PIN_VIBRO, OUTPUT);
    pinMode(PIN_BATT, INPUT);
    pinMode(ACC_INT_1_PIN, INPUT);  // BMA motion interrupt
    pinMode(RTC_INT_PIN, INPUT);    // RTC alarm interrupt
    digitalWrite(PIN_VARIO_EN, 0);
    digitalWrite(PIN_VIBRO, 0);

    // Инициализация буззера (PWM) — API ESP32 Core 3.x
    ledcAttach(BUZZER_PIN, BZ_CLIMB_BASE, 8);  // канал не нужен, пин сам становится PWM
    ledcWrite(BUZZER_PIN, 0);                   // беззвучно

    Wire.begin();
    display.init(115200, true, 2, false);
    display.setRotation(1);
    setCpuFrequencyMhz(80);
    WiFi.mode(WIFI_OFF);

    readRTC();

    // Восстанавливаем runtime FSM state из RTC
    fsm.state = fsmStateRTC;

    // Определяем причину пробуждения
    // Wake reason determines behavior:
    // - ESP_SLEEP_WAKEUP_UNDEFINED = first boot → show clock
    // - BTN_UP wake → user wants to see clock
    // - RTC_INT_PIN → timed check: 1 BMP read for flight detection
    // - ACC_INT_1_PIN → motion wake → show clock
    // FSM_RUNNING on wake = anomaly (deep sleep shouldn't happen mid-flight)
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    uint64_t wakePin = 0;
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
        wakePin = esp_sleep_get_ext1_wakeup_status();
    }

    if (fsmStateRTC == FSM_RUNNING && cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        // Проснулись во время полёта — нештатная ситуация, идём в CLOCK
        fsmStateRTC = FSM_CLOCK;
        fsm.state = FSM_CLOCK;
    }

    if (fsmStateRTC == FSM_CLOCK) {
        if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
            // Первый запуск после подачи питания — показываем часы
            drawClock(true);
            fsmStateRTC = FSM_CLOCK;
            fsm.state = FSM_CLOCK;
            // Идём в loop()
        }
        else if (cause == ESP_SLEEP_WAKEUP_EXT1) {
            if (wakePin & BIT64(BTN_UP)) {
                // Пробуждение по кнопке UP — показываем часы
                drawClock(true);
                fsmStateRTC = FSM_CLOCK;
                fsm.state = FSM_CLOCK;
            }
            else if (wakePin & BIT64(RTC_INT_PIN)) {
                // Пробуждение по RTC alarm (раз в минуту)
                readRTC();
                if (rtc_m != lastWakeMinute || lastWakeMinute < 0) {
                    lastWakeMinute = rtc_m;
                    drawClock(false);
                }
                // Быстрая проверка: 1 чтение BMP
                digitalWrite(PIN_VARIO_EN, 1);
                delay(10);
                if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
                    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
                    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
                    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
                    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
                    if (bmp.performReading()) {
                        float altNow = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                        float vz = (altNow - altCheck) / 60.0f;
                        altCheck = altNow;
                        if (vz > FLIGHT_DETECT_VZ) {
                            display.setPartialWindow(0,0,200,200);
                            display.firstPage();
                            do {
                                display.fillScreen(GxEPD_WHITE);
                                display.setTextColor(GxEPD_BLACK);
                                drawItem(-1, 90, &FreeSansBold18pt7b, "FLIGHT!");
                                drawItem(-1, 130, &FreeSansBold9pt7b, "Press DOWN");
                            } while(display.nextPage());
                        }
                    }
                    digitalWrite(PIN_VARIO_EN, 0);
                } else {
                    digitalWrite(PIN_VARIO_EN, 0);
                }
                fsmStateRTC = FSM_CLOCK;
                fsm.state = FSM_CLOCK;
            }
            else if (wakePin & BIT64(ACC_INT_1_PIN)) {
                // Пробуждение по движению
                drawClock(true);
                fsmStateRTC = FSM_CLOCK;
                fsm.state = FSM_CLOCK;
            }
        }
    } else if (fsmStateRTC == FSM_STOPPED) {
        drawMain();
    }

    if (fsmStateRTC == FSM_RUNNING) {
        // Восстановление после глубокого сна в RUNNING — переходим в CLOCK
        fsm.state = FSM_CLOCK;
        fsmStateRTC = FSM_CLOCK;
        drawClock(true);
    }
}

void loop() {
    // === FSM TICK ===
    bool btn[3] = { digitalRead(BTN_DOWN), digitalRead(BTN_OK), digitalRead(BTN_UP) };
    bool anyBtn = btn[0] || btn[1] || btn[2];
    unsigned long now = millis();

    // Rising-edge detection with 50ms debounce delay.
    // fsm.lastBtn stores previous state for edge comparison.
    // Index mapping: [0]=DOWN, [1]=OK, [2]=UP.
    // Debounce: detection on rising edge
    bool press[3] = { false, false, false };
    for (int i = 0; i < 3; i++) {
        if (btn[i] && !fsm.lastBtn[i]) {
            delay(50);
            if (digitalRead(i == 0 ? BTN_DOWN : (i == 1 ? BTN_OK : BTN_UP))) {
                press[i] = true;
            }
        }
        fsm.lastBtn[i] = btn[i];
    }

    // FSM dispatcher
    switch (fsm.state) {

    case FSM_CLOCK: {
        // CLOCK state: device is awake briefly between deep sleep cycles.
        // Buttons checked first (DOWN→settings, OK→flight, UP→sleep).
        // If no button: motion tracking + RTC alarm timing → goDeepSleep().
        // DOWN → settings
        if (press[0]) {
            fsm.state = FSM_SETTINGS;
            drawSettings();
            return;
        }
        // OK → start flight
        if (press[1]) {
            fsm.state = FSM_CALIBRATING;
            startFlight();
            return;
        }
        // UP → deep sleep (24h)
        if (press[2]) {
            setRTCAlarmSec(CLOCK_SLEEP_STILL);
            initBMAMotionWake();
            goDeepSleep();
        }
        // No button → motion logic + deep sleep
        if (!anyBtn) {
            bool hadMotion = false;
            uint64_t pin = 0;
            esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
            if (cause == ESP_SLEEP_WAKEUP_EXT1) {
                pin = esp_sleep_get_ext1_wakeup_status();
                hadMotion = (pin & BIT64(ACC_INT_1_PIN));
            }
            if (lastWakeMinute < 0) motionTime = 0;
            if (hadMotion) motionTime++;
            else motionTime = 0;
            if (cause == ESP_SLEEP_WAKEUP_EXT1 && (pin & (BIT64(BTN_OK) | BIT64(BTN_UP)))) {
                altCheck = 0.0f;
            }
            bool stillEnough = (motionTime >= 15) || (!hadMotion && lastWakeMinute >= 0);
            int sleepSec = stillEnough ? CLOCK_SLEEP_STILL : CLOCK_SLEEP_MOTION;
            setRTCAlarmSec(sleepSec);
            initBMAMotionWake();
            goDeepSleep();
        } else {
            // Button was pressed but not handled (shouldn't happen) — sleep anyway
            setRTCAlarmSec(CLOCK_SLEEP_MOTION);
            initBMAMotionWake();
            goDeepSleep();
        }
        break;
    }

    case FSM_SETTINGS:
        if (press[1]) { // OK — toggle
            if (!buzzerEnabled || !vibroEnabled) {
                if (!buzzerEnabled) buzzerEnabled = true;
                else if (!vibroEnabled) vibroEnabled = true;
            } else {
                buzzerEnabled = false;
            }
            drawSettings();
            return;
        }
        if (press[0]) { // DOWN — reset all to ON
            buzzerEnabled = true; vibroEnabled = true;
            drawSettings();
            return;
        }
        if (press[2]) { // UP — exit to clock
            readRTC(); drawClock(true);
            fsm.state = FSM_CLOCK;
            return;
        }
        delay(50);
        break;

    case FSM_CALIBRATING:
        // startFlight() handled calibration and set RUNNING.
        // If we're still here, calibration failed or was never called.
        fsm.state = FSM_CLOCK;
        drawClock(true);
        break;

    case FSM_RUNNING: {
        // RUNNING: varioTask is active on core 0, loop handles UI and auto-landing.
        // UP = emergency stop, OK = pause+stats, DOWN = ignored.
        // Track flag enables max/min recording after 5s stabilization.
        // Ensure vario task exists
        if (!vTaskH) {
            fsm.state = FSM_CLOCK;
            drawClock(true);
            return;
        }
        // UP → stop flight, back to clock
        if (press[2]) {
            if(vTaskH) { vTaskDelete(vTaskH); vTaskH = NULL; }
            digitalWrite(PIN_VARIO_EN, 0);
            digitalWrite(PIN_VIBRO, 0);
            ledcWrite(BUZZER_PIN, 0);
            vfsm.reset();
            readRTC(); drawClock(true);
            fsm.state = FSM_CLOCK;
            return;
        }
        // OK → stop flight, show stats
        if (press[1]) {
            stopwatchElapsed += (now - data.tStart) / 1000;
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
            if (fabsf(vel) < LANDING_DETECT_VZ && fabsf(ax) < 0.1f && fabsf(ay) < 0.1f) {
                if (vfsm.lastActivity == 0) vfsm.lastActivity = now;
                if ((now - vfsm.lastActivity) > (LANDING_DETECT_SEC * 1000UL)) {
                    stopwatchElapsed += (now - data.tStart) / 1000;
                    if(vTaskH) { vTaskDelete(vTaskH); vTaskH = NULL; }
                    digitalWrite(PIN_VARIO_EN, 0);
                    digitalWrite(PIN_VIBRO, 0);
                    ledcWrite(BUZZER_PIN, 0);
                    vfsm.reset();
                    readRTC(); drawClock(true);
                    fsm.state = FSM_CLOCK;
                    return;
                }
            } else {
                vfsm.lastActivity = 0;
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
        // STOPPED: flight timer frozen, stats displayed. UP=exit, OK=reset RTC, DOWN=restart.
        if (press[2]) { // UP → clock
            if(vTaskH) { vTaskDelete(vTaskH); vTaskH = NULL; }
            digitalWrite(PIN_VARIO_EN, 0);
            digitalWrite(PIN_VIBRO, 0);
            ledcWrite(BUZZER_PIN, 0);
            vfsm.reset();
            readRTC(); drawClock(true);
            fsm.state = FSM_CLOCK;
            return;
        }
        if (press[1]) { // OK — reset RTC to 00:00
            i2cWrite(ADDR_RTC, 0x02, 0);
            i2cWrite(ADDR_RTC, 0x03, 0);
            i2cWrite(ADDR_RTC, 0x04, 0);
            rtc_h = 0; rtc_m = 0;
            drawClock(false);
            fsm.state = FSM_CLOCK;
            return;
        }
        if (press[0]) { // DOWN → new flight
            startFlight();
            return;
        }
        // Screen refresh
        if (now - data.tScreen >= REFRESH_MS) {
            data.tScreen = now; drawMain();
        }
        delay(10);
        break;
    }
}
