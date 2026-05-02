// VibroVarioAuto v1.6 — Complementary Filter Class
// Gravity-aligned IMU + Baro fusion variometer filter.
// Include from main sketch: #include "VarioEMA.h"

#ifndef VARIOEMA_H
#define VARIOEMA_H

#include <cmath>
#include "config.h"

class VarioEMA {
  float altFilt_        = 0.0f;
  float altPrev_        = 0.0f;
  float varioFilt_      = 0.0f;
  float accelLinFilt_   = 0.0f;
  float velInertial_    = 0.0f;   // Integrated accelerometer velocity
  float velInertialLP_  = 0.0f;   // Low-pass for drift removal
  float varioBaroLP_    = 0.0f;   // Low-pass baro vario

  float tauComp_        = 3.0f;   // Complementary tau (runtime-adjustable via setTauComp)

  // Gravity vector estimation (LPF on raw accel)
  float gxEst_ = 0.0f, gyEst_ = 0.0f, gzEst_ = 0.0f;

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
      tauComp_       = 3.0f;
      inited_        = true;
  }

  // Set complementary filter time constant (sensitivity control)
  void setTauComp(float t) { if (t > 0.1f && t < 20.0f) tauComp_ = t; }

  // Main filter update.
  // ax_raw, ay_raw, az_raw — raw accelerometer (G, ±8g range)
  // accelMagMs2 — linear acceleration magnitude (m/s²), for turbulence
  // baroAlt — raw barometer altitude
  // Returns filtered vertical speed (m/s, positive = climb)
  float update(float ax_raw, float ay_raw, float az_raw,
               float accelMagMs2, float baroAlt, float dt) {
      if (dt < 0.001f) dt = 0.001f;
      if (dt > 0.1f)   dt = 0.1f;

      if (!inited_) {
          init(baroAlt);
      }

      // 1. Gravity direction estimation (LPF of accelerometer vector)
      float aGrav = alphaFromTau(dt, CFG_TAU_GRAVITY_VEC);
      gxEst_ += aGrav * (ax_raw - gxEst_);
      gyEst_ += aGrav * (ay_raw - gyEst_);
      gzEst_ += aGrav * (az_raw - gzEst_);

      float gNorm = sqrtf(gxEst_*gxEst_ + gyEst_*gyEst_ + gzEst_*gzEst_);
      if (gNorm > 0.001f) {
          float invG = 1.0f / gNorm;
          gxEst_ *= invG;
          gyEst_ *= invG;
          gzEst_ *= invG;
      } else {
          gxEst_ = 0.0f; gyEst_ = 0.0f; gzEst_ = 1.0f;
      }

      // 2. Project acceleration onto gravity => vertical linear acceleration
      float accelVerticalG = ax_raw*gxEst_ + ay_raw*gyEst_ + az_raw*gzEst_ - 1.0f;
      if (fabsf(accelVerticalG) < 0.02f) accelVerticalG = 0.0f;
      float azMs2 = accelVerticalG * GRAVITY_G;

      // 3. Filter acceleration magnitude (for turbulence)
      float aAcc = alphaFromTau(dt, CFG_TAU_ACCEL);
      accelLinFilt_ += aAcc * (accelMagMs2 - accelLinFilt_);

      // 4. Altitude filtering
      float aAlt = alphaFromTau(dt, CFG_TAU_BARO_ALT);
      altFilt_ += aAlt * (baroAlt - altFilt_);

      // 5. Baro vario (altitude derivative)
      float varioRaw = (altFilt_ - altPrev_) / dt;
      altPrev_ = altFilt_;

      // 6. Adapt vario tau based on turbulence
      float turb = fabsf(accelLinFilt_);
      float turbNorm = 0.0f;
      if (CFG_ACCEL_TURB_REF > 0.0f) {
          turbNorm = turb / CFG_ACCEL_TURB_REF;
          if (turbNorm > 1.0f) turbNorm = 1.0f;
      }
      float tauVario = CFG_TAU_BARO_VARIO_BASE + turbNorm * CFG_TAU_BARO_VARIO_TURB;
      float aVario = alphaFromTau(dt, tauVario);

      // 7. Low-pass baro vario (for complementary filter)
      varioBaroLP_ += aVario * (varioRaw - varioBaroLP_);

      // 8. Complementary filter: IMU + Baro
      velInertial_ += azMs2 * dt;

      float aComp = alphaFromTau(dt, tauComp_);
      velInertialLP_ += aComp * (velInertial_ - velInertialLP_);

      float velInertialHP = velInertial_ - velInertialLP_;

      varioFilt_ = velInertialHP + varioBaroLP_;

      if (varioFilt_ >  25.0f) varioFilt_ =  25.0f;
      if (varioFilt_ < -25.0f) varioFilt_ = -25.0f;

      return varioFilt_ * CFG_VARIO_SENS;
  }

  float getAltitude() const { return altFilt_; }
  float getVario()    const { return varioFilt_ * CFG_VARIO_SENS; }
  float getAccelLin() const { return accelLinFilt_; }

  void getGravityEst(float &gx, float &gy, float &gz) const {
      gx = gxEst_; gy = gyEst_; gz = gzEst_;
  }
};

#endif // VARIOEMA_H
