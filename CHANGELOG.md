# Changelog — VibroVarioAuto

All notable changes to this project will be documented in this file.

## 1.6 (2026-05-02)

### Added
- Flight tracker: ring buffer on flash, 1 record/sec with date+time+altitude, ~18 flights of 3h
- WiFi + HTTP server export mode: AP "VibroVario", CSV of all/single flights, 15-min auto-off
- Web settings: set clock time and field altitude via browser (`/settings`)
- Sensitivity setting (0-9): controls complementary filter response speed (0=fast, 9=smooth)
- Watchdog timer (10s) on both cores — recovery from I2C lock
- Self-test on every button wake (buttons, sensors, battery)
- CRC16 integrity check on each track record

### Fixed
- CSV export date bug: each record now carries its own date (10-byte format)
- I2C writes now return bool; `initBMAMotionWake()` bails early if feature config write fails
- HTTP server timeout: `setTimeout(2000)` prevents hanging on broken connections
- Landing detection: gated on vibro cooldown, stale accel data no longer used
- Turbulence adaptation: `CFG_TAU_BARO_VARIO_TURB` raised from 0.3 to 1.0 (now has real effect)

### Changed
- Refactored code split: `config.h` + `VarioEMA.h` extracted from main .ino
- Docs: complete RU/EN pilot manuals with setup guide for Arduino IDE
- Partition uses `Default 4MB with spiffs` — 1.5 MB for track storage

### Breaking
- **Track format change**: record size 8→10→12 bytes across v1.6 dev cycle
- Magic upgraded from TRK2→TRK3→TRK4 — existing track data is erased on first boot
- `i2cWrite()` changed from `void` to `bool` return

## 1.5e (2026-04-xx)
- Initial fork from isemaster/VibroVario
- FSM with 5 states, zero static locals
- Complementary filter (gravity-vector + baro)
- Buzzer + vibration Brauneiger-style
- Settings screen (Buzzer/Vibro ON/OFF, time, altitude/QNH)
- Smart sleep (motion tracking, 15 min → 24h)
