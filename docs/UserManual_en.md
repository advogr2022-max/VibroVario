# VibroVarioAuto v1.6 — Pilot Manual

**Firmware version:** 1.6
**Fork of:** [VibroVario](https://github.com/isemaster/VibroVario) by isemaster

## Differences from parent project

| Area | VibroVario (original) | VibroVarioAuto v1.6 (fork) |
|------|----------------------|-----------------------------|
| Version | 1.1 (EMA filter) | 1.6 (FSM + tracker + WiFi) |
| Lines of code | ~549 | ~1370 |
| Sound | Vibration only | Vibration + Brauneiger-style buzzer |
| Filter | Simple EMA smoothing | Complementary: gravity-vector + baro |
| Architecture | Linear code | Finite state machine (6 states) |
| Dispatcher | `if (state == X)` | `switch(fsm.state)` — table-driven |
| Hidden state | `static` locals | `struct VarioFsm` — zero static |
| Buttons | BACK/SELECT/RIGHT | UP/OK/DOWN — any wakes |
| Wake source | Only BACK (GPIO 25) | Any button (UP/OK/DOWN) |
| Settings | None | On-device screen + **web browser** |
| Self-test | None | SELF-TEST on button wake |
| BMP failure handling | Silent freeze | SENSOR FAIL overlay |
| Accelerometer | Magnitude only | Chip ID check + fallback |
| Task shutdown | Immediate vTaskDelete | Flag + delay — safe I2C |
| Sleep logic | Fixed | Smart: motionTime, 15 min → 24h |
| Watchdog timer | None | 10s WDT on both cores |
| Flight logging | None | Ring buffer on flash (1.5MB, ~18 flights) |
| WiFi export | None | AP VibroVario + HTTP: CSV export |
| Config style | `#define` | `constexpr` |
| Code language | Russian | English (100%) |
| Sensitivity | Fixed | Adjustable 0-9, affects filter response |
| Flight logging | None | Ring buffer with per-record date in CSV |

---

**License:** MIT

---

## 1. What It Is

VibroVarioAuto is a barometric variometer for paragliders, hang gliders, and sailplanes, built on the Watchy smartwatch platform (ESP32 + E-Ink display).

### Key Features

- **Silent mode** — wrist vibration motor, no beeping
- **Sound mode** — optional Brauneiger-style buzzer
- **E-Ink screen** — readable in direct sunlight, zero power when idle
- **Power saving** — weeks in clock mode, 10+ hours in flight
- **Smart sleep** — auto-wakes every minute to check for launch
- **Flight logging** — time + altitude written to flash, 1 Hz
- **WiFi export** — built-in web server, download CSV of any flight
- **Web settings** — set clock time and field altitude via browser
- **Weight** — 24 g with battery

---

## 2. Hardware Sensors

| Sensor | Measures | Purpose |
|--------|----------|---------|
| BMP390 | Air pressure | Altitude and vertical speed |
| BMA423 | Acceleration (3-axis) | Fast filter response + tilt compensation |
| PCF8563 | Precise time | RTC, survives deep sleep |
| Vibration motor | — | Tactile feedback on wrist |

### How Sensors Work Together

**Barometer (BMP390)** samples at 50 Hz. Altitude is computed from pressure using standard atmosphere formula (adjustable QNH).

**Accelerometer (BMA423)** provides two benefits:
1. **Gravity direction** — regardless of watch orientation, the gravity vector is tracked via LPF (2s time constant)
2. **Fast response** — acceleration projected onto gravity gives instant vertical acceleration before baro catches up

**Fusion**: Complementary filter — IMU (fast, drifts) high-pass + Baro (slow, accurate) low-pass.

---

## 3. Screen Reference

### Clock Screen (CLOCK mode)

```
┌──────────────────────┐
│  3.95V   87%         │    ← battery voltage and charge %
│                      │
│       14:32          │    ← current time (large)
│                      │
│      01.05           │    ← date (day.month)
│                      │
└──────────────────────┘
```

### Flight Screen (RUNNING mode)

```
┌──────────────────────┐
│ 00:12:34  24.5c  87% │    ← flight time | °C | battery
│                      │
│ Start, m    Sea, m   │    ← headers
│    +245       589    │    ← climb from start | altitude MSL
│                      │
│ Vario  max+3.2 min-1.1│   ← flight statistics
│        +1.7          │    ← VERTICAL SPEED (large)
└──────────────────────┘
```

### Settings Screen

```
┌──────────────────────┐
│      Settings        │
│                      │
│ > Buzzer: [ON]       │
│   Vibro: [OFF]       │
│   Time:   14:32      │
│   Alt:    300m       │
│   WiFi:   [OFF]
│                      │
│ UP-exit  DOWN-next   │
│          OK-act      │
└──────────────────────┘
```

### WiFi Export Screen

```
┌──────────────────────┐
│    WiFi Export       │
│                      │
│ SSID: VibroVario     │
│ IP: 192.168.4.1      │
│ Timer: 14:32  Flights:5│
│                      │
│ 192.168.4.1 in browser│
│ UP - exit            │
└──────────────────────┘
```

---

## 4. Operating Modes

The device has 6 states:

```
DEEP SLEEP  ──►  CLOCK  ──►  SETTINGS  ──►  CALIBRATING  ──►  RUNNING  ──►  STOPPED
                     │
                     └──►  WEB_EXPORT  (WiFi AP, 15 min)
```

### DEEP SLEEP
Consumption: **~5 µA**. Screen blank, all sensors off. Wake by any button. Self-test runs on wake.

### CLOCK — Clock Mode
Consumption: **~30 µA**. Shows time, date, battery. Wakes for ~0.1s every 60s for pressure check. Buttons: DOWN → Settings, OK → Start flight, UP → Deep sleep.

### WEB_EXPORT — WiFi Export Mode
Consumption: **~80 mA** (WiFi AP). Auto-off after 15 minutes. Press UP to exit early.

**Web interface at http://192.168.4.1/:**
- Flight table: number, date, start time, duration, points, CSV download link
- `/export` — download all flights as CSV
- `/export?f=3` — download flight #3 only
- `/settings` — set clock time and field altitude

### RUNNING — Flight Mode
Consumption: **~7 mA**. All sensors active. Display updates 1x/sec. Flight logging at 1 Hz. **Auto-landing:** 5 min idle → CLOCK.

### STOPPED — Flight Paused
All readings frozen. UP → CLOCK, DOWN → new flight.

---

## 5. Button Reference

| Button | GPIO | Position | CLOCK | RUNNING | SETTINGS | WEB_EXPORT |
|--------|------|----------|-------|---------|----------|-----------|
| SETUP | 26 | Top-left | — (sensor power) | — | — | — |
| UP | 25 | Top-right | Deep sleep | Exit to CLOCK | Exit to CLOCK | Exit WiFi → CLOCK |
| OK | 4 | Bottom-left | Start flight | Stop flight | Toggle / Edit | — |
| DOWN | 35 | Bottom-right | Settings | — | Next row | — |

**Wake from deep sleep:** Any button (UP/OK/DOWN). SETUP does not wake.

---

## 6. Flight Workflow

### Before Flight
1. Check battery > 50%
2. Mount watch on wrist or harness
3. Verify vibro/buzzer works
4. Check Settings (DOWN): time, altitude for your field

### Launch
**Manual:** OK → "Calibrating..." (2s still) → flight.
**Auto:** device detects climb → "FLIGHT!" → OK/DOWN.

### In Flight
- **Buzzer**: pitch proportional to climb rate
- **Vibration**: 1-4 pulses depending on lift strength
- **Screen**: altitude, vario, flight time
- **Tracking**: runs automatically, 1 record/second

### Landing
- **Manual:** OK (stop) → review stats → UP (clock)
- **Auto:** 5 min idle → auto-stop → clock

### After Flight — Data Export
1. Settings (DOWN) → WiFi → OK
2. Phone/laptop → WiFi → connect to `VibroVario`
3. Browser → `http://192.168.4.1/`
4. Flight table → download CSV (all or single flight)
5. Press UP on watch to turn off WiFi

---

## 7. Flight Logging

- **Rate**: 1 record/sec during flight
- **Format**: flight number + time + altitude (8 bytes/record)
- **Capacity**: ~18 flights of 3 hours (1.5 MB ring buffer)
- **Overwrite**: oldest flight is overwritten when full
- **Export**: WiFi AP → browser → CSV

Each CSV record: `FLT;DATE;TIME;ALT_M`

---

## 8. Settings

### On-Device (DOWN in CLOCK)

| Row | Parameter | OK action |
|-----|-----------|----------|
| 0 | Buzzer: ON/OFF | Toggle |
| 1 | Vibro: ON/OFF | Toggle |
| 2 | Time: HH:MM | Enter edit (UP/DOWN change, OK next digit) |
| 3 | Alt: 300m | Enter edit (UP/DOWN +5/-5, OK save + compute QNH) |
| 4 | WiFi: [OFF] | Start WiFi export mode |
| 5 | Sens: [4] | Enter edit (UP/DOWN 0..9, OK save) |

Settings persist across deep sleep (stored in RTC memory).

### Sensitivity (Sens: 0-9)

Controls how fast the variometer responds to vertical movements.

| Value | Response | Use case |
|-------|----------|----------|
| 0-2 | **Fast** — reacts instantly, more noise | Active acro, speed flying, quick thermal detection |
| 3-5 | **Balanced** (default 4) — clean signal, good response | General paragliding, cross-country |
| 6-9 | **Slow** — smooth, ignores small bumps, slight lag | Soaring, thermalling in light air, sensitive pilots |

Internally adjusts the complementary filter crossover: lower values let the accelerometer dominate for faster response; higher values let the barometer dominate for smoother output.

### Via Browser

In WiFi export mode: `http://192.168.4.1/settings`
- Set clock hour/minute
- Set field elevation (auto QNH calculation)
- Current QNH displayed

---

## 9. Self-Test

On every button wake, the device checks:
- Buttons DOWN, OK → `BTN: DOWN OK` if stuck
- BMP390 → `BARO FAIL`
- BMA423 → `ACCEL FAIL`
- Battery (< 3.4V) → `LOW BATT`
- In-flight BMP → `SENSOR FAIL`

On error: SELF-TEST screen for 5 seconds, then clock.

---

## 10. Power Consumption

| Mode | Current | Runtime |
|------|---------|---------|
| DEEP SLEEP | 5 µA | Months |
| CLOCK | 30 µA | Weeks |
| RUNNING | 7 mA | 10+ h |
| RUNNING + buzzer | ~10 mA | 8+ h |
| WEB_EXPORT | ~80 mA | 15 min (auto-off) |

Battery: 200 mAh LiPo (3.7V). 4.2V = 100%, 3.3V = 0%.

---

## 11. Setup & Compilation (for DIY builders)

### Project Structure

```
VibroVario/                    ← sketch folder
├── VibroVario.ino             ← main file (open in Arduino IDE)
├── config.h                   ← pins, constants, filter settings
├── VarioEMA.h                 ← complementary filter class
├── README.md                  ← project description
└── docs/
    ├── UserManual_ru.md       ← Russian manual
    └── UserManual_en.md       ← this manual
```

### Prerequisites

1. **Arduino IDE** (download from arduino.cc) — version 2.x or 1.8+
2. **ESP32 board support** — File → Preferences → Additional boards manager URLs, add:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. **Install ESP32** — Tools → Board → Boards Manager → search "ESP32" → Install
4. **Libraries** — Sketch → Include Library → Manage Libraries:
   - `GxEPD2` (latest version)
   - `Adafruit BMP3XX`
   - `Adafruit Unified Sensor`
5. **Connect Watchy** via USB
6. **Select board** — Tools → Board → ESP32 Arduino → ESP32 Dev Module
7. **Board settings:**
   - Flash Size: `4MB (32Mb)`
   - Partition Scheme: `Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)`
8. **Open sketch** — File → Open → select the `VibroVario` folder → `VibroVario.ino` opens
9. **Click Upload** (→)

`config.h` and `VarioEMA.h` are auto-detected — no manual file adding needed.

### CLI Compilation (advanced)

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "GxEPD2"
arduino-cli lib install "Adafruit BMP3XX"
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

---

## 12. Troubleshooting

**Q: Screen blank, nothing happens.**
A: Press any button (UP/OK/DOWN). If no response — connect USB.

**Q: Device locked up.**
A: Watchdog timer will reset it in 10 seconds. If not — connect USB.

**Q: Variometer shows nonsense.**
A: UP → CLOCK → OK → calibrate (2s still).

**Q: SENSOR FAIL on screen.**
A: BMP390 failure. Check connections. If temporary — will clear on next read.

**Q: How do I export track data?**
A: Settings → WiFi → OK → connect to VibroVario → browser 192.168.4.1/

**Q: Battery drains fast in WiFi mode.**
A: Normal — WiFi consumes ~80 mA. Auto-off after 15 minutes.

---

## 13. Credits

Original project: [github.com/isemaster/VibroVario](https://github.com/isemaster/VibroVario)
Original by: isemaster
Fork by: advogr2022-max
License: MIT

*Fly safe!*
