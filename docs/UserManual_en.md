# VibroVarioAuto — Pilot Manual

**Firmware version:** 1.5e  
**Fork of:** [VibroVario](https://github.com/isemaster/VibroVario) by isemaster  

## Differences from parent project

| Area | VibroVario (original) | VibroVarioAuto (fork) |
|------|----------------------|----------------------|
| Version | 1.1 (EMA filter) | 1.5e (FSM) |
| Lines of code | ~549 | ~1077 |
| Sound | Vibration only | Vibration + Brauneiger-style buzzer |
| Filter | Simple EMA smoothing | Complementary: gravity-vector + baro |
| Architecture | Linear code | Finite state machine (5 states) |
| Dispatcher | `if (state == X)` scattered | `switch(fsm.state)` — table-driven |
| Hidden state | `static` locals | `struct VarioFsm` — zero static |
| Buttons | BACK/SELECT/RIGHT | UP/OK/DOWN — any wakes |
| Wake source | Only BACK (GPIO 25) | Any button (UP/OK/DOWN) |
| Settings | None | Settings screen: Buzzer/Vibro ON/OFF |
| Self-test | None | SELF-TEST on button wake |
| BMP failure | Silent freeze | SENSOR FAIL overlay |
| Accelerometer | Magnitude only | Chip ID check + fallback |
| Task shutdown | Immediate vTaskDelete | Flag + delay — safe I2C |
| Sleep logic | Fixed | Smart: motionTime, 15 min → 24h |
| Config style | `#define` | `constexpr` |
| Code language | Russian | English (100%) |

---

**License:** MIT

---

## 1. What It Is

VibroVarioAuto is a barometric variometer for paragliders, hang gliders, and sailplanes, built on the Watchy smartwatch platform (ESP32 + E-Ink display).

### Key Differences from Other Variometers

- **Silent mode** — wrist vibration motor, no beeping
- **Sound mode** — optional buzzer with Brauneiger-style tones
- **E-Ink screen** — readable in direct sunlight, zero power when idle
- **Power saving** — weeks in clock mode, 10+ hours in flight
- **Smart sleep** — auto-wakes every minute to check if you've launched
- **Weight** — only 24 g with battery

---

## 2. Hardware Sensors

| Sensor | Measures | Purpose |
|--------|----------|---------|
| BMP390 | Air pressure | Altitude and vertical speed |
| BMA423 | Acceleration (3-axis) | Fast filter response + tilt compensation |
| PCF8563 | Precise time | RTC, survives deep sleep |
| Vibration motor | — | Tactile feedback on wrist |

### How Sensors Work Together

**Barometer (BMP390)** samples at 50 Hz. Altitude is computed from pressure using standard atmosphere formula (1013.25 hPa reference).

**Accelerometer (BMA423)** provides two benefits:
1. **Gravity direction** — regardless of watch orientation, the gravity vector is tracked via LPF (2s time constant)
2. **Fast response** — acceleration is projected onto the gravity vector, giving instant vertical acceleration before baro catches up

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

**Top line:** flight time (hh:mm:ss), temperature (°C), battery %
**Middle:** relative altitude from start (`Start, m`) and barometric MSL altitude (`Sea, m`)
**Bottom:** `Vario` label, max/min climb/sink rates, current vertical speed (±m/s)

### Settings Screen

```
┌──────────────────────┐
│      Settings        │
│                      │
│   Buzzer: [ON]       │
│                      │
│   Vibro: [OFF]       │
│                      │
│  OK-toggle UP-exit   │
└──────────────────────┘
```

**Controls in SETTINGS:**

| Button | Action |
|--------|--------|
| OK | Toggle: Buzzer ON/OFF → Vibro ON/OFF → ... |
| DOWN | Reset both to ON |
| UP | Exit to clock (CLOCK) |

Settings persist across deep sleep (stored in RTC memory).

### Self-Test Screen

On every button wake (UP/OK/DOWN), the device runs a self-check:

| Check | Method | On failure shows |
|-------|--------|-----------------|
| DOWN, OK buttons | Check if stuck | `BTN: DOWN OK` |
| BMP390 barometer | I2C communication | `BARO FAIL` |
| BMA423 accelerometer | Chip ID register | `ACCEL FAIL` |
| Battery | Voltage < 3.4V | `LOW BATT` |
| BMP390 in flight | 50 Hz reads | `SENSOR FAIL` overlay |

If all OK — self-test passes silently.
If issue found — **SELF-TEST** screen displayed for 5 seconds, then clock screen.

---

## 4. Operating Modes (FSM)

The device has 5 states:

```
    ┌──────────┐
    │  CLOCK   │ ◄──── deep sleep / wake
    └────┬─────┘
         │
    ┌────▼──────┐   ┌────────────┐
    │ SETTINGS  │   │ CALIBRATING│
    └────▲──────┘   └─────┬──────┘
         │                │
    ┌────┴──────────┐     │
    │   RUNNING     │◄────┘
    │ (vario active)│
    └──────┬───────-┘
           │
    ┌──────▼───────-┐
    │   STOPPED     │
    │ (stats frozen)│
    └──────┬───────-┘
           │
      (back to CLOCK)
```

### DEEP SLEEP

Consumption: **~5 µA** (battery lasts months).

- Screen blank, all sensors off
- Wake by **any button**: UP, OK, or DOWN (SETUP = sensor power pin, does not wake)
- Self-test runs on wake
- Auto-entry: after 5 min idle in CLOCK

### CLOCK — Clock Mode

Consumption: **~30 µA** average.

- Shows time, date, battery
- Sleeps most of the time, wakes for ~0.1s every 60s for 1 BMP reading
- Auto-detects launch: if Vz > 0.75 m/s since last check → shows "FLIGHT!"
- Also wakes on motion (accelerometer any-motion interrupt)
- **Buttons in CLOCK:** DOWN → Settings, OK → Start flight, UP → Deep sleep

### CALIBRATING — Pre-Flight Calibration

- Sensors powered, 2-second calibration
- Accelerometer G-reference captured
- Filter stabilizes altitude
- Transitions to RUNNING when done

### RUNNING — Flight Mode

Consumption: **~7 mA** (buzzer adds ~3 mA).

- All sensors active: baro 50 Hz, accelerometer, display, buzzer, vibration
- Vertical speed updated in real-time (complementary filter)
- Display refreshes 1x/sec (E-Ink is slow)
- Buzzer and vibro run continuously

**Auto-landing:** if Vz < 0.75 m/s and no motion for 5 minutes → auto-transition to CLOCK.

### STOPPED — Flight Paused

- All readings frozen, stopwatch stopped
- Sensors powered down to save battery
- UP → exit to CLOCK, DOWN → new flight, OK → reset RTC to 00:00

---

## 5. Button Reference

| Button | GPIO | Position | CLOCK | RUNNING | STOPPED | SETTINGS |
|--------|------|----------|-------|---------|---------|----------|
| SETUP | 26 | Top-left | — (sensor power) | — | — | — |
| UP | 25 | Top-right | Go to sleep | Exit to CLOCK | Exit to CLOCK | Exit to CLOCK |
| OK | 4 | Bottom-left | Start flight | Stop flight | Reset RTC | Toggle ON/OFF |
| DOWN | 35 | Bottom-right | Settings | — | New flight | Reset to ON |

**Wake from deep sleep:** Any button (UP/OK/DOWN). SETUP does not wake.

---

## 6. Flight Workflow

### Before Flight

1. Check battery — should be > 50% on clock screen
2. Mount watch on wrist, harness pocket, or riser
3. Verify vibro is felt. If buzzer is connected — verify sound.
4. Check Settings (DOWN in CLOCK): Buzzer and Vibro should be [ON]

### Launch

**Manual (recommended):**
1. Wake device if sleeping (any button)
2. Press OK on clock screen
3. "Calibrating..." appears — **keep still for 2 seconds**
4. Flight mode starts automatically

**Automatic:**
1. Device in CLOCK mode (showing time)
2. When you launch, device detects altitude gain on its next check (1-60s delay)
3. Screen shows "FLIGHT! Press DOWN"
4. Press DOWN or OK → calibration (2s) → flight

### In Flight

**Buzzer (if connected):**
- Fast high-pitch beeping → you're in a thermal (good)
- Slow low-pitch beeping → you're sinking
- Continuous low tone → emergency sink (> 3 m/s down)
- Silence — speed below 0.75 m/s

**Vibration motor:**
- 1 pulse — weak lift (0.15-0.4 m/s)
- 2 pulses — moderate lift (0.4-1.0 m/s)
- 3 pulses — strong lift (1.0-2.0 m/s)
- Frequent pulses — very strong lift (> 2.0 m/s)
- Continuous vibration — hard sink (> 5 m/s down)
- No vibration — silent zone or very weak movement

**Screen** (updates 1x/sec): flight time, altitude, vario, max/min records

### Landing

**Manual:**
1. Press OK — flight stops (timer freezes)
2. Review flight statistics
3. Press UP — return to CLOCK

**Automatic:**
1. After landing, device detects Vz < 0.75 m/s + no motion
2. After 5 minutes — auto-stops flight, returns to CLOCK

### After Flight

- If left untouched for 5 minutes in CLOCK → goes to DEEP SLEEP
- To force sleep: press UP in CLOCK
- Charge via USB

---

## 7. Power Consumption

| Mode | Current | Runtime |
|------|---------|---------|
| DEEP SLEEP | 5 µA | Months |
| CLOCK (average) | 30 µA | Weeks |
| RUNNING (flight) | 7 mA | 10+ hours on 200 mAh |
| RUNNING + buzzer | ~10 mA | 8+ hours |

Battery: 200 mAh LiPo (3.7V).
Charge indicator: 4.2V = 100%, 3.3V = 0%.

---

## 8. Troubleshooting

**Q: Screen is blank, nothing happens.**
A: Press any button (UP/OK/DOWN) to wake. If no response — connect USB.

**Q: Device doesn't respond to buttons.**
A: Locked up. Connect USB — it will reboot.

**Q: Variometer shows nonsense.**
A: Recalibrate:
1. Press UP → CLOCK
2. Press OK → "Calibrating..."
3. Keep still for 2 seconds
4. Fly again

**Q: No buzzer sound.**
A: Check buzzer wiring (GPIO 32 + GND). May need a transistor if >50 mA.
Check Settings: DOWN on clock → Buzzer: [ON].

**Q: Auto-launch doesn't work.**
A: Up to 60s delay between checks. Press OK manually to start.

**Q: SENSOR FAIL on screen.**
A: BMP390 barometer failure. Check wiring/solder joints. If error clears on next wake — temporary I2C glitch.

**Q: SELF-TEST shows LOW BATT.**
A: Battery below 3.4V. Charge via USB.

**Q: Battery drains quickly.**
A: Flight: 7-10 mA is normal. On ground, device should sleep at ~5 µA. If not — press UP for forced sleep.

---

## 9. Credits

Original project: [github.com/isemaster/VibroVario](https://github.com/isemaster/VibroVario)  
Developed by: isemaster  
License: MIT — use, modify, sell. Keep attribution.

---

*Fly safe!*
