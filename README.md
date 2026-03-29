# 🎵 Audio Visualizer + Party Lights v1.1

A real-time audio spectrum analyser and RGB party light controller for the **ESP32-S3**. Uses an INMP441 microphone, ST7789 TFT display, and the built-in WS2812B RGB LED. FFT runs on Core 0 while the display and LED effects run on Core 1 for smooth, stutter-free visuals.

---

## ✨ Features

- **24-band FFT spectrum analyser** with logarithmic frequency spacing
- **3 visualiser modes** — auto-cycle every 30 seconds:
  - `SPECTRUM` — animated gradient bars with peak-hold dots
  - `MIRROR` — bars grow symmetrically above and below a centre line
  - `RADIAL` — 24 rays burst from the screen centre with a pulsing bass circle
- **4 LED party light modes** cycling alongside the visualiser:
  - `Rainbow Pulse` — hue glides continuously, brightness tracks loudness
  - `Bass Strobe` — white flash on beat, dim glow between hits
  - `Freq Colour` — R = bass, G = mid, B = treble
  - `Party Chaos` — random rapid colour bursts on every beat
- Beat detection with adjustable sensitivity
- Animated boot sequence with simultaneous LED rainbow sweep
- Level meter strip and beat-flash side accent bars
- Dual-core FreeRTOS architecture (audio on Core 0, display on Core 1)

---

## 🛒 Hardware Required

| Component | Details |
|-----------|---------|
| ESP32-S3 Dev Module | With built-in WS2812B LED on GPIO 48 |
| INMP441 Microphone | I2S digital MEMS mic |
| ST7789 TFT Display | 240 × 320 px |

> **⚠️ GPIO 48 Note:** This sketch uses GPIO 48 as the WS2812B LED data pin. In the HW Diagnostics sketch, GPIO 48 is PA_EN (amplifier power). The codec/speaker is **not used** in this sketch so there is no conflict. If you need both speaker and LED simultaneously in a future project, reroute PA_EN to a free GPIO (e.g. GPIO 7).

---

## 🔌 Wiring

### INMP441 Microphone → ESP32-S3

| Signal | GPIO |
|--------|------|
| VDD | 3.3V |
| GND | GND |
| L/R | GND (left channel) |
| WS | 4 |
| SCK | 5 |
| SD | 6 |

### ST7789 TFT → ESP32-S3

| Signal | GPIO |
|--------|------|
| DC | 39 |
| CS | 47 |
| CLK | 41 |
| MOSI | 40 |
| BLK | 42 |

### Built-in WS2812B LED

| Signal | GPIO |
|--------|------|
| Data | 48 (built-in, no external wiring needed) |

---

## 📚 Libraries

| Library | Source |
|---------|--------|
| `arduino-audio-tools` | [pschatzmann/arduino-audio-tools](https://github.com/pschatzmann/arduino-audio-tools) — already installed |
| `Adafruit ST7789` | Arduino Library Manager — already installed |
| `Adafruit GFX Library` | Arduino Library Manager — already installed |
| `Adafruit NeoPixel` | Arduino Library Manager ← **install this** |
| `arduinoFFT` (v2.x by Enrique Condes) | Arduino Library Manager ← **install this** |

---

## ⚙️ Board Settings (Arduino IDE)

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| USB CDC on Boot | Enabled |

---

## 🎛️ Tuning Parameters

All sensitivity knobs are at the top of the sketch for easy adjustment:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `GAIN` | 14.0 | Increase if bars stay low; decrease if always clipping |
| `ATTACK_FAST` | 0.70 | Higher = bars rise faster (0.0 – 1.0) |
| `DECAY_RATE` | 0.80 | Lower = bars fall faster (0.0 – 1.0) |
| `BEAT_THRESH` | 1.6 | Lower = more beat triggers |
| `BEAT_MIN_BASS` | 0.06 | Minimum bass energy to qualify as a beat |
| `MODE_SEC` | 30 | Seconds between automatic mode changes |
| `LED_BRIGHT` | 210 | WS2812 maximum brightness (0 – 255) |
| `NUM_BANDS` | 24 | Number of EQ bars |
| `FFT_SIZE` | 256 | FFT window size — larger = more frequency resolution |

---

## 🏗️ Architecture

```
Core 0 — audioTask()
  INMP441 → I2S read → Hann window → FFT → band energies → beat detect
  Publishes data to shared buffer via FreeRTOS mutex

Core 1 — loop()
  Reads shared buffer → renders visualiser frame → updates WS2812B LED
  Auto-cycles viz mode + LED mode every MODE_SEC seconds
```

---

## 🚀 Getting Started

1. Wire hardware per the tables above.
2. Install **Adafruit NeoPixel** and **arduinoFFT** from the Library Manager.
3. Select **ESP32S3 Dev Module**, enable **USB CDC on Boot**.
4. Upload the sketch.
5. Play music or speak near the INMP441 — bars and LEDs will react immediately.
6. Modes auto-cycle every 30 seconds, or adjust `MODE_SEC` to change the interval.

---

## 📄 License

MIT License — see [LICENSE](LICENSE_AudioVisualizer) for details.  
© 2026 Patrick Perez
