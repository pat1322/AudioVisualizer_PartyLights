/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║      AUDIO VISUALIZER + PARTY LIGHTS  v1.1                   ║
 * ║                 by Patrick Perez  © 2026                     ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  ESP32-S3  │  INMP441 Mic  │  ST7789 TFT 320×240             ║
 * ║  Built-in WS2812 RGB LED (GPIO 48)                           ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  LIBRARIES — Install via Arduino Library Manager:            ║
 * ║  ① arduino-audio-tools  (pschatzmann) — already installed	  ║
 * ║  ② Adafruit ST7789 + Adafruit GFX    — already installed	  ║
 * ║  ③ Adafruit NeoPixel                 ← NEW  install this	  ║
 * ║  ④ arduinoFFT  (Enrique Condes v2.x) ← NEW  install this	  ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  ⚠  GPIO 48 CONFLICT:                                       ║
 * ║     In your diagnostics sketch GPIO 48 = PA_EN (amp power).  ║
 * ║     This sketch drives GPIO 48 as the built-in WS2812 LED.   ║
 * ║     Speaker / codec is NOT used here — no conflict arises.   ║
 * ║     If you ever want speaker + LED simultaneously, reroute   ║
 * ║     PA_EN to a free GPIO (e.g. GPIO 7) in both sketches.     ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 *  WIRING  (unchanged from your diagnostics sketch)
 *  ─────────────────────────────────────────────────────────────
 *  INMP441  :  WS→4   SCK→5   SD→6   VDD→3.3V  GND→GND  L/R→GND
 *  ST7789   :  DC→39  CS→47  CLK→41  MOSI→40   BLK→42
 *  LED      :  GPIO 48  (built-in WS2812B, 1 pixel)
 *
 *  VISUALIZER MODES  (auto-cycle every 30 s)
 *  ─────────────────────────────────────────────────────────────
 *  VIZ 0 — SPECTRUM    24 animated gradient bars, peak dots
 *  VIZ 1 — MIRROR      bars grow symmetrically from centre line
 *  VIZ 2 — RADIAL      24 rays burst from screen centre
 *
 *  LED EFFECTS  (cycle with viz modes)
 *  ─────────────────────────────────────────────────────────────
 *  LED 0 — Rainbow Pulse  hue glides, brightness = loudness
 *  LED 1 — Bass Strobe    white flash on beat, dim cyan glow
 *  LED 2 — Freq Colour    R=bass  G=mid  B=treble
 *  LED 3 — Party Chaos    random rapid colour bursts on beat
 *
 *  TUNING
 *  ─────────────────────────────────────────────────────────────
 *  GAIN          raise if bars stay low; lower if always clipping
 *  ATTACK_FAST   higher = bars rise faster  (0.0–1.0)
 *  DECAY_RATE    lower  = bars fall faster  (0.0–1.0)
 *  BEAT_THRESH   lower  = more beat triggers
 *  MODE_SEC      seconds between auto-mode changes
 */

#include "AudioTools.h"
#include "AudioTools/CoreAudio/AudioI2S/I2SStream.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_NeoPixel.h>
#include <arduinoFFT.h>          // v2.x: ArduinoFFT<float>

// ═══════════════════════════════════════════════════════════════
//  PIN MAP
// ═══════════════════════════════════════════════════════════════
#define PIN_MIC_WS    4
#define PIN_MIC_SCK   5
#define PIN_MIC_SD    6

#define PIN_TFT_CS    47
#define PIN_TFT_DC    39
#define PIN_TFT_RST   -1
#define PIN_TFT_BLK   42
#define PIN_TFT_CLK   41
#define PIN_TFT_MOSI  40

#define LED_PIN       48         // built-in WS2812B
#define LED_COUNT     1

// ═══════════════════════════════════════════════════════════════
//  USER-TUNABLE PARAMETERS
// ═══════════════════════════════════════════════════════════════
#define SAMPLE_RATE   16000      // must match mic capability
#define FFT_SIZE      256        // power of 2 — larger = more resolution
#define NUM_BANDS     24         // EQ bar count
#define MODE_SEC      30         // seconds before auto-switching modes
#define LED_BRIGHT    210        // WS2812 max brightness 0-255

// ── Sensitivity knobs ──────────────────────────────────────────
//  GAIN: was 7.0 → 14.0  (doubles bar height for the same input)
//  Push higher (e.g. 20) for very quiet environments; pull back if clipping.
#define GAIN          14.0f

//  Attack blend: was 0.50 → 0.70  (bars jump up faster)
#define ATTACK_FAST   0.70f

//  Decay multiplier: was 0.85 → 0.80  (bars fall slightly faster → snappier)
#define DECAY_RATE    0.80f

//  Beat detection: energy must exceed (average × BEAT_THRESH)
//  was 2.0 → 1.6  (fires on quieter transients)
#define BEAT_THRESH   1.6f

//  Minimum bass energy to qualify as a beat: was 0.10 → 0.06
#define BEAT_MIN_BASS 0.06f

// ═══════════════════════════════════════════════════════════════
//  DISPLAY GEOMETRY  (320 × 240 landscape)
// ═══════════════════════════════════════════════════════════════
#define SCR_W        320
#define SCR_H        240
#define TITLE_H       22         // header bar height
#define FOOTER_H      14         // footer bar height
#define BAR_BOTTOM   (SCR_H - FOOTER_H)     // y = 226
#define BAR_AREA_H   (BAR_BOTTOM - TITLE_H) // 204 px of usable height
#define BAR_W         12         // individual bar pixel width
#define BAR_GAP        1         // gap between bars
#define BAR_STRIDE   (BAR_W + BAR_GAP)      // 13 px per bar slot
// 24 × 13 = 312 px; centre on 320 → start at x = 4
#define BAR_ORIGIN_X ((SCR_W - NUM_BANDS * BAR_STRIDE + BAR_GAP) / 2)
// Radial mode centre
#define RAD_CX       160
#define RAD_CY       ((TITLE_H + BAR_BOTTOM) / 2)  // 124
#define RAD_INNER     20
#define RAD_MAX       75

// RGB565 colours
#define C_BG     0x0841
#define C_PANEL  0x1082
#define C_BORDER 0x2945
#define C_CYAN   0x07FF
#define C_WHT    0xFFFF
#define C_DIM    0x4228
#define C_GRN    0x07E0
#define C_BLK    0x0000

// ═══════════════════════════════════════════════════════════════
//  HARDWARE OBJECTS
// ═══════════════════════════════════════════════════════════════
SPIClass          tftSPI(HSPI);
Adafruit_ST7789   tft(&tftSPI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
I2SStream         mic;

// ═══════════════════════════════════════════════════════════════
//  FFT
// ═══════════════════════════════════════════════════════════════
static float vR[FFT_SIZE], vI[FFT_SIZE];
ArduinoFFT<float> FFT(vR, vI, FFT_SIZE, (float)SAMPLE_RATE);

// Band → bin mapping (logarithmic spacing, built once at startup)
static int binLo[NUM_BANDS], binHi[NUM_BANDS];

void buildBandMap() {
    const float lo = 1.5f, hi = FFT_SIZE / 2.0f - 1.0f;
    for (int b = 0; b < NUM_BANDS; b++) {
        binLo[b] = max(1, (int)(lo * powf(hi / lo, (float)b       / NUM_BANDS)));
        binHi[b] = max(binLo[b] + 1,
                       (int)(lo * powf(hi / lo, (float)(b + 1) / NUM_BANDS)));
    }
}

// Precomputed sin/cos for radial rays
static float radSin[NUM_BANDS], radCos[NUM_BANDS];

void buildRadialMap() {
    for (int b = 0; b < NUM_BANDS; b++) {
        float a = ((float)b / NUM_BANDS) * TWO_PI - HALF_PI;
        radSin[b] = sinf(a);
        radCos[b] = cosf(a);
    }
}

// ═══════════════════════════════════════════════════════════════
//  SHARED DATA  (Core 0 → Core 1 via mutex)
// ═══════════════════════════════════════════════════════════════
static volatile float  shBand[NUM_BANDS];
static volatile float  shPeak[NUM_BANDS];
static volatile float  shLevel, shBass;
static volatile bool   shBeat;
static SemaphoreHandle_t mtx;

// ═══════════════════════════════════════════════════════════════
//  COLOR MATH
// ═══════════════════════════════════════════════════════════════

// h: 0–360   s,v: 0–1  → RGB565
uint16_t hsvTo565(float h, float s, float v) {
    h = fmodf(h + 720.0f, 360.0f);
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r, g, b;
    if      (h <  60) { r = c+m; g = x+m; b = m;   }
    else if (h < 120) { r = x+m; g = c+m; b = m;   }
    else if (h < 180) { r = m;   g = c+m; b = x+m; }
    else if (h < 240) { r = m;   g = x+m; b = c+m; }
    else if (h < 300) { r = x+m; g = m;   b = c+m; }
    else              { r = c+m; g = m;   b = x+m; }
    return ((uint16_t)(r * 31) << 11)
         | ((uint16_t)(g * 63) <<  5)
         |  (uint16_t)(b * 31);
}

// h: 0–360   s,v: 0–1  → NeoPixel uint32
uint32_t neoHSV(float h, float s, float v) {
    h = fmodf(h + 720.0f, 360.0f);
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r, g, b;
    if      (h <  60) { r = c+m; g = x+m; b = m;   }
    else if (h < 120) { r = x+m; g = c+m; b = m;   }
    else if (h < 180) { r = m;   g = c+m; b = x+m; }
    else if (h < 240) { r = m;   g = x+m; b = c+m; }
    else if (h < 300) { r = x+m; g = m;   b = c+m; }
    else              { r = c+m; g = m;   b = x+m; }
    return led.Color((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255));
}

// Gradient colour for a bar pixel:
//   norm   = fractional height  0 (bottom) → 1 (top)
//   band   = bar index, spreads hue slightly left→right
//   hBase  = global hue shift (animated over time)
uint16_t barColor(int band, float norm, float hBase) {
    float h = hBase + (float)band * 6.0f + norm * 55.0f;
    float v = 0.28f + norm * 0.72f;
    return hsvTo565(h, 1.0f, v);
}

// ═══════════════════════════════════════════════════════════════
//  AUDIO TASK  — Core 0
//  Reads INMP441, runs FFT, publishes band energies + beat flag.
// ═══════════════════════════════════════════════════════════════
static float smBand[NUM_BANDS]  = {};   // smoothed magnitudes (task-local)
static float smPeak[NUM_BANDS]  = {};   // peak hold (task-local)
static int   peakTmr[NUM_BANDS] = {};   // peak hold counter

void audioTask(void*) {
    // --- Init INMP441 on I2S port 1 (same config as diagnostics sketch) ---
    auto cfg          = mic.defaultConfig(RX_MODE);
    cfg.sample_rate   = SAMPLE_RATE;
    cfg.channels      = 2;
    cfg.bits_per_sample = 32;       // 24-bit data in 32-bit frame
    cfg.i2s_format    = I2S_STD_FORMAT;
    cfg.port_no       = 1;
    cfg.pin_ws        = PIN_MIC_WS;
    cfg.pin_bck       = PIN_MIC_SCK;
    cfg.pin_data      = PIN_MIC_SD;
    cfg.pin_mck       = -1;
    cfg.use_apll      = false;
    mic.begin(cfg);

    static int32_t  raw[FFT_SIZE * 2];   // stereo 32-bit DMA buffer
    static float    avgLevel   = 0.0f;
    static uint32_t lastBeatMs = 0;

    for (;;) {
        // --- Collect exactly FFT_SIZE stereo 32-bit samples ---
        int need = FFT_SIZE * 2 * sizeof(int32_t);
        int got  = 0;
        while (got < need) {
            int r = mic.readBytes((uint8_t*)raw + got, need - got);
            if (r > 0) got += r;
            else vTaskDelay(1);
        }

        // --- Extract left channel; apply Hann window; zero imag ---
        for (int i = 0; i < FFT_SIZE; i++) {
            // INMP441 left channel: shift right by 14 to get ≈16-bit range
            // then normalise to ±1.0 (same bit-shift as diagnostics recordMic)
            float s = (float)(raw[i * 2] >> 14) / 32768.0f;
            float w = 0.5f * (1.0f - cosf(TWO_PI * i / (FFT_SIZE - 1)));
            vR[i] = s * w;
            vI[i] = 0.0f;
        }

        // --- FFT (arduinoFFT v2 API) ---
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude();  // result in vR[0..FFT_SIZE/2-1]

        // --- Bin → band accumulation; normalise; apply GAIN ---
        float rawB[NUM_BANDS];
        float totLevel = 0.0f;
        const float normScale = GAIN / (float)FFT_SIZE;
        for (int b = 0; b < NUM_BANDS; b++) {
            float sum = 0.0f;
            int   cnt = binHi[b] - binLo[b];
            for (int k = binLo[b]; k < binHi[b]; k++) sum += vR[k];
            rawB[b]   = constrain(sum * normScale / (float)cnt, 0.0f, 1.0f);
            totLevel += rawB[b];
        }
        totLevel = constrain(totLevel / NUM_BANDS, 0.0f, 1.0f);

        // --- Asymmetric attack / decay envelope per band ---
        // CHANGED: ATTACK_FAST (0.70) blends more new signal on the way up
        //          DECAY_RATE  (0.80) drops slightly faster than before (0.85)
        for (int b = 0; b < NUM_BANDS; b++) {
            smBand[b] = (rawB[b] > smBand[b])
                      ? ATTACK_FAST * rawB[b] + (1.0f - ATTACK_FAST) * smBand[b]
                      : smBand[b] * DECAY_RATE;

            // Peak hold: freeze for ~35 frames then slide down
            if (smBand[b] >= smPeak[b]) {
                smPeak[b]   = smBand[b];
                peakTmr[b]  = 35;
            } else if (peakTmr[b] > 0) {
                --peakTmr[b];
            } else {
                smPeak[b] *= 0.93f;
            }
        }

        // --- Bass energy (bands 0–4 ≈ 60–500 Hz) ---
        float bass = 0.0f;
        for (int b = 0; b < 5; b++) bass += smBand[b];
        bass /= 5.0f;

        // --- Beat detection ---
        // CHANGED: threshold 2.0 → BEAT_THRESH (1.6), min bass 0.10 → BEAT_MIN_BASS (0.06)
        avgLevel = 0.95f * avgLevel + 0.05f * totLevel;
        bool beat = (bass > avgLevel * BEAT_THRESH)
                 && (bass > BEAT_MIN_BASS)
                 && ((millis() - lastBeatMs) > 200);
        if (beat) lastBeatMs = millis();

        // --- Publish to Core 1 (non-blocking try-lock) ---
        if (xSemaphoreTake(mtx, 0) == pdTRUE) {
            for (int b = 0; b < NUM_BANDS; b++) {
                shBand[b] = smBand[b];
                shPeak[b] = smPeak[b];
            }
            shLevel = totLevel;
            shBass  = bass;
            shBeat  = beat;
            xSemaphoreGive(mtx);
        }

        vTaskDelay(2);   // yield so Core 0 can handle system tasks
    }
}

// ═══════════════════════════════════════════════════════════════
//  DISPLAY — local working copies (Core 1 only)
// ═══════════════════════════════════════════════════════════════
static float dBand[NUM_BANDS] = {};
static float dPeak[NUM_BANDS] = {};
static float dLevel = 0.0f, dBass = 0.0f;
static bool  dBeat  = false;

static float hueBase = 0.0f;    // global animated hue offset

// Previous bar heights for incremental erase (avoids full clear each frame)
static float prevH[NUM_BANDS]   = {};
static float prevP[NUM_BANDS]   = {};
static float prevMH[NUM_BANDS]  = {};   // mirror mode
static float prevRad[NUM_BANDS] = {};   // radial mode

// Mode state
static uint8_t  vizMode = 0;            // 0 spectrum  1 mirror  2 radial
static uint8_t  ledMode = 0;            // 0–3
static uint32_t modeTs  = 0;

// Beat flash accent state
static float flashAmt = 0.0f;

// Level-meter previous pixel
static int prevLvlPx = 0;

// ═══════════════════════════════════════════════════════════════
//  CHROME (static header + footer)
// ═══════════════════════════════════════════════════════════════
static const char* const VIZ_NAMES[] = { "SPECTRUM", " MIRROR ", " RADIAL " };
static const char* const LED_NAMES[] = { "RainbowPulse", "BassStrobe  ",
                                          "FreqColour  ", "PartyMode   " };

void drawChrome() {
    // ── Header ──────────────────────────────────────────────────
    tft.fillRect(0, 0, SCR_W, TITLE_H, C_PANEL);
    tft.drawFastHLine(0, TITLE_H - 1, SCR_W, C_CYAN);
    tft.setTextSize(1);
    tft.setTextColor(C_CYAN);
    tft.setCursor(4, 7);  tft.print("AUDIO VISUALIZER");
    tft.setTextColor(C_GRN);
    tft.setCursor(SCR_W - 62, 7); tft.print(VIZ_NAMES[vizMode]);

    // ── Footer ──────────────────────────────────────────────────
    tft.fillRect(0, BAR_BOTTOM, SCR_W, FOOTER_H, C_PANEL);
    tft.drawFastHLine(0, BAR_BOTTOM, SCR_W, C_BORDER);
    tft.setTextColor(C_DIM); tft.setTextSize(1);
    tft.setCursor(4, BAR_BOTTOM + 3);
    tft.print("Patrick Perez 2026  \xb7  INMP441 + WS2812 + ESP32-S3");

    // LED mode badge (top-right, small)
    tft.setTextColor(0xFD20);          // orange
    tft.setCursor(4, BAR_BOTTOM + 3 + 0);
    // overprint already done above; just show led mode at right
    tft.fillRect(SCR_W - 82, BAR_BOTTOM + 1, 82, 11, C_PANEL);
    tft.setTextColor(0xFD20);
    tft.setCursor(SCR_W - 80, BAR_BOTTOM + 3);
    tft.print(LED_NAMES[ledMode]);
}

// ═══════════════════════════════════════════════════════════════
//  VIZ MODE 0 — SPECTRUM BARS
// ═══════════════════════════════════════════════════════════════
void drawSpectrum() {
    for (int b = 0; b < NUM_BANDS; b++) {
        int x      = BAR_ORIGIN_X + b * BAR_STRIDE;
        int newPx  = constrain((int)(dBand[b] * BAR_AREA_H), 0, BAR_AREA_H);
        int peakPx = constrain((int)(dPeak[b] * BAR_AREA_H), 0, BAR_AREA_H);
        int oldPx  = constrain((int)(prevH[b]  * BAR_AREA_H), 0, BAR_AREA_H);
        int oldPPx = constrain((int)(prevP[b]  * BAR_AREA_H), 0, BAR_AREA_H);

        // Erase top of bar if it shrank
        if (oldPx > newPx)
            tft.fillRect(x, BAR_BOTTOM - oldPx, BAR_W, oldPx - newPx, C_BG);

        // Erase old peak dot (3 px tall safety zone)
        if (oldPPx > 2)
            tft.fillRect(x, BAR_BOTTOM - oldPPx - 2, BAR_W, 3, C_BG);

        // Draw bar in 5 colour segments (saves per-pixel SPI calls)
        if (newPx > 0) {
            for (int s = 0; s < 5; s++) {
                int y0 = s       * newPx / 5;
                int y1 = (s + 1) * newPx / 5;
                if (y1 <= y0) continue;
                float norm = (float)(y0 + y1) * 0.5f / (float)BAR_AREA_H;
                tft.fillRect(x, BAR_BOTTOM - y1, BAR_W, y1 - y0,
                             barColor(b, norm, hueBase));
            }
        }

        // Peak hold dot — 2 px white line
        if (peakPx > 3)
            tft.fillRect(x, BAR_BOTTOM - peakPx - 1, BAR_W, 2, C_WHT);

        prevH[b] = dBand[b];
        prevP[b] = dPeak[b];
    }
}

// ═══════════════════════════════════════════════════════════════
//  VIZ MODE 1 — MIRROR BARS  (symmetric above & below midline)
// ═══════════════════════════════════════════════════════════════
void drawMirror() {
    const int midY   = (TITLE_H + BAR_BOTTOM) / 2;     // y = 124
    const int hMax   = (BAR_BOTTOM - TITLE_H) / 2 - 2; // ≈ 100 px

    // Thin centre axis line (drawn once, stays until clear)
    tft.drawFastHLine(0, midY, SCR_W, C_BORDER);

    for (int b = 0; b < NUM_BANDS; b++) {
        int x    = BAR_ORIGIN_X + b * BAR_STRIDE;
        int newH = constrain((int)(dBand[b] * hMax), 0, hMax);
        int oldH = constrain((int)(prevMH[b] * hMax),  0, hMax);

        // Erase the parts that shrank
        if (oldH > newH) {
            tft.fillRect(x, midY - oldH,     BAR_W, oldH - newH, C_BG);
            tft.fillRect(x, midY + newH + 1, BAR_W, oldH - newH, C_BG);
        }

        if (newH > 0) {
            // Colour shifts 180° from spectrum mode for contrast
            for (int s = 0; s < 4; s++) {
                int h0 = s * newH / 4, h1 = (s + 1) * newH / 4;
                if (h1 <= h0) continue;
                float norm = (float)(h0 + h1) * 0.5f / (float)hMax;
                uint16_t col = barColor(b, norm, hueBase + 180.0f);
                // Upper half (grows upward from midY)
                tft.fillRect(x, midY - h1,     BAR_W, h1 - h0, col);
                // Lower half (grows downward from midY)
                tft.fillRect(x, midY + h0 + 1, BAR_W, h1 - h0, col);
            }
        }
        prevMH[b] = dBand[b];
    }
}

// ═══════════════════════════════════════════════════════════════
//  VIZ MODE 2 — RADIAL BARS  (rays burst from screen centre)
// ═══════════════════════════════════════════════════════════════
static int prevCR = 0;   // previous centre-circle radius

void drawRadial() {
    for (int b = 0; b < NUM_BANDS; b++) {
        float oldLen = prevRad[b] * RAD_MAX;
        float newLen = dBand[b]   * RAD_MAX;
        float cs = radCos[b], sn = radSin[b];

        // Erase old ray — draw it in background colour (3 px wide)
        if (oldLen > 0.5f) {
            int ox1 = RAD_CX + (int)(cs * RAD_INNER);
            int oy1 = RAD_CY + (int)(sn * RAD_INNER);
            int ox2 = RAD_CX + (int)(cs * (RAD_INNER + oldLen));
            int oy2 = RAD_CY + (int)(sn * (RAD_INNER + oldLen));
            tft.drawLine(ox1,   oy1,   ox2,   oy2,   C_BG);
            tft.drawLine(ox1+1, oy1,   ox2+1, oy2,   C_BG);
            tft.drawLine(ox1,   oy1+1, ox2,   oy2+1, C_BG);
        }

        // Draw new ray (3 px wide for visibility)
        if (newLen > 1.5f) {
            int nx1 = RAD_CX + (int)(cs * RAD_INNER);
            int ny1 = RAD_CY + (int)(sn * RAD_INNER);
            int nx2 = RAD_CX + (int)(cs * (RAD_INNER + newLen));
            int ny2 = RAD_CY + (int)(sn * (RAD_INNER + newLen));
            uint16_t col = barColor(b, dBand[b], hueBase);
            tft.drawLine(nx1,   ny1,   nx2,   ny2,   col);
            tft.drawLine(nx1+1, ny1,   nx2+1, ny2,   col);
            tft.drawLine(nx1,   ny1+1, nx2,   ny2+1, col);
        }

        prevRad[b] = dBand[b];
    }

    // Pulsing centre circle (radius driven by bass energy)
    int newCR = constrain((int)(dBass * 18) + 4, 4, 22);
    if (newCR != prevCR) {
        // Erase old
        tft.fillCircle(RAD_CX, RAD_CY, prevCR + 1, C_BG);
        // Draw new
        tft.fillCircle(RAD_CX, RAD_CY, newCR,
                        hsvTo565(hueBase + 30.0f, 1.0f, 0.95f));
        prevCR = newCR;
    }
}

// ═══════════════════════════════════════════════════════════════
//  LEVEL METER  (2 px strip just above the footer divider)
// ═══════════════════════════════════════════════════════════════
void drawLevelMeter() {
    int newPx = constrain((int)(dLevel * (SCR_W - 2)), 0, SCR_W - 2);
    if (newPx == prevLvlPx) return;

    if (newPx > prevLvlPx)
        tft.fillRect(prevLvlPx + 1, BAR_BOTTOM - 2,
                     newPx - prevLvlPx, 2,
                     hsvTo565(hueBase, 1.0f, 0.85f));
    else
        tft.fillRect(newPx + 1, BAR_BOTTOM - 2,
                     prevLvlPx - newPx, 2, C_BG);

    prevLvlPx = newPx;
}

// ═══════════════════════════════════════════════════════════════
//  BEAT FLASH  (brief cyan side-bar accent on transient)
// ═══════════════════════════════════════════════════════════════
void handleBeatFlash() {
    if (dBeat) flashAmt = 1.0f;
    if (flashAmt > 0.04f) {
        uint16_t fc = hsvTo565(hueBase + 40.0f, 0.9f, flashAmt * 0.55f);
        tft.fillRect(0,         TITLE_H, 4, BAR_AREA_H, fc);
        tft.fillRect(SCR_W - 4, TITLE_H, 4, BAR_AREA_H, fc);
        flashAmt *= 0.50f;
    }
}

// ═══════════════════════════════════════════════════════════════
//  LED EFFECTS  (4 modes, driven by audio data)
// ═══════════════════════════════════════════════════════════════
static float   ledHue      = 0.0f;
static float   ledSmooth   = 0.0f;   // smoothed brightness for mode 0
static uint32_t strobeMs   = 0;

void updateLED() {
    uint32_t now = millis();
    uint32_t color;

    switch (ledMode) {

        // ── Mode 0: Rainbow Pulse ───────────────────────────────
        // Hue glides continuously; brightness tracks loudness
        case 0:
            ledHue    = fmodf(ledHue + 2.2f, 360.0f);
            ledSmooth = 0.65f * ledSmooth + 0.35f * (dLevel * 2.8f + 0.08f);
            color     = neoHSV(ledHue, 1.0f, constrain(ledSmooth, 0.0f, 1.0f));
            break;

        // ── Mode 1: Bass Strobe ─────────────────────────────────
        // White burst on beat → fades → settles to dim beat-coloured glow
        case 1:
            if (dBeat) {
                color   = led.Color(255, 255, 255);
                strobeMs = now;
            } else {
                float t = 1.0f - constrain((float)(now - strobeMs) / 110.0f, 0.0f, 1.0f);
                if (t > 0.02f) {
                    uint8_t br = (uint8_t)(t * 255);
                    color = led.Color(br, br, br);
                } else {
                    color = neoHSV(hueBase, 1.0f,
                                   constrain(dBass * 0.45f + 0.05f, 0.0f, 1.0f));
                }
            }
            break;

        // ── Mode 2: Frequency Colour ────────────────────────────
        // R = bass energy   G = mid energy   B = treble energy
        case 2: {
            float r = 0.0f, g = 0.0f, bv = 0.0f;
            for (int b = 0;  b <  5;       b++) r  += dBand[b];
            for (int b = 5;  b < 15;       b++) g  += dBand[b];
            for (int b = 15; b < NUM_BANDS; b++) bv += dBand[b];
            r  = constrain(r  / 5.0f  * 1.6f, 0.0f, 1.0f);
            g  = constrain(g  / 10.0f * 1.6f, 0.0f, 1.0f);
            bv = constrain(bv / 9.0f  * 1.6f, 0.0f, 1.0f);
            color = led.Color((uint8_t)(r * 255),
                              (uint8_t)(g * 255),
                              (uint8_t)(bv * 255));
            break;
        }

        // ── Mode 3: Party Chaos ─────────────────────────────────
        // Random hue snaps on every beat (or every 110 ms during silence)
        // Bright on the transient, dim between
        case 3:
        default:
            if (dBeat || (now - strobeMs > 110)) {
                ledHue  = (float)random(360);
                strobeMs = now;
            }
            {
                bool hot = dBeat || ((now - strobeMs) < 55);
                color   = neoHSV(ledHue, 1.0f, hot ? 1.0f : 0.30f);
            }
            break;
    }

    led.setPixelColor(0, color);
    led.show();
}

// ═══════════════════════════════════════════════════════════════
//  BOOT ANIMATION
// ═══════════════════════════════════════════════════════════════
void bootAnim() {
    tft.fillScreen(C_BLK);

    // Phase 1 — Starfield + LED rainbow sweep simultaneously
    for (int i = 0; i < 100; i++) {
        uint16_t bright = random(3);
        uint16_t sc = (bright == 0) ? 0x2104 : (bright == 1) ? 0x4208 : 0x7BEF;
        tft.drawPixel(random(SCR_W), random(SCR_H), sc);
        led.setPixelColor(0, neoHSV((float)i * 3.6f, 1.0f, 0.85f));
        led.show();
        delay(5);
    }
    delay(80);

    // Phase 2 — Expanding concentric squares with hue gradient
    int cx = SCR_W / 2, cy = SCR_H / 2;
    for (int r = 6; r <= 70; r += 4) {
        tft.drawRect(cx - r, cy - r, r * 2, r * 2,
                     hsvTo565(160.0f + (float)r * 1.4f, 1.0f, 0.85f));
        led.setPixelColor(0, neoHSV(160.0f + (float)r * 1.4f, 1.0f, 1.0f));
        led.show();
        delay(22);
    }
    tft.fillRect(cx - 72, cy - 30, 144, 60, C_BG);

    // Phase 3 — Title fade-in
    uint16_t fades[] = { 0x0410, 0x0451, 0x04B2, 0x06D4, 0x07FF };
    for (int f = 0; f < 5; f++) {
        tft.fillRect(cx - 72, cy - 30, 144, 60, C_BG);
        tft.setTextColor(fades[f]); tft.setTextSize(2);
        tft.setCursor(cx - 60, cy - 24); tft.print("AUDIO VIZ");
        tft.setTextColor(C_WHT); tft.setTextSize(1);
        tft.setCursor(cx - 52, cy + 4);  tft.print("by Patrick Perez");
        tft.setCursor(cx - 40, cy + 16); tft.print("ESP32-S3  2026");
        led.setPixelColor(0, neoHSV((float)f * 30.0f + 180.0f, 1.0f, 0.9f));
        led.show();
        delay(90);
    }

    // Phase 4 — LED full rainbow burst
    for (int h = 0; h < 720; h += 8) {
        led.setPixelColor(0, neoHSV((float)h, 1.0f, 1.0f));
        led.show();
        delay(5);
    }
    led.setPixelColor(0, 0); led.show();
    delay(300);

    // Phase 5 — Wipe to working background
    for (int y = 0; y <= SCR_H; y += 5) {
        tft.fillRect(0, y, SCR_W, 5, C_BG);
        delay(3);
    }
}

// ═══════════════════════════════════════════════════════════════
//  CLEAR BAR AREA  (called when switching modes)
// ═══════════════════════════════════════════════════════════════
void clearBarArea() {
    tft.fillRect(0, TITLE_H, SCR_W, BAR_AREA_H, C_BG);
    memset(prevH,   0, sizeof(prevH));
    memset(prevP,   0, sizeof(prevP));
    memset(prevMH,  0, sizeof(prevMH));
    memset(prevRad, 0, sizeof(prevRad));
    prevLvlPx = 0;
    prevCR    = 0;
    flashAmt  = 0.0f;
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    // ── TFT ─────────────────────────────────────────────────────
    pinMode(PIN_TFT_BLK, OUTPUT);
    digitalWrite(PIN_TFT_BLK, HIGH);
    tftSPI.begin(PIN_TFT_CLK, -1, PIN_TFT_MOSI, PIN_TFT_CS);
    tft.init(240, 320);
    tft.setRotation(3);       // landscape, matches diagnostics sketch
    tft.fillScreen(C_BLK);

    // ── LED ─────────────────────────────────────────────────────
    led.begin();
    led.setBrightness(LED_BRIGHT);
    led.show();

    // ── Precompute lookup tables ─────────────────────────────────
    buildBandMap();
    buildRadialMap();

    // ── Boot animation ──────────────────────────────────────────
    bootAnim();

    // ── Draw initial UI ─────────────────────────────────────────
    tft.fillScreen(C_BG);
    drawChrome();

    // ── Launch audio/FFT task on Core 0, priority 2 ─────────────
    mtx = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 2, nullptr, 0);

    modeTs = millis();
    Serial.println("[VIZ] Ready — speak or play music near the INMP441");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP  — Core 1  (display + LED)
// ═══════════════════════════════════════════════════════════════
void loop() {
    // ── Fetch latest audio data from Core 0 ──────────────────────
    if (xSemaphoreTake(mtx, 8) == pdTRUE) {
        for (int b = 0; b < NUM_BANDS; b++) {
            dBand[b] = shBand[b];
            dPeak[b] = shPeak[b];
        }
        dLevel = shLevel;
        dBass  = shBass;
        dBeat  = shBeat;
        xSemaphoreGive(mtx);
    }

    // ── Advance global hue (drives animated colour palette) ──────
    hueBase = fmodf(hueBase + 0.55f, 360.0f);

    // ── Auto-cycle modes every MODE_SEC seconds ──────────────────
    if ((millis() - modeTs) > (uint32_t)MODE_SEC * 1000UL) {
        vizMode = (vizMode + 1) % 3;
        ledMode = (ledMode + 1) % 4;
        modeTs  = millis();
        clearBarArea();
        drawChrome();        // update mode label in header/footer
        Serial.printf("[VIZ] → viz=%d  led=%d\n", vizMode, ledMode);
    }

    // ── Render visualizer frame ──────────────────────────────────
    switch (vizMode) {
        case 0: drawSpectrum(); break;
        case 1: drawMirror();   break;
        case 2: drawRadial();   break;
    }

    // ── Overlays ────────────────────────────────────────────────
    handleBeatFlash();
    drawLevelMeter();

    // ── LED party light effect ───────────────────────────────────
    updateLED();

    // ~60 fps target — actual frame rate is TFT-SPI limited (~20–40 fps)
    delay(16);
}
