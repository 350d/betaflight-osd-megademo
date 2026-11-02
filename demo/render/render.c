// Host render of the OSD demo: the firmware's demo code (included whole, so the engine's statics
// are at hand) on a model of the AT7456E, driven field by field on a virtual clock, with the AY
// music from the firmware's own PT3 player and synth. Video: raw grey frames (360 × 288, one a
// field, 50 a second) to stdout; audio: s16le 48 kHz mono to the file given.
//   render <font.mcm> <audio.raw> [max seconds] > video.gray
#include "../../src/main/io/osd_demo.c"

#include <stdio.h>
#include <stdlib.h>
#include "drivers/ay_synth.h"
#include "hostenv.h"

#define OUT_W        360   // OSD pixels a line (30 cells × 12)
#define OUT_H        288   // lines a field (16 rows × 18)
#define AUDIO_HZ     48000u
#define CAMERA_PCT   45    // the intro's camera picture: plain grey (after it the camera is capped: black)
// The drone's MCU clock against real time: HAKRCF411D runs on the HSI (no SYSTEM_HSE_MHZ), ~2.05 %
// fast (measured: a video line reads 65.309 us for 64, the AY recording 1.0205 × in tempo and
// pitch). Its time (millis, micros) and its audio sample clock run that much fast; the video
// (AT7456, own crystal) does not. The firmware now measures that against the video at the demo
// start and runs the synth at the true rate, so the drone sounds nominal: 1 here. Env
// MCU_CLOCK_SCALE overrides (1.0205: the drone before that fix).
#define MCU_CLOCK_SCALE 1.0

static volatile int pendSpi5;
void hostPendIrq(int irq)
{
    if (irq == (int)SPI5_IRQn) {
        pendSpi5 = 1;
    }
}

// Betaflight's .mcm font into glyphs 0..255 (the user's font the demo backs up and restores)
static void loadMcm(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "no font %s\n", path);
        exit(1);
    }
    char line[64];
    if (!fgets(line, sizeof(line), f)) {
        exit(1);
    }
    for (int g = 0; g < 256; g++) {
        for (int b = 0; b < 64; b++) {
            if (!fgets(line, sizeof(line), f)) {
                fclose(f);
                return;
            }
            if (b < 54) {
                chip.nvm[g][b] = (uint8_t)strtol(line, NULL, 2);
            }
        }
    }
    fclose(f);
}

// One field as the chip shows it: per line the 30 codes and INV bits (the raster scenes' bands /
// bitmap lines, else display memory), the row levels, the background, HOS per line, VOS.
static uint8_t lineCode[OUT_H][30], lineInv[OUT_H][30];
static int8_t lineHos[OUT_H];
static uint8_t lineRb[OUT_H]; // RB a line (per-line copper: the slot engine's rbLine)

static void captureField(void)
{
    const bool raster = irqRunning && irqScene;
    if (raster) {
        const uint8_t bl = irqScene->bandLines ? irqScene->bandLines : 1u;
        for (uint8_t y = 0; y < 16; y++) {
            for (uint8_t b = 0; b < irqScene->bands; b++) {
                uint8_t g[30];
                memset(g, OSD_DEMO_PIXEL_OFF, sizeof(g));
                mgBandInv = 0;
                irqScene->band(y, b, 30, g);
                for (uint8_t k = 0; k < bl; k++) {
                    const uint16_t l = (uint16_t)(y * 18u + b * bl + k);
                    if (l < OUT_H) {
                        memcpy(lineCode[l], g, 30);
                        for (int x = 0; x < 30; x++) {
                            lineInv[l][x] = (uint8_t)((mgBandInv >> x) & 1u);
                        }
                    }
                }
            }
        }
    } else {
        for (int l = 0; l < OUT_H; l++) {
            memcpy(lineCode[l], &chip.code[(l / 18) * 30], 30);
            memcpy(lineInv[l], &chip.inv[(l / 18) * 30], 30);
        }
    }
    for (int l = 0; l < OUT_H; l++) {
        int hos = chip.hos;
        if (raster && irqMotion.hosLine && l > 0) {
            hos = constrain((int)irqHosBase + irqMotion.hosLine[l], 0, 63);
        }
        lineHos[l] = (int8_t)(hos - 32);
        lineRb[l] = (raster && irqMotion.rbLine && zdmaSpc >= 2u) ? irqMotion.rbLine[l] : chip.rb[l / 18];
    }
}

static uint8_t lum(int pct)
{
    const int v = pct * 255 / 120;
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static void renderField(uint8_t *out)
{
    const bool grayBg = (chip.vm1 & 0x80u) != 0;
    // the camera's picture where pixels are clear: grey in the intro, then black (the lens capped;
    // the demo runs on external sync so a grabber keeps the camera's timing)
    const int bgPct = grayBg ? ((chip.vm1 >> 4) & 7) * 7 : ((chip.syncMode == 1 || !introPhase) ? 0 : CAMERA_PCT);
    const uint8_t bg = lum(bgPct);
    const int vosOff = (int)chip.vos - 16;
    static const int whitePct[4] = { 120, 100, 90, 80 };
    for (int sy = 0; sy < OUT_H; sy++) {
        uint8_t *o = &out[sy * OUT_W];
        const int l = sy - vosOff;
        if (l < 0 || l >= OUT_H) {
            memset(o, bg, OUT_W);
            continue;
        }
        const int row = l / 18, gl = l % 18;
        const uint8_t rb = lineRb[l];
        (void)row;
        const uint8_t black = lum(((rb >> 2) & 3) * 10), white = lum(whitePct[rb & 3]);
        for (int sx = 0; sx < OUT_W; sx++) {
            const int x = sx - lineHos[l];
            if (x < 0 || x >= OUT_W) {
                o[sx] = bg;
                continue;
            }
            const int c = x / 12, px = x % 12;
            const uint8_t *g = chip.nvm[lineCode[l][c]];
            const uint8_t byte = g[gl * 3 + px / 4];
            const uint8_t v = (uint8_t)((byte >> (6 - 2 * (px % 4))) & 3u);
            if (v & 1u) {
                o[sx] = bg; // transparent
            } else {
                const bool w = (v == 2u) != (lineInv[l][c] != 0);
                o[sx] = w ? white : black;
            }
        }
    }
}

// Glyph pixels as 0 black / 1 white / 2 clear; with inv black and white swapped
static void glyphPx(uint8_t code, bool inv, uint8_t out[18][12])
{
    for (int r = 0; r < 18; r++) {
        for (int c = 0; c < 12; c++) {
            const uint8_t v = (uint8_t)((chip.nvm[code][r * 3 + c / 4] >> (6 - 2 * (c % 4))) & 3u);
            out[r][c] = (v & 1u) ? 2u : ((v == 2u) != inv ? 1u : 0u);
        }
    }
}

// Which 6x6 mask glyphs (0x3C..) another glyph of the demo font already shows (as is or INV)
static void hostCheckDups(void)
{
    // Every mask the texts use: the glyph it gets shows exactly that mask (bit 2r + c: dot row r
    // of 6 lines, dot column c of 6 px)
    for (int m = 1; m < 63; m++) {
        if (!s1MaskGlyph[m]) {
            continue;
        }
        uint8_t a[18][12];
        glyphPx((uint8_t)s1MaskGlyph[m], (s1MaskGlyph[m] >> 8) != 0, a);
        bool ok = true;
        for (int r = 0; r < 18 && ok; r++) {
            for (int c = 0; c < 12 && ok; c++) {
                ok = a[r][c] == (uint8_t)((m >> (2 * (r / 6) + c / 6)) & 1);
            }
        }
        const uint16_t code = s1MaskGlyph[m];
        const bool slot = (code & 0xFF) >= OSD_DEMO_S1_MP_BASE && (code & 0xFF) < OSD_DEMO_S1_MP_BASE + OSD_DEMO_S1_MP_SLOTS;
        fprintf(stderr, "mask 0x%02X -> 0x%02X%s %s%s\n", m, code & 0xFF, (code >> 8) ? " INV" : "",
                slot ? "slot" : "other glyph", ok ? "" : "  MISMATCH");
    }
    for (uint8_t i = 0; i < s1MpCount; i++) {
        const uint8_t g = (uint8_t)(OSD_DEMO_S1_MP_BASE + i);
        uint8_t a[18][12], b[18][12];
        glyphPx(g, false, a);
        for (int code = 0; code < 256; code++) {
            if (code == g) {
                continue;
            }
            for (int inv = 0; inv < 2; inv++) {
                glyphPx((uint8_t)code, inv != 0, b);
                if (!memcmp(a, b, sizeof(a))) {
                    fprintf(stderr, "slot 0x%02X (mask 0x%02X) = glyph 0x%02X%s\n", g, s1MpMask[i], code, inv ? " INV" : "");
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: render <font.mcm> <audio.raw> [max seconds] > video.gray\n");
        return 1;
    }
    const double maxS = (argc > 3) ? atof(argv[3]) : 900.0;
    FILE *aud = fopen(argv[2], "wb");
    if (!aud) {
        return 1;
    }
    hostChipInit();
    loadMcm(argv[1]);
    const double clk = getenv("MCU_CLOCK_SCALE") ? atof(getenv("MCU_CLOCK_SCALE")) : MCU_CLOCK_SCALE;
    // the synth believes AUDIO_HZ / clk: in real time its frames and tones run clk × fast
    aySynthConfigure((uint32_t)(AUDIO_HZ / clk + 0.5), 32767, 16, true, 7, AY_DAC_AY);
    fprintf(stderr, "MCU clock scale %.4f\n", clk);
    aySynthReset();

    hostSetTimeUs(1000000); // a second after boot
    hsCapOn = true; // the HSYNC wire on PB3, as on the board
    hsPin = &osdDemoHsPins[4];
    if (!osdDemoStartScene(0)) { // the intro, then the demo to its end
        fprintf(stderr, "start failed: %s\n", osdDemoStartLastError());
        return 1;
    }
    static uint8_t frame[OUT_W * OUT_H];
    uint64_t t = 1000000;
    double tMcu = 1000000.0; // the MCU's micros
    uint32_t fields = 0, tailFields = 0;
    double hpPrevIn = 0, hpPrevOut = 0;
    bool wasActive = true;
    while (fields < (uint32_t)(maxS * 50.0)) {
        for (int ms = 0; ms < 20; ms++) {
            t += 1000;    // real time: 48 output samples a ms
            tMcu += 1000.0 * clk;
            hostSetTimeUs((uint64_t)tMcu);
            uint16_t s[AUDIO_HZ / 1000u];
            int16_t pcm[AUDIO_HZ / 1000u];
            if (hostAudioOn) {
                aySynthRender(s, AUDIO_HZ / 1000u);
            } else {
                for (unsigned i = 0; i < AUDIO_HZ / 1000u; i++) {
                    s[i] = 16384;
                }
            }
            for (unsigned i = 0; i < AUDIO_HZ / 1000u; i++) { // DC out (the PWM duty is one-sided)
                const double in = (double)s[i];
                const double y = in - hpPrevIn + 0.9995 * hpPrevOut;
                hpPrevIn = in;
                hpPrevOut = y;
                const double v = y; // level set at the encode (peak to −1 dB)
                pcm[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
            }
            fwrite(pcm, sizeof(pcm[0]), AUDIO_HZ / 1000u, aud);
            if (osdDemoIsActive()) {
                osdDemoPoll(micros());
            }
        }
        // VSYNC: the raster engine's field hook, then the frame build it pends
        mgFieldFirst = (fields & 1u) != 0u;
        if (irqRunning) {
            osdDemoIrqVsync();
            if (pendSpi5) {
                pendSpi5 = 0;
                hostSpi5IrqHandler();
            }
        }
        if (getenv("RB_DUMP") && fields >= 3700 && fields < 3760) { fprintf(stderr, "f%u env %d..%d  ", fields, wipeEnvTop, wipeEnvBot); for (int r = 0; r < 16; r++) fputc("012"[(chip.rb[r] >> 2) & 3], stderr); fputc(10, stderr); }
        captureField();
        renderField(frame);
        fwrite(frame, 1, sizeof(frame), stdout);
        fields++;
        static bool checked;
        if (!checked && s1MpCount && getenv("OSD_CHECK_DUPS")) {
            checked = true;
            hostCheckDups();
        }
        if (!osdDemoIsActive()) {
            if (wasActive) {
                fprintf(stderr, "demo ended at field %u (%.1f s)\n", fields, fields / 50.0);
                wasActive = false;
            }
            if (++tailFields >= 50) {
                break; // a second after the end
            }
        }
        if ((fields % 500u) == 0u) {
            fprintf(stderr, "%.0f s\n", fields / 50.0);
        }
    }
    fclose(aud);
    fprintf(stderr, "fields %u; 6x6 masks in slots: %u of %u\n", fields, (unsigned)s1MpCount, (unsigned)OSD_DEMO_S1_MP_SLOTS);
    return 0;
}
