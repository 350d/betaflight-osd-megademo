/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Pro Tracker 3 / Vortex Tracker II module player, written from the format: a pattern line
 * decoder and the per-tick register builder of the Z80 player (sample / ornament steps, tone,
 * amplitude, noise and envelope slides, glissando, portamento, on/off). The note and volume
 * tables follow Vince Weaver's 6502 PT3 library (0BSD): the 3.4+ note tables built from one
 * octave, halved down and corrected by a bit mask; 3.3 modules use the 3.4 tables except
 * table 0.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_CHIPTUNE

#include "io/pt3.h"

#define PT3_HDR_VERSION   13
#define PT3_HDR_TABLE     0x63
#define PT3_HDR_SPEED     0x64
#define PT3_HDR_POSITIONS 0x65
#define PT3_HDR_LOOP      0x66
#define PT3_HDR_PATTERNS  0x67
#define PT3_HDR_SAMPLES   0x69
#define PT3_HDR_ORNAMENTS 0xA9
#define PT3_HDR_ORDER     0xC9

static const uint8_t pt3Volume33[16][16] = {
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2 },
    { 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3 },
    { 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4 },
    { 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5 },
    { 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6 },
    { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7 },
    { 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8 },
    { 0, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 8, 8, 9 },
    { 0, 0, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10 },
    { 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 11 },
    { 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12 },
    { 0, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13 },
    { 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
};
static const uint8_t pt3Volume35[16][16] = {
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2 },
    { 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3 },
    { 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4 },
    { 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5 },
    { 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6 },
    { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7 },
    { 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8 },
    { 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9 },
    { 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10 },
    { 0, 1, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 10, 10, 11 },
    { 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12 },
    { 0, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 12, 13 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
};

// One octave (C-1 .. B-1) of tone periods per table; the octaves below by halving, then the
// bits of the mask (LSB = octave 0) add one to that octave's low byte.
static const uint16_t pt3Base[4][12] = {
    { 0xC22, 0xB73, 0xACF, 0xA33, 0x9A1, 0x917, 0x894, 0x819, 0x7A4, 0x737, 0x6CF, 0x66D },
    { 0xEF8, 0xE10, 0xD60, 0xC80, 0xBD8, 0xB28, 0xA88, 0x9F0, 0x960, 0x8E0, 0x858, 0x7E0 },
    { 0xD10, 0xC55, 0xBA4, 0xAFC, 0xA5F, 0x9CA, 0x93D, 0x8B8, 0x83B, 0x7C5, 0x755, 0x6EC },
    { 0xCDA, 0xC22, 0xB73, 0xACF, 0xA33, 0x9A1, 0x917, 0x894, 0x819, 0x7A4, 0x737, 0x6CF },
};
static const uint16_t pt3Base0V3[12] = {
    0xC21, 0xB73, 0xACE, 0xA33, 0x9A0, 0x916, 0x893, 0x818, 0x7A4, 0x736, 0x6CE, 0x66D,
};
static const uint8_t pt3Adjust[4][12] = {
    { 0x40, 0xE6, 0x9C, 0x66, 0x40, 0x2C, 0x20, 0x30, 0x48, 0x6C, 0x1C, 0x5A },
    { 0 },
    { 0x20, 0xA8, 0x40, 0xF8, 0xBC, 0x90, 0x78, 0x70, 0x74, 0x08, 0x2A, 0x50 },
    { 0xB4, 0x40, 0xE6, 0x9C, 0x66, 0x40, 0x2C, 0x20, 0x30, 0x48, 0x6C, 0x1C },
};

static void pt3MakeNoteTable(pt3Song_t *s, uint8_t table)
{
    const uint16_t *base = (table == 0 && s->version <= 3) ? pt3Base0V3 : pt3Base[table];
    uint16_t *t = s->noteTable;
    memcpy(t, base, 12 * sizeof(uint16_t));
    for (unsigned i = 0; i < 84; i++) {
        t[i + 12] = t[i] >> 1;
    }
    if (table == 1) {
        t[23] = 0x3FD;
        t[46] = (t[46] & 0xFF00u) | ((t[46] - 1u) & 0xFFu);
    } else if (!(table == 0 && s->version <= 3)) {
        for (unsigned x = 0; x < 12; x++) {
            uint8_t m = pt3Adjust[table][x];
            for (unsigned k = x; m; k += 12, m >>= 1) {
                if (m & 1u) {
                    t[k] = (t[k] & 0xFF00u) | ((t[k] + 1u) & 0xFFu); // the low byte only
                }
            }
        }
    }
}

static uint8_t pt3Byte(const pt3Song_t *s, uint32_t at)
{
    return at < s->len ? s->data[at] : 0;
}

static uint16_t pt3Word(const pt3Song_t *s, uint32_t at)
{
    return (uint16_t)(pt3Byte(s, at) | (pt3Byte(s, at + 1) << 8));
}

static void pt3SetSample(pt3Song_t *s, pt3Channel_t *c, uint8_t n)
{
    const uint16_t p = pt3Word(s, PT3_HDR_SAMPLES + 2u * (n & 31u));
    c->sampleNum = n;
    c->sampleLoop = pt3Byte(s, p);
    c->sampleLen = pt3Byte(s, p + 1u);
    c->sample = (uint16_t)(p + 2u);
}

static void pt3SetOrnament(pt3Song_t *s, pt3Channel_t *c, uint8_t n)
{
    const uint16_t p = pt3Word(s, PT3_HDR_ORNAMENTS + 2u * (n & 15u));
    c->ornamentLoop = pt3Byte(s, p);
    c->ornamentLen = pt3Byte(s, p + 1u);
    c->ornament = (uint16_t)(p + 2u);
    c->ornamentPos = 0;
}

static void pt3SetEnvelope(pt3Song_t *s, uint8_t type, uint16_t at)
{
    s->envType = type;
    s->envBase = (uint16_t)((pt3Byte(s, at) << 8) | pt3Byte(s, at + 1u));
    s->envSlide = 0;
    s->envDelayLeft = 0;
}

bool pt3Load(pt3Song_t *s, const uint8_t *data, uint32_t len)
{
    memset(s, 0, sizeof(*s));
    if (len <= PT3_HDR_ORDER) {
        return false;
    }
    s->data = data;
    s->len = len;
    const uint8_t v = (uint8_t)(data[PT3_HDR_VERSION] - '0');
    s->version = (v < 9) ? v : 6;
    s->speed = data[PT3_HDR_SPEED];
    s->positions = data[PT3_HDR_POSITIONS];
    s->loop = data[PT3_HDR_LOOP];
    pt3MakeNoteTable(s, data[PT3_HDR_TABLE] < 3 ? data[PT3_HDR_TABLE] : 3);
    s->volumeTable = (s->version <= 4) ? pt3Volume33 : pt3Volume35;
    s->envType = 0xFF;
    for (unsigned i = 0; i < 3; i++) {
        pt3Channel_t *c = &s->ch[i];
        pt3SetOrnament(s, c, 0);
        pt3SetSample(s, c, 1);
        c->volume = 15;
        c->skip = 1;
        c->skipLeft = 1;
    }
    pt3SetPosition(s, 0);
    return s->positions > 0;
}

void pt3SetPosition(pt3Song_t *s, uint8_t position)
{
    const uint16_t table = pt3Word(s, PT3_HDR_PATTERNS);
    const uint8_t entry = pt3Byte(s, PT3_HDR_ORDER + position); // pattern × 3
    for (unsigned i = 0; i < 3; i++) {
        s->ch[i].pattern = pt3Word(s, table + 2u * entry + 2u * i);
    }
    s->noiseBase = 0;
}

static uint16_t pt3Note(const pt3Song_t *s, int note)
{
    return s->noteTable[note < 0 ? 0 : (note > 95 ? 95 : note)];
}

static void pt3NoteReset(pt3Channel_t *c)
{
    c->samplePos = 0;
    c->ampSliding = 0;
    c->noiseSliding = 0;
    c->envSliding = 0;
    c->ornamentPos = 0;
    c->toneSlideLeft = 0;
    c->toneSliding = 0;
    c->toneAcc = 0;
    c->onOffLeft = 0;
}

// One row of a channel: commands up to a note, a rest or an empty row, then the effect
// parameters in the order the effects came.
static void pt3Interpret(pt3Song_t *s, pt3Channel_t *c)
{
    const uint8_t prevNote = c->note;
    const int16_t prevSliding = c->toneSliding;
    uint8_t effects[8];
    uint8_t nEffects = 0;
    uint16_t p = c->pattern;
    c->newNote = false;
    for (bool done = false; !done; p++) {
        const uint8_t b = pt3Byte(s, p);
        if (b >= 0xF0) {
            pt3SetOrnament(s, c, (uint8_t)(b - 0xF0));
            pt3SetSample(s, c, (uint8_t)(pt3Byte(s, ++p) / 2u));
            c->envelope = false;
        } else if (b >= 0xD1) {
            pt3SetSample(s, c, (uint8_t)(b - 0xD0));
        } else if (b == 0xD0) {
            done = true;
        } else if (b >= 0xC1) {
            c->volume = (uint8_t)(b - 0xC0);
        } else if (b == 0xC0) {
            pt3NoteReset(c);
            c->enabled = false;
            done = true;
        } else if (b >= 0xB2) {
            c->envelope = true;
            pt3SetEnvelope(s, (uint8_t)(b - 0xB1), (uint16_t)(p + 1u));
            p = (uint16_t)(p + 2u);
            c->ornamentPos = 0;
        } else if (b == 0xB1) {
            c->skip = pt3Byte(s, ++p);
        } else if (b == 0xB0) {
            c->envelope = false;
            c->ornamentPos = 0;
        } else if (b >= 0x50) {
            c->note = (uint8_t)(b - 0x50);
            pt3NoteReset(c);
            c->enabled = true;
            c->newNote = true;
            done = true;
        } else if (b >= 0x40) {
            pt3SetOrnament(s, c, (uint8_t)(b - 0x40));
        } else if (b >= 0x20) {
            s->noiseBase = (uint8_t)(b - 0x20);
        } else if (b >= 0x10) {
            if (b == 0x10) {
                c->envelope = false;
            } else {
                pt3SetEnvelope(s, (uint8_t)(b - 0x10), (uint16_t)(p + 1u));
                p = (uint16_t)(p + 2u);
                c->envelope = true;
            }
            pt3SetSample(s, c, (uint8_t)(pt3Byte(s, ++p) / 2u));
            c->ornamentPos = 0;
        } else if (b >= 1 && b <= 9 && b != 6 && b != 7 && nEffects < sizeof(effects)) {
            effects[nEffects++] = b;
        }
    }
    // Effect parameters, last effect first
    while (nEffects) {
        switch (effects[--nEffects]) {
        case 1: // glissando
            c->toneSlideDelay = pt3Byte(s, p);
            c->toneSlideLeft = c->toneSlideDelay;
            c->toneSlideStep = (int16_t)pt3Word(s, p + 1u);
            p = (uint16_t)(p + 3u);
            c->simpleGliss = true;
            c->onOffLeft = 0;
            if (c->toneSlideLeft == 0 && s->version >= 7) {
                c->toneSlideLeft++;
            }
            break;
        case 2: { // portamento to the note just set
            c->simpleGliss = false;
            c->onOffLeft = 0;
            c->toneSlideDelay = pt3Byte(s, p);
            c->toneSlideLeft = c->toneSlideDelay;
            const int16_t step = (int16_t)pt3Word(s, p + 3u);
            c->toneSlideStep = (int16_t)(step < 0 ? -step : step);
            p = (uint16_t)(p + 5u);
            c->toneDelta = (int16_t)(pt3Note(s, c->note) - pt3Note(s, prevNote));
            c->slideToNote = c->note;
            c->note = prevNote;
            if (s->version >= 6) {
                c->toneSliding = prevSliding;
            }
            if (c->toneDelta - c->toneSliding < 0) {
                c->toneSlideStep = (int16_t)-c->toneSlideStep;
            }
            break;
        }
        case 3: // sample position
            c->samplePos = pt3Byte(s, p++);
            break;
        case 4: // ornament position
            c->ornamentPos = pt3Byte(s, p++);
            break;
        case 5: // on / off
            c->onOffDelay = pt3Byte(s, p);
            c->offOnDelay = pt3Byte(s, p + 1u);
            c->onOffLeft = c->onOffDelay;
            p = (uint16_t)(p + 2u);
            c->toneSlideLeft = 0;
            c->toneSliding = 0;
            break;
        case 8: // envelope slide
            s->envDelay = pt3Byte(s, p);
            s->envDelayLeft = s->envDelay;
            s->envSlideAdd = (int16_t)pt3Word(s, p + 1u);
            p = (uint16_t)(p + 3u);
            break;
        case 9: // speed
            s->speed = pt3Byte(s, p++);
            break;
        default:
            break;
        }
    }
    c->pattern = p;
    c->skipLeft = c->skip;
}

bool pt3DecodeLine(pt3Song_t *s)
{
    if (s->ch[0].skipLeft == 1 && pt3Byte(s, s->ch[0].pattern) == 0) {
        return true; // channel A ends the pattern
    }
    for (unsigned i = 0; i < 3; i++) {
        pt3Channel_t *c = &s->ch[i];
        c->newNote = false;
        if (--c->skipLeft == 0) {
            pt3Interpret(s, c);
        }
    }
    return false;
}

// One tick of a channel: tone, amplitude and its noise / envelope offset from the sample and
// ornament step, the slides; mixer bits into *mixer (tone bit 3, noise bit 6, shifted down).
static void pt3ChannelTick(pt3Song_t *s, pt3Channel_t *c, int8_t *envAdd, uint8_t *mixer)
{
    if (c->enabled) {
        const uint32_t at = c->sample + 4u * c->samplePos;
        const uint8_t b0 = pt3Byte(s, at);
        const uint8_t b1 = pt3Byte(s, at + 1u);
        uint16_t tone = (uint16_t)(pt3Word(s, at + 2u) + c->toneAcc);
        if (b1 & 0x40u) {
            c->toneAcc = tone;
        }
        const int note = (int)c->note + (int8_t)pt3Byte(s, c->ornament + c->ornamentPos);
        c->tone = (uint16_t)((tone + c->toneSliding + pt3Note(s, note)) & 0xFFFu);
        if (c->toneSlideLeft > 0 && --c->toneSlideLeft == 0) {
            c->toneSliding = (int16_t)(c->toneSliding + c->toneSlideStep);
            c->toneSlideLeft = c->toneSlideDelay;
            if (!c->simpleGliss
                && ((c->toneSlideStep < 0 && c->toneSliding <= c->toneDelta)
                    || (c->toneSlideStep >= 0 && c->toneSliding >= c->toneDelta))) {
                c->note = c->slideToNote;
                c->toneSlideLeft = 0;
                c->toneSliding = 0;
            }
        }
        if (b0 & 0x80u) {
            if (b0 & 0x40u) {
                if (c->ampSliding < 15) {
                    c->ampSliding++;
                }
            } else if (c->ampSliding > -15) {
                c->ampSliding--;
            }
        }
        int amp = (int)(b1 & 0x0Fu) + c->ampSliding;
        amp = amp < 0 ? 0 : (amp > 15 ? 15 : amp);
        c->amplitude = s->volumeTable[c->volume][amp];
        if (!(b0 & 1u) && c->envelope) {
            c->amplitude |= 0x10u;
        }
        if (b1 & 0x80u) {
            const int8_t e = (int8_t)(((b0 & 0x20u) ? ((b0 >> 1) | 0xF0u) : ((b0 >> 1) & 0x0Fu)) + (uint8_t)c->envSliding);
            if (b1 & 0x20u) {
                c->envSliding = e;
            }
            *envAdd = (int8_t)(*envAdd + e);
        } else {
            s->noiseAdd = (uint8_t)((b0 >> 1) + (uint8_t)c->noiseSliding);
            if (b1 & 0x20u) {
                c->noiseSliding = (int8_t)s->noiseAdd;
            }
        }
        *mixer |= (uint8_t)((b1 >> 1) & 0x48u);
        if (++c->samplePos >= c->sampleLen) {
            c->samplePos = c->sampleLoop;
        }
        if (++c->ornamentPos >= c->ornamentLen) {
            c->ornamentPos = c->ornamentLoop;
        }
    } else {
        c->amplitude = 0;
    }
    *mixer >>= 1;
    if (c->onOffLeft > 0 && --c->onOffLeft == 0) {
        c->enabled = !c->enabled;
        c->onOffLeft = c->enabled ? c->onOffDelay : c->offOnDelay;
    }
}

void pt3MakeFrame(pt3Song_t *s, uint8_t regs[16])
{
    int8_t envAdd = 0;
    uint8_t mixer = 0;
    for (unsigned i = 0; i < 3; i++) {
        pt3ChannelTick(s, &s->ch[i], &envAdd, &mixer);
    }
    for (unsigned i = 0; i < 3; i++) {
        regs[2 * i] = (uint8_t)s->ch[i].tone;
        regs[2 * i + 1] = (uint8_t)(s->ch[i].tone >> 8);
        regs[8 + i] = s->ch[i].amplitude;
    }
    regs[6] = (uint8_t)((s->noiseBase + s->noiseAdd) & 31u);
    regs[7] = mixer;
    const uint16_t env = (uint16_t)(envAdd + s->envSlide + s->envBase);
    regs[11] = (uint8_t)env;
    regs[12] = (uint8_t)(env >> 8);
    regs[13] = s->envType;
    s->envType = 0xFF;
    if (s->envDelayLeft > 0 && --s->envDelayLeft == 0) {
        s->envDelayLeft = s->envDelay;
        s->envSlide = (int16_t)(s->envSlide + s->envSlideAdd);
    }
}

#endif
