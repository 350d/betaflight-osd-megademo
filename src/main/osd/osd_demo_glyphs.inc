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

/* MAX7456 glyph slots used by the demo. Payloads are generated at install
 * time (osdDemoInstallFont) — no NVM tables in flash. */
#define OSD_DEMO_GLYPH_BASE   OSD_DEMO_PIXEL_OFF  // persistent effect set 0x90..0xBE
#define OSD_DEMO_GLYPH_COUNT  47
#define OSD_DEMO_GLYPH_BYTES  54
#define OSD_DEMO_ASCII_FIRST  0x20  // ASCII range kept at its standard slots
#define OSD_DEMO_ASCII_LAST   0x5F
