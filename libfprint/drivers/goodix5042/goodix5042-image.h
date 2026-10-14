/*
 * Goodix 5042 (BENSS UA002) driver for libfprint — image handling
 *
 * Pure functions with no libfprint dependency.
 *
 * Copyright (C) 2026 Marco Ziliani <rylos78@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

#include <glib.h>

/*
 * Wire layout (chip id 0x2208):
 *
 *   payload = 5-byte header | 176 lines x 102 bytes | CRC32 (4 bytes)
 *
 * Each 102-byte line packs 54 pixels in 82 bytes (Goodix 12-bit, 6 bytes ->
 * 4 pixels, last group 4 bytes -> 2 pixels) followed by 20 padding bytes.
 * Because 102 = 17 * 6, decoding the whole line with the 6->4 rule yields 68
 * values whose first 54 are the real pixels, so the classic decoder can be
 * reused as long as the result is read as 176 lines of 68.
 *
 * Pixel 0 of every line and the first/last line are dummy (always ~0).
 */
#define G5042_HDR_BYTES 5
#define G5042_LINES 176
#define G5042_LINE_BYTES 102
#define G5042_LINE_DECODED 68         /* values per decoded line */
#define G5042_LINE_PIXELS 54          /* real pixels per line */
#define G5042_RAW_BYTES (G5042_LINES * G5042_LINE_BYTES)          /* 17952 */
#define G5042_CRC_BYTES 4
#define G5042_RAW_PIXELS (G5042_LINES * G5042_LINE_DECODED)       /* 11968 */

/* Active area handed to the matcher: lines 1..174, pixels 1..52 */
#define G5042_ACT_FIRST_LINE 1
#define G5042_ACT_FIRST_PIX 1
#define G5042_ACT_W 174               /* along the 176 lines */
#define G5042_ACT_H 52                /* along the 54 pixels */
#define G5042_ACT_PIXELS (G5042_ACT_W * G5042_ACT_H)

/* Decode the image part of a GET_IMAGE reply payload (header included).
 * Returns G5042_RAW_PIXELS values (176 lines x 68) or NULL if too short.
 * *crc_ok is set to TRUE/FALSE when the CRC trailer is present. */
guint16 *g5042_decode_image (const guint8 *payload,
                             gsize         payload_len,
                             gboolean     *crc_present,
                             gboolean     *crc_ok);

/* CRC-32/MPEG-2 as used by the firmware (poly 0x04c11db7, init ~0, no
 * reflection, no final xor). */
guint32 g5042_crc32_mpeg2 (const guint8 *data,
                           gsize         len);

/* Mean of the active area */
double g5042_active_mean (const guint16 *raw);

typedef enum {
  G5042_TOUCH_NONE,    /* no change against the base */
  G5042_TOUCH_FINGER,  /* a finger covers part of the sensor */
  G5042_TOUCH_DRIFT,   /* uniform shift without a finger: stale base */
  G5042_TOUCH_BRIGHT,  /* frame brighter than the base: the base was
                        * taken with something on the sensor */
} G5042Touch;

/* Classify @frame against the "no finger" @base (both G5042_RAW_PIXELS).
 * Works on the relative contact (base - frame) / base averaged over 6x6
 * blocks: on recorded touches a finger covered at least 30% of the blocks,
 * consecutive empty frames at most 1%. */
G5042Touch  g5042_touch_classify (const guint16 *base,
                                  const guint16 *frame,
                                  guint         *touched_pct);

const char *g5042_touch_str (G5042Touch t);

/* Spread (standard deviation) of the active area */
double g5042_active_spread (const guint16 *raw);
