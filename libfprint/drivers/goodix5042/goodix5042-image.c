/*
 * Goodix 5042 (BENSS UA002) driver for libfprint — image handling
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

#include "goodix5042-image.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

guint32
g5042_crc32_mpeg2 (const guint8 *data,
                   gsize         len)
{
  static guint32 table[256];
  static gsize table_init = 0;
  guint32 crc = 0xffffffff;

  if (g_once_init_enter (&table_init))
    {
      for (guint32 i = 0; i < 256; i++)
        {
          guint32 c = i << 24;

          for (int k = 0; k < 8; k++)
            c = (c & 0x80000000) ? (c << 1) ^ 0x04c11db7 : (c << 1);
          table[i] = c;
        }
      g_once_init_leave (&table_init, 1);
    }

  for (gsize i = 0; i < len; i++)
    crc = table[(crc >> 24) ^ data[i]] ^ (crc << 8);

  return crc;
}

guint16 *
g5042_decode_image (const guint8 *payload,
                    gsize         payload_len,
                    gboolean     *crc_present,
                    gboolean     *crc_ok)
{
  const guint8 *data;
  guint16 *img;
  gsize n = 0;

  if (crc_present)
    *crc_present = FALSE;
  if (crc_ok)
    *crc_ok = FALSE;

  if (payload_len < G5042_HDR_BYTES + G5042_RAW_BYTES)
    return NULL;

  data = payload + G5042_HDR_BYTES;

  if (payload_len >= G5042_HDR_BYTES + G5042_RAW_BYTES + G5042_CRC_BYTES)
    {
      /* the stored value is
       * b[n-2]<<24 | b[n-1]<<16 | b[n-4]<<8 | b[n-3] over n = data + CRC */
      const guint8 *t = data + G5042_RAW_BYTES;
      guint32 stored = ((guint32) t[2] << 24) | ((guint32) t[3] << 16) |
                       ((guint32) t[0] << 8) | t[1];

      if (crc_present)
        *crc_present = TRUE;
      if (crc_ok)
        *crc_ok = stored == g5042_crc32_mpeg2 (data, G5042_RAW_BYTES);
    }

  img = g_new (guint16, G5042_RAW_PIXELS);
  for (gsize i = 0; i + 5 < G5042_RAW_BYTES; i += 6)
    {
      const guint8 *c = data + i;

      img[n++] = ((c[0] & 0x0f) << 8) | c[1];
      img[n++] = ((guint16) c[3] << 4) | (c[0] >> 4);
      img[n++] = ((c[5] & 0x0f) << 8) | c[2];
      img[n++] = ((guint16) c[4] << 4) | (c[5] >> 4);
    }

  return img;
}

double
g5042_active_mean (const guint16 *raw)
{
  double sum = 0;

  for (int l = 0; l < G5042_ACT_W; l++)
    for (int p = 0; p < G5042_ACT_H; p++)
      sum += raw[(l + G5042_ACT_FIRST_LINE) * G5042_LINE_DECODED +
                 p + G5042_ACT_FIRST_PIX];

  return sum / G5042_ACT_PIXELS;
}


double
g5042_active_spread (const guint16 *raw)
{
  double mean = g5042_active_mean (raw), sum = 0;

  for (int l = 0; l < G5042_ACT_W; l++)
    for (int p = 0; p < G5042_ACT_H; p++)
      {
        double d = raw[(l + G5042_ACT_FIRST_LINE) * G5042_LINE_DECODED +
                       p + G5042_ACT_FIRST_PIX] - mean;

        sum += d * d;
      }

  return sqrt (sum / G5042_ACT_PIXELS);
}

#define TC_BLOCK 6
#define TC_BX (G5042_ACT_W / TC_BLOCK)               /* 29 along the lines */
#define TC_BY 8                                      /* 48 of the 52 pixels */
#define TC_FIRST_PIX (G5042_ACT_FIRST_PIX + 2)
#define TC_CONTACT 0.15          /* block mean contact of a finger */
#define TC_BRIGHT -0.05          /* block brighter than the base */
#define TC_FINGER_PCT 10
#define TC_BRIGHT_PCT 10
#define TC_DRIFT 0.05            /* mean contact of a uniform shift */

G5042Touch
g5042_touch_classify (const guint16 *base,
                      const guint16 *frame,
                      guint         *touched_pct)
{
  guint touched = 0, bright = 0;
  double total = 0;

  for (int bx = 0; bx < TC_BX; bx++)
    for (int by = 0; by < TC_BY; by++)
      {
        double s = 0;

        for (int l = 0; l < TC_BLOCK; l++)
          for (int p = 0; p < TC_BLOCK; p++)
            {
              guint i = (bx * TC_BLOCK + l + G5042_ACT_FIRST_LINE) * G5042_LINE_DECODED +
                        by * TC_BLOCK + p + TC_FIRST_PIX;

              s += ((double) base[i] - frame[i]) / MAX (base[i], 1);
            }
        s /= TC_BLOCK * TC_BLOCK;
        total += s;
        touched += s > TC_CONTACT;
        bright += s < TC_BRIGHT;
      }

  touched = touched * 100 / (TC_BX * TC_BY);
  bright = bright * 100 / (TC_BX * TC_BY);
  total /= TC_BX * TC_BY;
  if (touched_pct)
    *touched_pct = touched;

  if (touched >= TC_FINGER_PCT)
    return G5042_TOUCH_FINGER;
  if (bright >= TC_BRIGHT_PCT)
    return G5042_TOUCH_BRIGHT;
  if (ABS (total) >= TC_DRIFT)
    return G5042_TOUCH_DRIFT;
  return G5042_TOUCH_NONE;
}

const char *
g5042_touch_str (G5042Touch t)
{
  switch (t)
    {
    case G5042_TOUCH_NONE:
      return "none";

    case G5042_TOUCH_FINGER:
      return "finger";

    case G5042_TOUCH_DRIFT:
      return "drift";

    case G5042_TOUCH_BRIGHT:
      return "bright";
    }
  return "?";
}
