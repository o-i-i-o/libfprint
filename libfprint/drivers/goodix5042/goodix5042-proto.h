/*
 * Goodix 5042 (BENSS UA002) driver for libfprint — wire protocol
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

/* Message: [cmd_byte][size LE16][payload][checksum]
 *   cmd_byte = category << 4 | command << 1, bit 0 = continuation chunk
 *   size     = len(payload) + 1
 *   checksum = (0xAA - sum(all previous bytes)) & 0xff, 0x88 = not checked
 * Messages travel in 64-byte USB chunks; continuation chunks start with
 * cmd_byte | 1 and carry 63 more bytes. */
#define G5042_CHUNK 64
#define G5042_RX_BUF_SIZE (20 * 1024)      /* image reply is 17965 bytes */

#define G5042_CMD(cat, cmd)  ((guint8) (((cat) << 4) | ((cmd) << 1)))
#define G5042_CAT(b)         ((b) >> 4)
#define G5042_SUB(b)         (((b) & 0x0f) >> 1)

typedef struct
{
  guint8 *buf;
  gsize   len;
  gsize   expected;
  guint8  cmd_byte;
} G5042Rx;

/* Build the chunked, zero-padded USB buffer for one message */
guint8  *g5042_proto_build (guint8        cmd_byte,
                            const guint8 *payload,
                            gsize         payload_len,
                            gsize        *out_len);

void     g5042_rx_reset (G5042Rx *rx);
void     g5042_rx_free (G5042Rx *rx);
gboolean g5042_rx_feed (G5042Rx      *rx,
                        const guint8 *chunk,
                        gsize         len);
gboolean g5042_rx_complete (const G5042Rx *rx);

/* Payload of the reassembled message (without size and checksum) */
gboolean g5042_rx_parse (const G5042Rx *rx,
                         guint8        *cmd_byte,
                         const guint8 **payload,
                         gsize         *payload_len,
                         gboolean      *checksum_ok);
