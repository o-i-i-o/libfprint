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

#include "goodix5042-proto.h"

#include <string.h>

guint8 *
g5042_proto_build (guint8        cmd_byte,
                   const guint8 *payload,
                   gsize         payload_len,
                   gsize        *out_len)
{
  gsize msg_len = 3 + payload_len + 1;
  g_autofree guint8 *msg = g_malloc (msg_len);
  guint16 size = payload_len + 1;
  guint sum = 0;
  gsize chunks, src, dst;
  guint8 *out;

  msg[0] = cmd_byte;
  msg[1] = size & 0xff;
  msg[2] = size >> 8;
  if (payload_len)
    memcpy (msg + 3, payload, payload_len);
  for (gsize i = 0; i < msg_len - 1; i++)
    sum += msg[i];
  msg[msg_len - 1] = (0xaa - sum) & 0xff;

  if (msg_len <= G5042_CHUNK)
    chunks = 1;
  else
    chunks = 1 + (msg_len - G5042_CHUNK + 62) / 63;

  out = g_malloc0 (chunks * G5042_CHUNK);
  memcpy (out, msg, MIN (msg_len, G5042_CHUNK));
  src = G5042_CHUNK;
  dst = G5042_CHUNK;
  while (src < msg_len)
    {
      gsize n = MIN (63, msg_len - src);

      out[dst] = cmd_byte | 1;
      memcpy (out + dst + 1, msg + src, n);
      src += n;
      dst += G5042_CHUNK;
    }

  *out_len = chunks * G5042_CHUNK;
  return out;
}

void
g5042_rx_reset (G5042Rx *rx)
{
  if (rx->buf == NULL)
    rx->buf = g_malloc (G5042_RX_BUF_SIZE);
  rx->len = 0;
  rx->expected = 0;
  rx->cmd_byte = 0;
}

void
g5042_rx_free (G5042Rx *rx)
{
  g_clear_pointer (&rx->buf, g_free);
  rx->len = rx->expected = 0;
}

gboolean
g5042_rx_feed (G5042Rx      *rx,
               const guint8 *chunk,
               gsize         len)
{
  if (len == 0)
    return TRUE;

  if (rx->len == 0)
    {
      if (len < 3)
        return FALSE;
      rx->cmd_byte = chunk[0];
      rx->expected = 3 + (gsize) (chunk[1] | (chunk[2] << 8));
      if (rx->expected > G5042_RX_BUF_SIZE)
        return FALSE;
      memcpy (rx->buf, chunk, len);
      rx->len = len;
      return TRUE;
    }

  if ((chunk[0] & 1) == 0 || (chunk[0] & 0xfe) != rx->cmd_byte)
    return FALSE;
  if (rx->len + len - 1 > G5042_RX_BUF_SIZE)
    return FALSE;
  memcpy (rx->buf + rx->len, chunk + 1, len - 1);
  rx->len += len - 1;
  return TRUE;
}

gboolean
g5042_rx_complete (const G5042Rx *rx)
{
  return rx->expected != 0 && rx->len >= rx->expected;
}

gboolean
g5042_rx_parse (const G5042Rx *rx,
                guint8        *cmd_byte,
                const guint8 **payload,
                gsize         *payload_len,
                gboolean      *checksum_ok)
{
  guint sum = 0;
  guint8 cs;

  if (!g5042_rx_complete (rx) || rx->expected < 4)
    return FALSE;

  cs = rx->buf[rx->expected - 1];
  for (gsize i = 0; i < rx->expected - 1; i++)
    sum += rx->buf[i];

  if (cmd_byte)
    *cmd_byte = rx->cmd_byte;
  if (payload)
    *payload = rx->buf + 3;
  if (payload_len)
    *payload_len = rx->expected - 4;
  if (checksum_ok)
    *checksum_ok = cs == 0x88 || cs == ((0xaa - sum) & 0xff);
  return TRUE;
}
