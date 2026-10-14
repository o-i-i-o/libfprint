/*
 * Goodix 5042 (BENSS UA002 USB dongle) driver for libfprint
 *
 * Plaintext Goodix protocol over the CDC-ACM data interface, worked out
 * from USB captures. The sensor only captures images: matching runs on
 * the host (goodix5042-match.c).
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

#define FP_COMPONENT "goodix5042"

#include "drivers_api.h"
#include "fpi-print.h"

#include "goodix5042.h"
#include "goodix5042-proto.h"
#include "goodix5042-image.h"
#include "goodix5042-consts.h"
#include "goodix5042-match.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------
 * Tunables
 * ------------------------------------------------------------------------ */

#define G5042_USB_INTERFACE 1
#define G5042_EP_OUT (0x03 | FPI_USB_ENDPOINT_OUT)
#define G5042_EP_IN (0x01 | FPI_USB_ENDPOINT_IN)

#define G5042_TIMEOUT_ACK 2000
#define G5042_TIMEOUT_DATA 2000
#define G5042_TIMEOUT_IMAGE 5000
#define G5042_TIMEOUT_CONT 2000      /* continuation chunks of one message */
#define G5042_FDT_POLL_MS 3000       /* re-arm FDT_DOWN after this silence */
#define G5042_TIMEOUT_LIFT 20000     /* FDT_UP is ACKed only after lift-off */
#define G5042_MAX_REARM 5            /* non-finger events before a rebase */
#define G5042_MAX_SPONTANEOUS 8
#define G5042_DRAIN_IDLE_MS 250
#define G5042_DRAIN_MAX 2000         /* chunks */
#define G5042_INIT_ATTEMPTS 3

/* The finger keeps settling for a few hundred ms after the FDT interrupt
 * (the image mean keeps dropping), and back-to-back frames are identical. */
#define G5042_SETTLE_MS 250
#define G5042_FRAME_GAP_MS 150
#define G5042_MAX_FRAMES 1           /* one frame per touch */

#define G5042_BASE_ATTEMPTS 4

/* FDT margins, in threshold units (half a reading). Empty-sensor readings
 * move by 1-2, a finger lowers them by 0x20-0x80. */
#define G5042_FDT_DOWN_MARGIN 4
#define G5042_FDT_UP_MARGIN 8
#define G5042_FDT_STEADY 4           /* base: max change between two reads */

/* pixel gain calibration (register 0x5c, see goodix5042-consts.h) */
#define G5042_GAIN_SPREAD 200.0
#define G5042_GAIN_MIN 0x00c0
#define G5042_GAIN_MAX 0x0180

/* Matching. The sensor area is small (52 x 174 px): the template keeps
 * the features of every enrollment touch, a probe is compared with each
 * of them. Thresholds from offline tests on recorded touches of 10
 * fingers: impostor scores averaged 2.6 (max 7). */
#define G5042_ENROLL_SAMPLES 15
#define G5042_MIN_COVERAGE 40        /* % of the sensor touched */
#define G5042_MATCH_SCORE 10         /* consistent keypoint matches */
#define G5042_DUP_SCORE 80           /* enrollment: same area as before */

#define G5042_PRINT_MAGIC "G5FM"
#define G5042_PRINT_VERSION 1

/* Command bytes (category << 4 | command << 1) */
#define CMD_GET_IMAGE 0x20
#define CMD_FDT_DOWN 0x32
#define CMD_FDT_UP 0x34
#define CMD_FDT_MODE 0x36
#define CMD_WRITE_REG 0x80
#define CMD_READ_REG 0x82
#define CMD_UPLOAD_CFG 0x90
#define CMD_A0 0xa0
#define CMD_RESET 0xa2
#define CMD_FW_VERSION 0xa8
#define CMD_ACK 0xb0

/* run_cmd() flags */
#define F_ACK (1 << 0)
#define F_DATA (1 << 1)
#define F_OPTIONAL (1 << 2)       /* timeout/unexpected reply -> cmd_ok FALSE */
#define F_CANCELLABLE (1 << 3)

typedef enum {
  AFTER_DONE,          /* complete now, the finger is still down */
  AFTER_DONE_WAIT_UP,  /* wait for lift-off, then complete */
  AFTER_REARM,         /* wait for lift-off, then wait for a finger again */
  AFTER_NEXT_STAGE,    /* wait for lift-off, refresh the base, next touch */
  AFTER_REBASE,        /* wait for lift-off, refresh the base, wait again */
} G5042After;

struct _FpiDeviceGoodix5042
{
  FpDevice      parent;

  GCancellable *cancel;

  /* transport */
  G5042Rx       rx;
  guint         rx_timeout;
  GCancellable *rx_cancel;
  gboolean      rx_allow_timeout;
  gboolean      rx_timed_out;
  guint         rx_skipped;
  guint         drain_chunks;

  /* current command */
  guint8   cmd_byte;
  guint8  *cmd_payload;
  gsize    cmd_len;
  guint    cmd_flags;
  guint    cmd_ack_timeout;
  guint    cmd_data_timeout;
  gboolean cmd_ok;

  /* last reply (owned copy) */
  guint8  reply_cmd;
  guint8 *reply;
  gsize   reply_len;

  /* sensor state */
  gboolean claimed;
  guint    init_step;
  guint    init_attempts;
  gboolean usb_reset_tried;
  gboolean needs_reinit;
  gchar   *fw_version;
  gboolean crc_seen_ok;
  guint8   config[256];
  guint    gain;
  gboolean gain_calibrated;

  /* "no finger" base (image and FDT readings), kept across open/close */
  guint16 *base;
  guint16 *base_cand;
  guint16  fdt_ref[12];
  gboolean fdt_ref_valid;
  guint16  fdt_a[12];
  guint    wait_events;
  guint    base_attempts;
  gboolean base_strict;
  gboolean base_comm_failed;
  guint    rearm_count;

  /* capture */
  guint16   *frames[G5042_MAX_FRAMES];
  guint      nframes;
  guint      wanted_frames;
  guint      image_retries;
  G5042After after;

  /* action */
  gboolean   result_reported;
  GPtrArray *enroll;            /* FmFeatures of the accepted touches */
  gint       enroll_stage;
  FmParams   match;
};

G_DEFINE_TYPE (FpiDeviceGoodix5042, fpi_device_goodix5042, FP_TYPE_DEVICE)

static void build_config (FpiDeviceGoodix5042 *self,
                          guint8               cfg[256]);

static void
clear_frames (FpiDeviceGoodix5042 *self)
{
  for (guint i = 0; i < G5042_MAX_FRAMES; i++)
    g_clear_pointer (&self->frames[i], g_free);
  self->nframes = 0;
}

/* ========================================================================
 * Transport
 * ======================================================================== */

static void
tx_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data,
       GError *error)
{
  if (error)
    fpi_ssm_mark_failed (transfer->ssm, error);
  else
    fpi_ssm_next_state (transfer->ssm);
}

/* Send one message; advances @ssm when the USB write completes */
static void
send_message (FpiSsm *ssm, FpDevice *dev, guint8 cmd_byte,
              const guint8 *payload, gsize len)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);
  gsize buf_len;
  guint8 *buf = g5042_proto_build (cmd_byte, payload, len, &buf_len);

  transfer->ssm = ssm;
  fpi_usb_transfer_fill_bulk_full (transfer, G5042_EP_OUT, buf, buf_len, g_free);
  fpi_usb_transfer_submit (transfer, 2000, NULL, tx_cb, NULL);
}

static void rx_cb (FpiUsbTransfer *transfer,
                   FpDevice       *dev,
                   gpointer        user_data,
                   GError         *error);

static void
rx_submit (FpiSsm *ssm, FpDevice *dev, guint timeout)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  transfer->ssm = ssm;
  fpi_usb_transfer_fill_bulk (transfer, G5042_EP_IN, G5042_CHUNK);
  fpi_usb_transfer_submit (transfer, timeout, self->rx_cancel, rx_cb, NULL);
}

/* Receive one complete message into self->reply. Unsolicited category 0xC
 * frames (seen after lift-off) are dropped. With @allow_timeout a timeout
 * sets rx_timed_out instead of failing. */
static void
recv_message (FpiSsm *ssm, FpDevice *dev, guint timeout,
              GCancellable *cancellable, gboolean allow_timeout)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  g5042_rx_reset (&self->rx);
  self->rx_timeout = timeout;
  self->rx_cancel = cancellable;
  self->rx_allow_timeout = allow_timeout;
  self->rx_timed_out = FALSE;
  self->rx_skipped = 0;
  rx_submit (ssm, dev, timeout);
}

static void
rx_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data,
       GError *error)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);
  const guint8 *payload;
  gsize payload_len;
  guint8 cmd_byte;
  gboolean cs_ok;

  if (error)
    {
      if (self->rx_allow_timeout &&
          g_error_matches (error, G_USB_DEVICE_ERROR,
                           G_USB_DEVICE_ERROR_TIMED_OUT))
        {
          g_error_free (error);
          self->rx_timed_out = TRUE;
          fpi_ssm_next_state (transfer->ssm);
          return;
        }
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (transfer->actual_length == 0)
    {
      rx_submit (transfer->ssm, dev, self->rx_timeout);
      return;
    }

  if (self->rx.len == 0 && (transfer->buffer[0] & 1))
    {
      /* stray continuation chunk (navigation stream tail): ignore */
      fp_dbg ("dropping stray chunk 0x%02x", transfer->buffer[0]);
      rx_submit (transfer->ssm, dev, self->rx_timeout);
      return;
    }

  if (!g5042_rx_feed (&self->rx, transfer->buffer, transfer->actual_length))
    {
      fp_warn ("protocol reassembly error (cmd 0x%02x), resyncing",
               self->rx.cmd_byte);
      g5042_rx_reset (&self->rx);
      rx_submit (transfer->ssm, dev, self->rx_timeout);
      return;
    }

  if (!g5042_rx_complete (&self->rx))
    {
      rx_submit (transfer->ssm, dev,
                 self->rx_timeout ? MAX (self->rx_timeout, G5042_TIMEOUT_CONT) :
                 G5042_TIMEOUT_CONT);
      return;
    }

  if (!g5042_rx_parse (&self->rx, &cmd_byte, &payload, &payload_len, &cs_ok))
    {
      fp_dbg ("dropping malformed message 0x%02x", self->rx.cmd_byte);
      g5042_rx_reset (&self->rx);
      rx_submit (transfer->ssm, dev, self->rx_timeout);
      return;
    }
  if (!cs_ok)
    fp_dbg ("checksum mismatch on 0x%02x reply", cmd_byte);

  if (G5042_CAT (cmd_byte) == 0x5 && self->rx_skipped++ < G5042_DRAIN_MAX)
    {
      /* navigation stream left running: not an answer to the command */
      g5042_rx_reset (&self->rx);
      rx_submit (transfer->ssm, dev, self->rx_timeout);
      return;
    }

  if (G5042_CAT (cmd_byte) == 0xc &&
      (self->rx_timeout == 0 || self->rx_timeout >= G5042_TIMEOUT_LIFT ||
       self->rx_skipped++ < G5042_MAX_SPONTANEOUS))
    {
      fp_dbg ("dropping spontaneous frame 0x%02x", cmd_byte);
      g5042_rx_reset (&self->rx);
      rx_submit (transfer->ssm, dev, self->rx_timeout);
      return;
    }

  g_free (self->reply);
  self->reply = g_memdup2 (payload, payload_len);
  self->reply_len = payload_len;
  self->reply_cmd = cmd_byte;
  fpi_ssm_next_state (transfer->ssm);
}

/* Read and discard everything the device sends until it is quiet for
 * G5042_DRAIN_IDLE_MS (the sensor streams navigation data after init). */
static void drain_cb (FpiUsbTransfer *transfer,
                      FpDevice       *dev,
                      gpointer        user_data,
                      GError         *error);

static void
drain_submit (FpiSsm *ssm, FpDevice *dev)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  transfer->ssm = ssm;
  fpi_usb_transfer_fill_bulk (transfer, G5042_EP_IN, G5042_CHUNK);
  fpi_usb_transfer_submit (transfer, G5042_DRAIN_IDLE_MS,
                           FPI_DEVICE_GOODIX5042 (dev)->cancel, drain_cb, NULL);
}

static void
drain_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data,
          GError *error)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  if (error)
    {
      if (g_error_matches (error, G_USB_DEVICE_ERROR,
                           G_USB_DEVICE_ERROR_TIMED_OUT))
        {
          g_error_free (error);
          fp_dbg ("drained %u chunks", self->drain_chunks);
          fpi_ssm_next_state (transfer->ssm);
          return;
        }
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (++self->drain_chunks >= G5042_DRAIN_MAX)
    {
      fp_warn ("device keeps streaming, giving up draining");
      fpi_ssm_next_state (transfer->ssm);
      return;
    }
  drain_submit (transfer->ssm, dev);
}

static void
drain (FpiSsm *ssm, FpDevice *dev)
{
  FPI_DEVICE_GOODIX5042 (dev)->drain_chunks = 0;
  drain_submit (ssm, dev);
}

/* ------------------------------------------------------------------------
 * Command sub-SSM: send -> [ACK] -> [data reply]
 * Result in self->cmd_ok and self->reply.
 * ------------------------------------------------------------------------ */

enum {
  CMD_SEND,
  CMD_RECV_ACK,
  CMD_CHECK_ACK,
  CMD_RECV_DATA,
  CMD_CHECK_DATA,
  CMD_NUM_STATES,
};

static void
cmd_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);
  gboolean optional = (self->cmd_flags & F_OPTIONAL) != 0;
  GCancellable *c = self->cancel;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case CMD_SEND:
      self->cmd_ok = FALSE;
      send_message (ssm, dev, self->cmd_byte, self->cmd_payload, self->cmd_len);
      break;

    case CMD_RECV_ACK:
      if (!(self->cmd_flags & F_ACK))
        {
          fpi_ssm_jump_to_state (ssm, CMD_RECV_DATA);
          return;
        }
      recv_message (ssm, dev, self->cmd_ack_timeout, c, optional);
      break;

    case CMD_CHECK_ACK:
      if (self->rx_timed_out)
        {
          fp_dbg ("no ACK for 0x%02x", self->cmd_byte);
          fpi_ssm_mark_completed (ssm);
          return;
        }
      if (self->reply_cmd != CMD_ACK)
        {
          if (optional)
            {
              fp_dbg ("expected ACK for 0x%02x, got 0x%02x",
                      self->cmd_byte, self->reply_cmd);
              fpi_ssm_mark_completed (ssm);
              return;
            }
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "Expected ACK for 0x%02x, got 0x%02x",
                                                         self->cmd_byte, self->reply_cmd));
          return;
        }
      if (self->reply_len >= 1 && self->reply[0] != self->cmd_byte)
        fp_dbg ("ACK echoes 0x%02x for command 0x%02x",
                self->reply[0], self->cmd_byte);
      fpi_ssm_next_state (ssm);
      break;

    case CMD_RECV_DATA:
      if (!(self->cmd_flags & F_DATA))
        {
          self->cmd_ok = TRUE;
          fpi_ssm_mark_completed (ssm);
          return;
        }
      recv_message (ssm, dev, self->cmd_data_timeout, c, optional);
      break;

    case CMD_CHECK_DATA:
      self->cmd_ok = !self->rx_timed_out;
      if (!self->cmd_ok)
        fp_dbg ("no data reply for 0x%02x", self->cmd_byte);
      fpi_ssm_mark_completed (ssm);
      break;

    }
}

static void
run_cmd_full (FpiSsm *parent, FpDevice *dev, guint8 cmd_byte,
              const guint8 *payload, gsize len, guint flags,
              guint ack_timeout, guint data_timeout)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);
  FpiSsm *sub;

  g_free (self->cmd_payload);
  self->cmd_payload = len ? g_memdup2 (payload, len) : NULL;
  self->cmd_len = len;
  self->cmd_byte = cmd_byte;
  self->cmd_flags = flags;
  self->cmd_ack_timeout = ack_timeout;
  self->cmd_data_timeout = data_timeout;

  sub = fpi_ssm_new (dev, cmd_handler, CMD_NUM_STATES);
  fpi_ssm_silence_debug (sub);
  fpi_ssm_start_subsm (parent, sub);
}

static void
run_cmd (FpiSsm *parent, FpDevice *dev, guint8 cmd_byte,
         const guint8 *payload, gsize len, guint flags)
{
  run_cmd_full (parent, dev, cmd_byte, payload, len, flags,
                G5042_TIMEOUT_ACK, G5042_TIMEOUT_DATA);
}

/* ========================================================================
 * Sensor initialisation
 * ======================================================================== */

typedef struct
{
  guint8        cmd;
  const guint8 *payload;
  gsize         len;
  guint         flags;
  gboolean      check_reply;   /* data reply carries the command byte */
} G5042Step;

static const guint8 pl_reset[] = { 0x05, 0x14 };
static const guint8 pl_zero[] = { 0x00, 0x00 };
static const guint8 pl_wr_1010[] = { 0x00, 0x08, 0x00, 0x10, 0x10 };
static const guint8 pl_rd_0006[] = { 0x00, 0x06, 0x00, 0x02, 0x00 };
static const guint8 pl_wr_1111[] = { 0x00, 0x08, 0x00, 0x11, 0x11 };
static const guint8 pl_rd_chip_id[] = { 0x00, 0x00, 0x00, 0x04, 0x00 };
static const guint8 pl_a0_1[] = { 0x00, 0x01 };
static const guint8 pl_a0_2[] = { 0x0a, 0x00 };
static const guint8 pl_image[] = { 0x01, 0x00 };

enum {
  STEP_RESET1, STEP_FW1, STEP_WR1, STEP_RD1, STEP_WR2, STEP_RESET2,
  STEP_CHIP_ID, STEP_CONFIG, STEP_FW2, STEP_A0_1, STEP_A0_2,
  NUM_INIT_STEPS,
};

/* start-up sequence as seen on the wire */
static const G5042Step init_steps[NUM_INIT_STEPS] = {
  [STEP_RESET1]  = { CMD_RESET, pl_reset, sizeof pl_reset, F_ACK | F_DATA },
  [STEP_FW1]     = { CMD_FW_VERSION, pl_zero, sizeof pl_zero, F_ACK | F_DATA, TRUE },
  [STEP_WR1]     = { CMD_WRITE_REG, pl_wr_1010, sizeof pl_wr_1010, F_ACK },
  [STEP_RD1]     = { CMD_READ_REG, pl_rd_0006, sizeof pl_rd_0006, F_ACK | F_DATA, TRUE },
  [STEP_WR2]     = { CMD_WRITE_REG, pl_wr_1111, sizeof pl_wr_1111, F_ACK },
  [STEP_RESET2]  = { CMD_RESET, pl_reset, sizeof pl_reset, F_ACK | F_DATA },
  [STEP_CHIP_ID] = { CMD_READ_REG, pl_rd_chip_id, sizeof pl_rd_chip_id, F_ACK | F_DATA, TRUE },
  [STEP_CONFIG]  = { CMD_UPLOAD_CFG, NULL, 256, F_ACK | F_DATA },
  [STEP_FW2]     = { CMD_FW_VERSION, pl_zero, sizeof pl_zero, F_ACK | F_DATA, TRUE },
  [STEP_A0_1]    = { CMD_A0, pl_a0_1, sizeof pl_a0_1, F_DATA },
  [STEP_A0_2]    = { CMD_A0, pl_a0_2, sizeof pl_a0_2, F_DATA },
};

enum {
  INIT_DRAIN,
  INIT_STEP,
  INIT_STEP_DONE,
  INIT_IDLE_DRAIN,
  INIT_GAIN_READ,
  INIT_GAIN_IMAGE,
  INIT_GAIN_DONE,
  INIT_NUM_STATES,
};

static void run_fdt_read (FpiSsm   *ssm,
                          FpDevice *dev);

/* checksum: last LE16 = -(0xa5a5 + sum of the first 127 LE16 words) */
static void
config_checksum (guint8 cfg[256])
{
  guint16 sum = 0xa5a5;

  for (int i = 0; i < 254; i += 2)
    sum += cfg[i] | (cfg[i + 1] << 8);
  sum = -sum;
  cfg[254] = sum & 0xff;
  cfg[255] = sum >> 8;
}

static void
build_config (FpiDeviceGoodix5042 *self, guint8 cfg[256])
{
  memcpy (cfg, g5042_config_template, 256);
  cfg[G5042_CFG_GAIN_OFFSET] = self->gain & 0xff;
  cfg[G5042_CFG_GAIN_OFFSET + 1] = self->gain >> 8;
  config_checksum (cfg);
}

/* Pixel gain for this unit: the spread of an empty frame scales with the
 * gain, bring it to G5042_GAIN_SPREAD (the image then uses about half of
 * the 12-bit range and a finger stays well clear of 0). */
static guint
gain_from_spread (guint gain, double spread)
{
  double g;

  if (spread < 1)
    return gain;
  g = gain * G5042_GAIN_SPREAD / spread;
  g = CLAMP (g, G5042_GAIN_MIN, G5042_GAIN_MAX);
  return ((guint) (g + 8)) & ~0xfu;
}

static void
init_handle_reply (FpiDeviceGoodix5042 *self, guint step)
{
  switch (step)
    {
    case STEP_FW1:
    case STEP_FW2:
      g_clear_pointer (&self->fw_version, g_free);
      self->fw_version = g_strndup ((const gchar *) self->reply,
                                    strnlen ((const gchar *) self->reply,
                                             self->reply_len));
      fp_dbg ("firmware: %s", self->fw_version);
      break;

    case STEP_CHIP_ID:
      if (self->reply_len >= 4)
        fp_dbg ("chip id: %02x %02x %02x %02x", self->reply[0],
                self->reply[1], self->reply[2], self->reply[3]);
      break;
    }
}

static void
init_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case INIT_DRAIN:
      /* the sensor boots in navigation mode and streams data */
      self->init_step = 0;
      if (self->gain == 0)
        self->gain = G5042_CFG_GAIN_DEFAULT;
      drain (ssm, dev);
      break;

    case INIT_STEP:
      {
        const G5042Step *s = &init_steps[self->init_step];

        if (self->init_step == STEP_CONFIG)
          {
            build_config (self, self->config);
            run_cmd (ssm, dev, s->cmd, self->config, sizeof self->config,
                     s->flags | F_OPTIONAL);
            return;
          }
        run_cmd (ssm, dev, s->cmd, s->payload, s->len, s->flags | F_OPTIONAL);
      }
      break;

    case INIT_STEP_DONE:
      if (self->cmd_ok && init_steps[self->init_step].check_reply &&
          self->reply_cmd != init_steps[self->init_step].cmd)
        {
          fp_dbg ("init step %u: unexpected reply 0x%02x", self->init_step,
                  self->reply_cmd);
          self->cmd_ok = FALSE;
        }
      if (!self->cmd_ok)
        {
          if (++self->init_attempts < G5042_INIT_ATTEMPTS)
            {
              fp_warn ("init step %u (cmd 0x%02x) failed, restarting init (%u/%u)",
                       self->init_step, init_steps[self->init_step].cmd,
                       self->init_attempts, G5042_INIT_ATTEMPTS);
              fpi_ssm_jump_to_state_delayed (ssm, INIT_DRAIN, 500);
              return;
            }
          /* the sensor MCU can stop answering altogether (seen after an
           * idle period); a USB port reset brings it back */
          if (!self->usb_reset_tried)
            {
              g_autoptr(GError) reset_error = NULL;

              self->usb_reset_tried = TRUE;
              fp_warn ("sensor not responding, resetting the USB device");
              if (g_usb_device_reset (fpi_device_get_usb_device (dev), &reset_error))
                {
                  self->init_attempts = 0;
                  g5042_rx_reset (&self->rx);
                  fpi_ssm_jump_to_state_delayed (ssm, INIT_DRAIN, 1000);
                  return;
                }
              fp_warn ("USB reset failed: %s", reset_error->message);
            }
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "Sensor init failed at step %u (cmd 0x%02x)",
                                                         self->init_step,
                                                         init_steps[self->init_step].cmd));
          return;
        }

      init_handle_reply (self, self->init_step);
      if (++self->init_step < NUM_INIT_STEPS)
        fpi_ssm_jump_to_state (ssm, INIT_STEP);
      else
        fpi_ssm_next_state_delayed (ssm, 500);
      break;

    case INIT_IDLE_DRAIN:
      drain (ssm, dev);
      break;

    case INIT_GAIN_READ:
      /* once per device: calibrate the pixel gain on an empty frame */
      if (self->gain_calibrated)
        {
          fpi_ssm_mark_completed (ssm);
          return;
        }
      run_fdt_read (ssm, dev);
      break;

    case INIT_GAIN_IMAGE:
      run_cmd_full (ssm, dev, CMD_GET_IMAGE, pl_image, sizeof pl_image,
                    F_ACK | F_DATA | F_OPTIONAL, G5042_TIMEOUT_ACK,
                    G5042_TIMEOUT_IMAGE);
      break;

    case INIT_GAIN_DONE:
      {
        g_autofree guint16 *img = NULL;
        double spread;
        guint gain;

        if (self->cmd_ok && self->reply_cmd == CMD_GET_IMAGE)
          img = g5042_decode_image (self->reply, self->reply_len, NULL, NULL);
        if (img == NULL)
          {
            fp_warn ("gain calibration: no image, keeping gain 0x%x", self->gain);
            fpi_ssm_mark_completed (ssm);
            return;
          }
        spread = g5042_active_spread (img);
        gain = gain_from_spread (self->gain, spread);
        fp_dbg ("gain calibration: spread %.0f at 0x%x -> gain 0x%x", spread,
                self->gain, gain);
        self->gain_calibrated = TRUE;
        if (gain == self->gain)
          {
            fpi_ssm_mark_completed (ssm);
            return;
          }
        /* upload the configuration again with the new gain */
        self->gain = gain;
        fpi_ssm_jump_to_state (ssm, INIT_DRAIN);
      }
      break;
    }
}

static void
start_init_subsm (FpiSsm *parent, FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  self->init_attempts = 0;
  self->usb_reset_tried = FALSE;
  fpi_ssm_start_subsm (parent, fpi_ssm_new (dev, init_handler, INIT_NUM_STATES));
}

/* ========================================================================
 * Finger detection (FDT)
 *
 * The sensor reads 10 capacitive channels (12 slots, the last two unused);
 * a finger lowers the readings by 0x20-0x80. An FDT command carries one
 * 16-bit threshold per slot:
 *
 *   FDT read  (0x36, mode 0x0d): reply with the current readings
 *   FDT down  (0x32, mode 0x0c): event once the readings fall below the
 *                                thresholds (finger down)
 *   FDT up    (0x34, mode 0x0e): event once they are back above (lift-off)
 *
 * Threshold format on the wire: the reading halved, in both bytes (slots
 * 8-9: high byte only). The driver keeps the readings of the last clean
 * "no finger" base and arms both events against them with a fixed margin.
 * Reply payload: event LE16, touched-channel mask LE16, 12 readings LE16.
 * ======================================================================== */

#define FDT_EV_DOWN 0x0002
#define FDT_EV_READ 0x0100
#define FDT_EV_UP 0x0200
#define FDT_EV_RESET 0x0400      /* the sensor restarted on its own */

static guint16
fdt_event (const guint8 *p, gsize len)
{
  return len >= 2 ? p[0] | (p[1] << 8) : 0;
}

static void
fdt_readings (const guint8 *p, guint16 out[12])
{
  for (int i = 0; i < 12; i++)
    out[i] = p[4 + 2 * i] | (p[5 + 2 * i] << 8);
}

static void
fdt_payload (guint8 mode, const guint16 *ref, guint margin, guint8 out[34])
{
  memset (out, 0, 34);
  out[0] = mode;
  out[1] = 0x01;
  if (ref == NULL)
    return;
  for (int i = 0; i < 10; i++)
    {
      guint h = CLAMP ((gint) (ref[i] >> 1) - (gint) margin, 0, 0xff);

      out[2 + 2 * i] = i < 8 ? h : 0;
      out[3 + 2 * i] = h;
    }
}

static void
run_fdt (FpiSsm *ssm, FpDevice *dev, guint8 cmd, guint flags, guint data_timeout)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);
  guint8 pl[34];

  switch (cmd)
    {
    case CMD_FDT_DOWN:
      fdt_payload (0x0c, self->fdt_ref, G5042_FDT_DOWN_MARGIN, pl);
      break;

    case CMD_FDT_UP:
      fdt_payload (0x0e, self->fdt_ref, G5042_FDT_UP_MARGIN, pl);
      break;

    default:
      fdt_payload (0x0d, NULL, 0, pl);
      break;
    }
  run_cmd_full (ssm, dev, cmd, pl, sizeof pl, flags, G5042_TIMEOUT_ACK, data_timeout);
}

static void
run_fdt_read (FpiSsm *ssm, FpDevice *dev)
{
  run_fdt (ssm, dev, CMD_FDT_MODE, F_ACK | F_DATA | F_OPTIONAL, G5042_TIMEOUT_DATA);
}

static gboolean
fdt_read_ok (FpiDeviceGoodix5042 *self, guint16 out[12])
{
  if (!self->cmd_ok || self->reply_cmd != CMD_FDT_MODE || self->reply_len < 28 ||
      fdt_event (self->reply, self->reply_len) != FDT_EV_READ)
    return FALSE;
  fdt_readings (self->reply, out);
  return TRUE;
}

/* largest drop of @now below @ref over the used channels */
static gint
fdt_max_drop (const guint16 ref[12], const guint16 now[12])
{
  gint drop = G_MININT;

  for (int i = 0; i < 10; i++)
    drop = MAX (drop, (gint) ref[i] - (gint) now[i]);
  return drop;
}

/* ========================================================================
 * "No finger" base
 *
 *   FDT read A -> image -> FDT read B
 *
 * The candidate is kept when the readings are steady (A ~ B), no channel
 * sits clearly below the previous base readings, and the image does not
 * show a finger against the previous base image. B becomes the reference
 * for the FDT thresholds, the image the reference for the captures.
 * ======================================================================== */

enum {
  BASE_PREPARE,
  BASE_RESYNC,
  BASE_READ_A,
  BASE_READ_A_DONE,
  BASE_IMAGE,
  BASE_IMAGE_DONE,
  BASE_READ_B,
  BASE_EVALUATE,
  BASE_DONE,
  BASE_NUM_STATES,
};

static void
base_retry_or_finish (FpiSsm *ssm, FpiDeviceGoodix5042 *self, const char *why)
{
  fp_dbg ("base attempt %u rejected: %s", self->base_attempts + 1, why);

  if (++self->base_attempts < G5042_BASE_ATTEMPTS)
    {
      fpi_ssm_jump_to_state_delayed (ssm, BASE_PREPARE, 300);
      return;
    }
  if (self->base)
    {
      fp_info ("keeping the previous base (%s)", why);
      fpi_ssm_jump_to_state (ssm, BASE_DONE);
      return;
    }
  if (self->base_strict)
    {
      fpi_ssm_mark_failed (ssm,
                           fpi_device_retry_new_msg (FP_DEVICE_RETRY_REMOVE_FINGER,
                                                     "Remove the finger from the sensor"));
      return;
    }
  fpi_ssm_jump_to_state (ssm, BASE_DONE);
}

static void
base_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case BASE_PREPARE:
      /* after repeated communication errors start over from a reset */
      if (self->base_attempts >= 2 && self->base_comm_failed)
        {
          self->base_comm_failed = FALSE;
          start_init_subsm (ssm, dev);
        }
      else
        {
          fpi_ssm_next_state (ssm);
        }
      break;

    case BASE_RESYNC:
      /* a late reply of a failed attempt must not desync the next one */
      if (self->base_attempts > 0)
        drain (ssm, dev);
      else
        fpi_ssm_next_state (ssm);
      break;

    case BASE_READ_A:
      run_fdt_read (ssm, dev);
      break;

    case BASE_READ_A_DONE:
      if (!fdt_read_ok (self, self->fdt_a))
        {
          self->base_comm_failed = TRUE;
          base_retry_or_finish (ssm, self, "no FDT reading (A)");
          return;
        }
      fpi_ssm_next_state (ssm);
      break;

    case BASE_IMAGE:
      run_cmd_full (ssm, dev, CMD_GET_IMAGE, pl_image, sizeof pl_image,
                    F_ACK | F_DATA | F_OPTIONAL, G5042_TIMEOUT_ACK,
                    G5042_TIMEOUT_IMAGE);
      break;

    case BASE_IMAGE_DONE:
      {
        gboolean crc_present = FALSE, crc_ok = FALSE;

        g_clear_pointer (&self->base_cand, g_free);
        if (self->cmd_ok && self->reply_cmd == CMD_GET_IMAGE)
          self->base_cand = g5042_decode_image (self->reply, self->reply_len,
                                                &crc_present, &crc_ok);
        if (self->base_cand == NULL)
          {
            self->base_comm_failed = TRUE;
            base_retry_or_finish (ssm, self, "no image");
            return;
          }
        if (crc_present && crc_ok)
          {
            self->crc_seen_ok = TRUE;
          }
        else if (crc_present && self->crc_seen_ok)
          {
            g_clear_pointer (&self->base_cand, g_free);
            base_retry_or_finish (ssm, self, "image CRC mismatch");
            return;
          }
        fpi_ssm_next_state (ssm);
      }
      break;

    case BASE_READ_B:
      run_fdt_read (ssm, dev);
      break;

    case BASE_EVALUATE:
      {
        guint16 b[12];
        G5042Touch touch = G5042_TOUCH_NONE;
        gint moved, drop;
        guint touched = 0;

        if (!fdt_read_ok (self, b))
          {
            self->base_comm_failed = TRUE;
            base_retry_or_finish (ssm, self, "no FDT reading (B)");
            return;
          }
        moved = MAX (fdt_max_drop (self->fdt_a, b), fdt_max_drop (b, self->fdt_a));
        if (moved > G5042_FDT_STEADY)
          {
            base_retry_or_finish (ssm, self, "FDT readings moving");
            return;
          }

        /* after two vetoes a change that passed the steadiness check is
         * taken as real (temperature, a re-plugged sensor) */
        if (self->base && self->base_attempts < 2)
          {
            drop = fdt_max_drop (self->fdt_ref, b);
            if (drop > G5042_FDT_DOWN_MARGIN * 2)
              {
                base_retry_or_finish (ssm, self, "FDT readings below the reference");
                return;
              }
            touch = g5042_touch_classify (self->base, self->base_cand, &touched);
            if (touch == G5042_TOUCH_FINGER)
              {
                base_retry_or_finish (ssm, self, "image shows a finger");
                return;
              }
          }

        fp_dbg ("base accepted: mean %.0f, %s (%u%%), FDT %03x %03x %03x %03x %03x",
                g5042_active_mean (self->base_cand), g5042_touch_str (touch),
                touched, b[0], b[2], b[4], b[6], b[8]);
        memcpy (self->fdt_ref, b, sizeof self->fdt_ref);
        self->fdt_ref_valid = TRUE;
        g_free (self->base);
        self->base = g_steal_pointer (&self->base_cand);
        fpi_ssm_next_state (ssm);
      }
      break;

    case BASE_DONE:
      g_clear_pointer (&self->base_cand, g_free);
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
start_base_subsm (FpiSsm *parent, FpDevice *dev, gboolean strict)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  self->base_attempts = 0;
  self->base_comm_failed = FALSE;
  self->base_strict = strict;
  fpi_ssm_start_subsm (parent, fpi_ssm_new (dev, base_handler, BASE_NUM_STATES));
}

/* ========================================================================
 * Finger wait: FDT down against the base readings
 *
 * The sensor may ACK the command and then stay silent; re-arm after a
 * silence instead of waiting forever. Other events re-arm; a sensor
 * restart or too many of them start over from the initialisation.
 * ======================================================================== */

enum {
  WAIT_ARM,
  WAIT_EVENT,
  WAIT_CHECK,
  WAIT_REINIT,
  WAIT_REBASE,
  WAIT_NUM_STATES,
};

static void
wait_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case WAIT_ARM:
      if (g_cancellable_is_cancelled (self->cancel))
        {
          fpi_ssm_mark_failed (ssm, g_error_new_literal (G_IO_ERROR,
                                                         G_IO_ERROR_CANCELLED,
                                                         "Cancelled"));
          return;
        }
      if (!self->fdt_ref_valid)
        {
          fpi_ssm_jump_to_state (ssm, WAIT_REBASE);
          return;
        }
      run_fdt (ssm, dev, CMD_FDT_DOWN, F_ACK | F_OPTIONAL | F_CANCELLABLE,
               G5042_TIMEOUT_DATA);
      break;

    case WAIT_EVENT:
      if (!self->cmd_ok && !self->rx_timed_out &&
          self->reply_cmd == CMD_FDT_DOWN)
        {
          /* the event overtook the ACK */
          fpi_ssm_jump_to_state (ssm, WAIT_CHECK);
          return;
        }
      if (!self->cmd_ok)
        {
          fpi_ssm_jump_to_state_delayed (ssm, WAIT_ARM, 100);
          return;
        }
      recv_message (ssm, dev, G5042_FDT_POLL_MS, self->cancel, TRUE);
      break;

    case WAIT_CHECK:
      {
        guint16 ev;

        if (self->rx_timed_out || self->reply_cmd != CMD_FDT_DOWN)
          {
            fpi_ssm_jump_to_state (ssm, WAIT_ARM);
            return;
          }
        ev = fdt_event (self->reply, self->reply_len);
        if (ev == FDT_EV_DOWN)
          {
            fp_dbg ("finger down");
            self->wait_events = 0;
            fpi_device_report_finger_status_changes (dev,
                                                     FP_FINGER_STATUS_PRESENT,
                                                     FP_FINGER_STATUS_NEEDED);
            fpi_ssm_mark_completed (ssm);
            return;
          }
        if (ev & FDT_EV_RESET || ++self->wait_events >= G5042_MAX_REARM)
          {
            fp_info ("FDT event 0x%04x, re-initialising the sensor", ev);
            self->wait_events = 0;
            fpi_ssm_jump_to_state (ssm, WAIT_REINIT);
            return;
          }
        fp_dbg ("FDT event 0x%04x, re-arming", ev);
        fpi_ssm_jump_to_state (ssm, WAIT_ARM);
      }
      break;

    case WAIT_REINIT:
      start_init_subsm (ssm, dev);
      break;

    case WAIT_REBASE:
      start_base_subsm (ssm, dev, FALSE);
      break;

    case WAIT_NUM_STATES:
      fpi_ssm_jump_to_state (ssm, WAIT_ARM);
      break;
    }
}

static void
start_wait_subsm (FpiSsm *parent, FpDevice *dev)
{
  fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_NEEDED,
                                           FP_FINGER_STATUS_PRESENT);
  fpi_ssm_start_subsm (parent, fpi_ssm_new (dev, wait_handler, WAIT_NUM_STATES + 1));
}

/* ========================================================================
 * Scan SSM shared by enroll / verify / identify
 * ======================================================================== */

enum {
  SCAN_INIT,
  SCAN_BASE,
  SCAN_WAIT,
  SCAN_SETTLE,
  SCAN_IMAGE,
  SCAN_IMAGE_DONE,
  SCAN_CLASSIFY,
  SCAN_PROCESS,
  SCAN_FDT_UP,
  SCAN_FDT_UP_DONE,
  SCAN_REBASE,
  SCAN_NUM_STATES,
};

static void
features_free (gpointer f)
{
  fm_features_clear (f);
  g_free (f);
}

/* features of the captured frame: active area of the frame and of the
 * "no finger" base (lines 1..174, pixels 1..52) */
static FmFeatures *
extract (FpiDeviceGoodix5042 *self)
{
  g_autofree guint16 *base = g_new (guint16, G5042_ACT_PIXELS);
  g_autofree guint16 *frame = g_new (guint16, G5042_ACT_PIXELS);
  FmFeatures *f = g_new0 (FmFeatures, 1);

  for (guint y = 0; y < G5042_ACT_H; y++)
    for (guint x = 0; x < G5042_ACT_W; x++)
      {
        guint i = (x + G5042_ACT_FIRST_LINE) * G5042_LINE_DECODED + y + G5042_ACT_FIRST_PIX;

        base[y * G5042_ACT_W + x] = self->base[i];
        frame[y * G5042_ACT_W + x] = self->frames[0][i];
      }
  fm_extract (&self->match, base, frame, G5042_ACT_W, G5042_ACT_H, f);
  fp_dbg ("features: %d keypoints, coverage %d, quality %d", f->n, f->coverage,
          f->quality);
  return f;
}

static void
process_enroll (FpiDeviceGoodix5042 *self, FpDevice *dev)
{
  FmFeatures *f = extract (self);

  self->after = AFTER_NEXT_STAGE;

  if (f->coverage < G5042_MIN_COVERAGE)
    {
      fp_info ("enroll: touch rejected, coverage %d (min %d)", f->coverage,
               G5042_MIN_COVERAGE);
      features_free (f);
      fpi_device_enroll_progress (dev, self->enroll_stage, NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      return;
    }

  /* a touch on the same area as an accepted one adds nothing */
  for (guint i = 0; i < self->enroll->len; i++)
    {
      int score = fm_match (&self->match, f, g_ptr_array_index (self->enroll, i));

      if (score >= G5042_DUP_SCORE)
        {
          fp_info ("enroll: touch rejected, same area as sample %u (score %d)", i, score);
          features_free (f);
          fpi_device_enroll_progress (dev, self->enroll_stage, NULL,
                                      fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
          return;
        }
    }

  g_ptr_array_add (self->enroll, f);
  self->enroll_stage = self->enroll->len;
  fp_info ("enroll: sample %d/%d accepted (%d keypoints, coverage %d)",
           self->enroll_stage, G5042_ENROLL_SAMPLES, f->n, f->coverage);
  fpi_device_enroll_progress (dev, self->enroll_stage, NULL, NULL);

  if (self->enroll_stage >= G5042_ENROLL_SAMPLES)
    self->after = AFTER_DONE_WAIT_UP;
}

/* print data: (magic, version, samples) with every sample serialized */
static GVariant *
print_data (GPtrArray *samples)
{
  g_autoptr(GByteArray) buf = g_byte_array_new ();

  for (guint i = 0; i < samples->len; i++)
    {
      const FmFeatures *f = g_ptr_array_index (samples, i);
      gsize off = buf->len;

      g_byte_array_set_size (buf, off + fm_serialized_size (f));
      fm_serialize (f, buf->data + off);
    }
  return g_variant_new ("(sq@ay)", G5042_PRINT_MAGIC, (guint16) G5042_PRINT_VERSION,
                        g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, buf->data,
                                                   buf->len, 1));
}

/* best score of @probe against the samples of @print, -1 if the print
 * data is not ours */
static int
compare_print (FpiDeviceGoodix5042 *self, const FmFeatures *probe, FpPrint *print)
{
  g_autoptr(GVariant) data = NULL;
  g_autoptr(GVariant) bytes = NULL;
  const char *magic;
  const guint8 *p;
  guint16 version;
  gsize len, off = 0;
  int best = -1;

  g_object_get (G_OBJECT (print), "fpi-data", &data, NULL);
  if (data == NULL || !g_variant_is_of_type (data, G_VARIANT_TYPE ("(sqay)")))
    return -1;
  g_variant_get (data, "(&sq@ay)", &magic, &version, &bytes);
  if (strcmp (magic, G5042_PRINT_MAGIC) != 0 || version != G5042_PRINT_VERSION)
    return -1;

  p = g_variant_get_fixed_array (bytes, &len, 1);
  while (off < len)
    {
      FmFeatures f;
      size_t used = 0;
      int score;

      if (fm_deserialize (p + off, len - off, &f, &used) != 0)
        return -1;
      off += used;
      score = fm_match (&self->match, probe, &f);
      fm_features_clear (&f);
      if (score > best)
        best = score;
    }
  return best;
}

static void
process_match (FpiDeviceGoodix5042 *self, FpDevice *dev)
{
  FpiDeviceAction action = fpi_device_get_current_action (dev);

  g_autoptr(GPtrArray) owned = NULL;
  GPtrArray *gallery = NULL;
  FpPrint *verify_print = NULL;
  FpPrint *match = NULL;
  FmFeatures *probe;
  int best = -1;
  gboolean any_valid = FALSE;

  if (action == FPI_DEVICE_ACTION_IDENTIFY)
    {
      fpi_device_get_identify_data (dev, &gallery);
    }
  else
    {
      fpi_device_get_verify_data (dev, &verify_print);
      gallery = owned = g_ptr_array_new ();
      g_ptr_array_add (owned, verify_print);
    }

  self->result_reported = TRUE;
  probe = extract (self);

  if (probe->coverage < G5042_MIN_COVERAGE)
    {
      GError *retry = fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER);

      fp_info ("match: coverage %d below %d, retry", probe->coverage, G5042_MIN_COVERAGE);
      features_free (probe);
      if (action == FPI_DEVICE_ACTION_IDENTIFY)
        fpi_device_identify_report (dev, NULL, NULL, retry);
      else
        fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, retry);
      self->after = AFTER_DONE_WAIT_UP;
      return;
    }

  /* the print with the best score, if it reaches the threshold */
  for (guint i = 0; i < gallery->len; i++)
    {
      FpPrint *p = g_ptr_array_index (gallery, i);
      int score = compare_print (self, probe, p);

      fp_dbg ("print %u: score %d", i, score);
      if (score < 0)
        continue;
      any_valid = TRUE;
      if (score > best)
        {
          best = score;
          match = score >= G5042_MATCH_SCORE ? p : NULL;
        }
    }
  features_free (probe);

  /* an empty gallery (fprintd's duplicate check before the first
   * enrollment) is a plain no match */
  if (!any_valid && gallery->len > 0)
    {
      /* prints from another driver or format: report, no retry */
      self->result_reported = FALSE;
      self->after = AFTER_DONE_WAIT_UP;
      return;
    }

  fp_info ("%s: best score %d, %s", action == FPI_DEVICE_ACTION_IDENTIFY ? "identify" : "verify",
           best, match ? "match" : "no match");
  if (action == FPI_DEVICE_ACTION_IDENTIFY)
    fpi_device_identify_report (dev, match, NULL, NULL);
  else
    fpi_device_verify_report (dev, match ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL,
                              NULL, NULL);
  /* a failed attempt waits for lift-off so one held finger cannot burn
   * through several PAM tries */
  self->after = match ? AFTER_DONE : AFTER_DONE_WAIT_UP;
}

static void
scan_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case SCAN_INIT:
      if (self->needs_reinit)
        start_init_subsm (ssm, dev);
      else
        fpi_ssm_next_state (ssm);
      break;

    case SCAN_BASE:
      self->needs_reinit = FALSE;
      start_base_subsm (ssm, dev, TRUE);
      break;

    case SCAN_WAIT:
      clear_frames (self);
      self->image_retries = 0;
      start_wait_subsm (ssm, dev);
      break;

    case SCAN_SETTLE:
      fpi_ssm_next_state_delayed (ssm, G5042_SETTLE_MS);
      break;

    case SCAN_IMAGE:
      run_cmd_full (ssm, dev, CMD_GET_IMAGE, pl_image, sizeof pl_image,
                    F_ACK | F_DATA, G5042_TIMEOUT_ACK, G5042_TIMEOUT_IMAGE);
      break;

    case SCAN_IMAGE_DONE:
      {
        gboolean crc_present, crc_ok;
        guint16 *img = NULL;

        if (self->reply_cmd == CMD_GET_IMAGE)
          img = g5042_decode_image (self->reply, self->reply_len,
                                    &crc_present, &crc_ok);
        if (img == NULL)
          {
            fpi_ssm_mark_failed (ssm,
                                 fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                           "Bad image reply (0x%02x, %zu bytes)",
                                                           self->reply_cmd, self->reply_len));
            return;
          }

        if (crc_present && crc_ok)
          {
            self->crc_seen_ok = TRUE;
          }
        else if (crc_present && self->crc_seen_ok)
          {
            /* drop frames failing the CRC */
            g_free (img);
            if (++self->image_retries <= 2)
              {
                fp_warn ("image CRC mismatch, capturing again");
                fpi_ssm_jump_to_state (ssm, SCAN_IMAGE);
                return;
              }
            fpi_ssm_mark_failed (ssm,
                                 fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                           "Image CRC mismatch"));
            return;
          }
        else if (crc_present)
          {
            fp_dbg ("image CRC does not match (never verified on this unit)");
          }

        self->frames[self->nframes++] = img;
        if (self->nframes < self->wanted_frames)
          fpi_ssm_jump_to_state_delayed (ssm, SCAN_IMAGE, G5042_FRAME_GAP_MS);
        else
          fpi_ssm_next_state (ssm);
      }
      break;

    case SCAN_CLASSIFY:
      {
        G5042Touch touch;
        guint touched;

        /* only a finger reaches the matcher */
        touch = g5042_touch_classify (self->base, self->frames[0], &touched);
        fp_dbg ("capture: mean %.0f, %s (%u%% of the blocks)",
                g5042_active_mean (self->frames[0]), g5042_touch_str (touch), touched);

        if (touch == G5042_TOUCH_FINGER)
          {
            self->rearm_count = 0;
            fpi_ssm_next_state (ssm);
            return;
          }
        /* a shifted or brighter frame means a stale base; repeated false
         * triggers point the same way */
        if (touch != G5042_TOUCH_NONE || ++self->rearm_count >= G5042_MAX_REARM)
          {
            self->rearm_count = 0;
            self->after = AFTER_REBASE;
          }
        else
          {
            self->after = AFTER_REARM;
          }
        fpi_ssm_jump_to_state (ssm, SCAN_FDT_UP);
      }
      break;

    case SCAN_PROCESS:
      if (fpi_device_get_current_action (dev) == FPI_DEVICE_ACTION_ENROLL)
        process_enroll (self, dev);
      else
        process_match (self, dev);

      if (self->after == AFTER_DONE)
        {
          /* skip the lift-off wait: the next action re-initialises */
          self->needs_reinit = TRUE;
          fpi_ssm_mark_completed (ssm);
          return;
        }
      fpi_ssm_next_state (ssm);
      break;

    case SCAN_FDT_UP:
      /* the event only arrives once the finger is lifted */
      run_fdt (ssm, dev, CMD_FDT_UP, F_ACK | F_DATA | F_OPTIONAL | F_CANCELLABLE,
               G5042_TIMEOUT_LIFT);
      break;

    case SCAN_FDT_UP_DONE:
      if (self->cmd_ok && self->reply_cmd != CMD_FDT_UP)
        fp_dbg ("lift-off: unexpected reply 0x%02x", self->reply_cmd);
      fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_NONE,
                                               FP_FINGER_STATUS_PRESENT |
                                               FP_FINGER_STATUS_NEEDED);
      if (!self->cmd_ok)
        {
          /* no lift-off answer: the sensor state machine is lost, start
           * over from a reset */
          fp_info ("no answer to FDT_UP, re-initialising the sensor");
          self->needs_reinit = TRUE;
        }
      switch (self->after)
        {
        case AFTER_REARM:
          fpi_ssm_jump_to_state (ssm, self->needs_reinit ? SCAN_INIT : SCAN_WAIT);
          break;

        case AFTER_REBASE:
          fpi_ssm_jump_to_state (ssm, self->needs_reinit ? SCAN_INIT : SCAN_REBASE);
          break;

        case AFTER_NEXT_STAGE:
          if (self->needs_reinit)
            fpi_ssm_jump_to_state (ssm, SCAN_INIT);
          else
            fpi_ssm_next_state (ssm);
          break;

        case AFTER_DONE:
        case AFTER_DONE_WAIT_UP:
          fpi_ssm_mark_completed (ssm);
          break;
        }
      break;

    case SCAN_REBASE:
      /* refresh the base after every lift-off */
      start_base_subsm (ssm, dev, FALSE);
      break;

    case SCAN_NUM_STATES:
      fpi_ssm_jump_to_state (ssm, SCAN_WAIT);
      break;
    }
}

static void
enroll_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  g_autoptr(GPtrArray) enroll = g_steal_pointer (&self->enroll);
  FpPrint *print = NULL;

  clear_frames (self);

  /* cancelled while waiting for the last lift-off: the print is complete */
  if (error && self->enroll_stage >= G5042_ENROLL_SAMPLES &&
      g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
    {
      self->needs_reinit = TRUE;
      g_clear_error (&error);
    }

  if (error)
    {
      self->needs_reinit = TRUE;
      if (error->domain == FP_DEVICE_RETRY)
        {
          /* no clean base: something rests on the sensor */
          g_clear_error (&error);
          error = fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                            "Sensor covered: lift the finger and retry");
        }
      fpi_device_enroll_complete (dev, NULL, error);
      return;
    }

  fpi_device_get_enroll_data (dev, &print);
  fpi_print_set_type (print, FPI_PRINT_RAW);
  g_object_set (G_OBJECT (print), "fpi-data", print_data (enroll), NULL);

  fp_info ("enrollment complete (%u samples)", enroll->len);
  fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}

static void
match_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);
  FpiDeviceAction action = fpi_device_get_current_action (dev);

  clear_frames (self);

  if (error)
    {
      self->needs_reinit = TRUE;

      /* cancelled while waiting for lift-off after the result went out */
      if (self->result_reported &&
          g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        {
          g_clear_error (&error);
        }
      else if (error->domain == FP_DEVICE_RETRY)
        {
          /* base could not be taken: finger resting on the sensor */
          if (action == FPI_DEVICE_ACTION_IDENTIFY)
            fpi_device_identify_report (dev, NULL, NULL, error);
          else
            fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, error);
          error = NULL;
        }
    }
  else if (!self->result_reported)
    {
      error = fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID,
                                        "Stored print is not a goodix5042 template: "
                                        "enroll again");
    }

  if (action == FPI_DEVICE_ACTION_IDENTIFY)
    fpi_device_identify_complete (dev, error);
  else
    fpi_device_verify_complete (dev, error);
}

static void
start_scan (FpDevice *dev, guint frames, FpiSsmCompletedCallback done)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  g_clear_object (&self->cancel);
  self->cancel = g_cancellable_new ();
  self->result_reported = FALSE;
  self->wanted_frames = frames;
  clear_frames (self);
  fpi_ssm_start (fpi_ssm_new (dev, scan_handler, SCAN_NUM_STATES + 1), done);
}

/* ========================================================================
 * FpDevice vfuncs
 * ======================================================================== */

enum {
  OPEN_CLAIM,
  OPEN_INIT,
  OPEN_NUM_STATES,
};

static void
open_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);
  GError *error = NULL;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case OPEN_CLAIM:
      /* detaches cdc_acm from the data interface. It is deliberately not
       * re-attached on release: a /dev/ttyACM node would be probed by
       * ModemManager with AT commands. Under device emulation there is no
       * kernel driver to detach. */
      if (!g_usb_device_claim_interface (fpi_device_get_usb_device (dev),
                                         G5042_USB_INTERFACE,
                                         fpi_device_emulation_mode_enabled (dev) ? 0 :
                                         G_USB_DEVICE_CLAIM_INTERFACE_BIND_KERNEL_DRIVER,
                                         &error))
        {
          fpi_ssm_mark_failed (ssm, error);
          return;
        }
      self->claimed = TRUE;
      fpi_ssm_next_state (ssm);
      break;

    case OPEN_INIT:
      start_init_subsm (ssm, dev);
      break;
    }
}

static void
open_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  if (error)
    {
      if (self->claimed)
        g_usb_device_release_interface (fpi_device_get_usb_device (dev),
                                        G5042_USB_INTERFACE, 0, NULL);
      self->claimed = FALSE;
      fpi_device_open_complete (dev, error);
      return;
    }

  self->needs_reinit = FALSE;
  fp_info ("Goodix 5042 ready (firmware %s, pixel gain 0x%x)",
           self->fw_version ? self->fw_version : "?", self->gain);
  fpi_device_open_complete (dev, NULL);
}

static void
g5042_open (FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  g_clear_object (&self->cancel);
  self->cancel = g_cancellable_new ();
  g5042_rx_reset (&self->rx);
  fpi_ssm_start (fpi_ssm_new (dev, open_handler, OPEN_NUM_STATES), open_done);
}

static void
g5042_close (FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);
  GError *error = NULL;

  clear_frames (self);
  g_clear_object (&self->cancel);
  g_clear_pointer (&self->reply, g_free);
  g_clear_pointer (&self->cmd_payload, g_free);
  g_clear_pointer (&self->base_cand, g_free);
  g5042_rx_free (&self->rx);

  if (self->claimed)
    g_usb_device_release_interface (fpi_device_get_usb_device (dev),
                                    G5042_USB_INTERFACE, 0, &error);
  self->claimed = FALSE;
  fpi_device_close_complete (dev, error);
}

static void
g5042_enroll (FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  self->enroll_stage = 0;
  g_clear_pointer (&self->enroll, g_ptr_array_unref);
  self->enroll = g_ptr_array_new_with_free_func (features_free);
  start_scan (dev, 1, enroll_done);
}

static void
g5042_verify_identify (FpDevice *dev)
{
  start_scan (dev, 1, match_done);
}

static void
g5042_cancel (FpDevice *dev)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (dev);

  if (self->cancel)
    g_cancellable_cancel (self->cancel);
}

static void
fpi_device_goodix5042_finalize (GObject *object)
{
  FpiDeviceGoodix5042 *self = FPI_DEVICE_GOODIX5042 (object);

  clear_frames (self);
  g_clear_pointer (&self->base, g_free);
  g_clear_pointer (&self->base_cand, g_free);
  g_clear_pointer (&self->fw_version, g_free);
  g_clear_pointer (&self->reply, g_free);
  g_clear_pointer (&self->cmd_payload, g_free);
  g_clear_pointer (&self->enroll, g_ptr_array_unref);
  g5042_rx_free (&self->rx);
  g_clear_object (&self->cancel);

  G_OBJECT_CLASS (fpi_device_goodix5042_parent_class)->finalize (object);
}

static void
fpi_device_goodix5042_init (FpiDeviceGoodix5042 *self)
{
  fm_params_default (&self->match);
}

static const FpIdEntry id_table[] = {
  { .vid = 0x27c6, .pid = 0x5042, },
  { .vid = 0, .pid = 0, .driver_data = 0 },
};

static void
fpi_device_goodix5042_class_init (FpiDeviceGoodix5042Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->finalize = fpi_device_goodix5042_finalize;

  dev_class->id = "goodix5042";
  dev_class->full_name = "Goodix 5042 Fingerprint Reader (BENSS UA002)";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->id_table = id_table;
  dev_class->nr_enroll_stages = G5042_ENROLL_SAMPLES;
  dev_class->temp_hot_seconds = -1;
  dev_class->features = FP_DEVICE_FEATURE_VERIFY | FP_DEVICE_FEATURE_IDENTIFY;

  dev_class->open = g5042_open;
  dev_class->close = g5042_close;
  dev_class->enroll = g5042_enroll;
  dev_class->verify = g5042_verify_identify;
  dev_class->identify = g5042_verify_identify;
  dev_class->cancel = g5042_cancel;
}
