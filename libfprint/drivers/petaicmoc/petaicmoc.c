/*
 * Petaic match-on-chip driver for libfprint
 *
 * Supports the Petaic "Touch Fingerprint Sensor" (USB 28e9:5151), a
 * match-on-chip sensor. Fingerprint templates are stored and matched inside
 * the sensor; the host only sends commands and reads results.
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

#define FP_COMPONENT "petaicmoc"

#include "drivers_api.h"

#include "petaicmoc.h"

#define EP_IN (1 | FPI_USB_ENDPOINT_IN)
#define EP_OUT (1 | FPI_USB_ENDPOINT_OUT)

/* Timeout for everything except the "image" commands, which wait for a finger */
#define CMD_TIMEOUT 3000
/* Number of valid images the chip needs for one enrollment */
#define PETAIC_ENROLL_STAGES 12
/* Give up an enrollment after this many rejected/failed captures */
#define PETAIC_MAX_CAPTURE_ATTEMPTS 60
/* Delay between two DETECT_FINGER polls, in ms */
#define PETAIC_POLL_INTERVAL 30
/* The record index is a single byte; the real number of slots is read from the sensor */
#define PETAIC_MAX_SLOTS 255

struct _FpiDevicePetaicMoc
{
  FpDevice   parent;

  FpiSsm    *task_ssm;
  gboolean   setup_done;

  guint8     db_free;
  guint8     db_used;

  gint       enroll_accepted;
  gint       capture_attempts;
  guint8     enroll_identity[PETAIC_IDENTITY_SIZE];

  GPtrArray *list_result;
  guint      list_index;
};

G_DEFINE_TYPE (FpiDevicePetaicMoc, fpi_device_petaicmoc, FP_TYPE_DEVICE)

typedef struct
{
  guint16 length;
  guint8  cmd;
  guint8  event;
  guint8  status2;
  guint8  payload[PETAIC_MAX_PAYLOAD];
} PetaicResp;

typedef void (*PetaicCmdCb) (FpiDevicePetaicMoc *self,
                             const PetaicResp   *resp,
                             GError             *error,
                             gpointer            user_data);

typedef struct
{
  FpiDevicePetaicMoc *self;
  guint8              cmd;
  gboolean            wait_finger;
  guint               rereads;
  guint8              frame[PETAIC_FRAME_SIZE];
  PetaicCmdCb         callback;
  gpointer            user_data;
} CmdCtx;

/******************************************************************************
 *
 *  Framing and command transport
 *
 *****************************************************************************/

static guint16
petaic_checksum (const guint8 *frame)
{
  guint32 sum = 0;

  for (guint i = 0; i < PETAIC_CHECKSUM_OFFSET; i++)
    sum += frame[i];

  return sum & 0xFFFF;
}

static void
cmd_ctx_finish (CmdCtx *ctx, const PetaicResp *resp, GError *error)
{
  if (ctx->callback)
    ctx->callback (ctx->self, resp, error, ctx->user_data);
  else if (error)
    g_error_free (error);

  g_free (ctx);
}

static void cmd_read (CmdCtx *ctx);

static void
cmd_in_cb (FpiUsbTransfer *transfer,
           FpDevice       *device,
           gpointer        user_data,
           GError         *error)
{
  CmdCtx *ctx = user_data;
  FpiDevicePetaicMoc *self = ctx->self;
  PetaicResp resp = { 0, };
  const guint8 *buf;
  guint16 stored_sum;

  if (error)
    {
      cmd_ctx_finish (ctx, NULL, error);
      return;
    }

  if (transfer->actual_length != PETAIC_FRAME_SIZE)
    {
      cmd_ctx_finish (ctx, NULL,
                      fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                "Unexpected response length %" G_GSSIZE_FORMAT,
                                                transfer->actual_length));
      return;
    }

  buf = transfer->buffer;
  stored_sum = buf[PETAIC_CHECKSUM_OFFSET] | (buf[PETAIC_CHECKSUM_OFFSET + 1] << 8);
  if (stored_sum != petaic_checksum (buf))
    {
      cmd_ctx_finish (ctx, NULL,
                      fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                "Response checksum mismatch"));
      return;
    }

  resp.length = buf[0] | (buf[1] << 8);
  resp.cmd = buf[2];
  resp.event = buf[4];
  resp.status2 = buf[5];
  memcpy (resp.payload, buf + PETAIC_PAYLOAD_OFFSET, PETAIC_MAX_PAYLOAD);

  /* The device echoes the sub-command; skip stale responses (re-read, at most twice) */
  if (resp.cmd != ctx->cmd)
    {
      if (ctx->rereads++ < 2)
        {
          fp_dbg ("Skipping response for 0x%02x while waiting for 0x%02x", resp.cmd, ctx->cmd);
          cmd_read (ctx);
          return;
        }
      cmd_ctx_finish (ctx, NULL,
                      fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                "Got response 0x%02x for command 0x%02x",
                                                resp.cmd, ctx->cmd));
      return;
    }

  /* CANCEL resets the sensor's session, the setup request has to be repeated */
  if (ctx->cmd == PETAIC_CMD_CANCEL)
    self->setup_done = FALSE;

  cmd_ctx_finish (ctx, &resp, NULL);
}

static void
cmd_read (CmdCtx *ctx)
{
  FpDevice *dev = FP_DEVICE (ctx->self);
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk (transfer, EP_IN, PETAIC_FRAME_SIZE);

  /* Image commands only answer once a finger was placed: no timeout, but cancellable */
  fpi_usb_transfer_submit (transfer,
                           ctx->wait_finger ? 0 : CMD_TIMEOUT,
                           ctx->wait_finger ? fpi_device_get_cancellable (dev) : NULL,
                           cmd_in_cb,
                           ctx);
}

static void
cmd_out_cb (FpiUsbTransfer *transfer,
            FpDevice       *device,
            gpointer        user_data,
            GError         *error)
{
  CmdCtx *ctx = user_data;

  if (error)
    {
      cmd_ctx_finish (ctx, NULL, error);
      return;
    }

  cmd_read (ctx);
}

static void
cmd_send (CmdCtx *ctx)
{
  FpDevice *dev = FP_DEVICE (ctx->self);
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk (transfer, EP_OUT, PETAIC_FRAME_SIZE);
  memcpy (transfer->buffer, ctx->frame, PETAIC_FRAME_SIZE);

  fpi_usb_transfer_submit (transfer, CMD_TIMEOUT, NULL, cmd_out_cb, ctx);
}

static void
cmd_setup_cb (FpiUsbTransfer *transfer,
              FpDevice       *device,
              gpointer        user_data,
              GError         *error)
{
  CmdCtx *ctx = user_data;

  if (error)
    {
      cmd_ctx_finish (ctx, NULL, error);
      return;
    }

  if (transfer->actual_length != 1 || transfer->buffer[0] != PETAIC_SETUP_MAGIC)
    {
      cmd_ctx_finish (ctx, NULL,
                      fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                "Setup handshake failed (got 0x%02x, expected 0x%02x)",
                                                transfer->actual_length > 0 ? transfer->buffer[0] : 0,
                                                PETAIC_SETUP_MAGIC));
      return;
    }

  ctx->self->setup_done = TRUE;
  cmd_send (ctx);
}

/*
 * Send one command frame and call @callback with the parsed response. The
 * vendor setup handshake is done first when the session is not set up.
 * @wait_finger marks "image" commands (0xc0 / 0xc4) that block until a finger
 * is placed on the sensor.
 */
static void
petaic_cmd (FpiDevicePetaicMoc *self,
            guint8              cmd,
            const guint8       *payload,
            gsize               payload_len,
            gboolean            wait_finger,
            PetaicCmdCb         callback,
            gpointer            user_data)
{
  CmdCtx *ctx = g_new0 (CmdCtx, 1);
  guint16 sum;

  g_assert (payload_len <= PETAIC_MAX_PAYLOAD);

  ctx->self = self;
  ctx->cmd = cmd;
  ctx->wait_finger = wait_finger;
  ctx->callback = callback;
  ctx->user_data = user_data;

  ctx->frame[0] = payload_len & 0xFF;
  ctx->frame[1] = (payload_len >> 8) & 0xFF;
  ctx->frame[2] = cmd;
  if (payload_len > 0)
    memcpy (ctx->frame + PETAIC_PAYLOAD_OFFSET, payload, payload_len);
  sum = petaic_checksum (ctx->frame);
  ctx->frame[PETAIC_CHECKSUM_OFFSET] = sum & 0xFF;
  ctx->frame[PETAIC_CHECKSUM_OFFSET + 1] = sum >> 8;

  if (!self->setup_done)
    {
      FpiUsbTransfer *transfer = fpi_usb_transfer_new (FP_DEVICE (self));

      fpi_usb_transfer_fill_control (transfer,
                                     G_USB_DEVICE_DIRECTION_DEVICE_TO_HOST,
                                     G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                     G_USB_DEVICE_RECIPIENT_DEVICE,
                                     PETAIC_SETUP_REQUEST, 0, 0, 1);
      fpi_usb_transfer_submit (transfer, CMD_TIMEOUT, NULL, cmd_setup_cb, ctx);
      return;
    }

  cmd_send (ctx);
}

/******************************************************************************
 *
 *  Helpers: identity records and prints
 *
 *****************************************************************************/

static gboolean
identity_is_ours (const guint8 *identity)
{
  guint32 type = identity[0] | (identity[1] << 8) | (identity[2] << 16) |
                 ((guint32) identity[3] << 24);

  return type == PETAIC_IDENTITY_TYPE_GUID &&
         memcmp (identity + 4, PETAIC_GUID_PREFIX, PETAIC_GUID_PREFIX_LEN) == 0;
}

static void
identity_make (guint8 *identity, const guint8 *guid, guint8 finger)
{
  memset (identity, 0, PETAIC_IDENTITY_SIZE);
  identity[0] = PETAIC_IDENTITY_TYPE_GUID;
  memcpy (identity + 4, guid, PETAIC_GUID_SIZE);
  identity[PETAIC_IDENTITY_SUBFACTOR_OFFSET] = finger;
}

static void
set_print_data (FpPrint *print, guint8 finger, const guint8 *guid)
{
  GVariant *guid_var = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                                  guid, PETAIC_GUID_SIZE, 1);
  GVariant *data = g_variant_new ("(y@ay)", finger, guid_var);

  fpi_print_set_type (print, FPI_PRINT_RAW);
  fpi_print_set_device_stored (print, TRUE);
  g_object_set (print, "fpi-data", data, NULL);
}

static FpPrint *
print_from_identity (FpiDevicePetaicMoc *self, const guint8 *identity)
{
  FpPrint *print = fp_print_new (FP_DEVICE (self));

  set_print_data (print, identity[PETAIC_IDENTITY_SUBFACTOR_OFFSET], identity + 4);
  return print;
}

static gboolean
identity_from_print (FpPrint *print, guint8 *identity)
{
  g_autoptr(GVariant) data = NULL;
  g_autoptr(GVariant) guid_var = NULL;
  const guint8 *guid;
  gsize guid_len = 0;
  guint8 finger;

  g_object_get (print, "fpi-data", &data, NULL);
  if (data == NULL || !g_variant_check_format_string (data, "(y@ay)", FALSE))
    return FALSE;

  g_variant_get (data, "(y@ay)", &finger, &guid_var);
  guid = g_variant_get_fixed_array (guid_var, &guid_len, 1);
  if (guid_len != PETAIC_GUID_SIZE ||
      memcmp (guid, PETAIC_GUID_PREFIX, PETAIC_GUID_PREFIX_LEN) != 0)
    return FALSE;

  identity_make (identity, guid, finger);
  return TRUE;
}

/*
 * The result of VERIFY_MATCHING_RESULT is XOR-obfuscated with a key derived
 * from the first 8 bytes of the payload. @plain receives 100 bytes.
 */
static void
decode_verify_result (const guint8 *payload, guint8 *plain)
{
  guint8 k[16];
  guint8 ct[100];
  guint j;

  k[0] = payload[0] + payload[7];
  for (guint i = 1; i < 16; i++)
    k[i] = (guint8) ((payload[i % 8] ^ k[i - 1]) + 0x87);

  memcpy (ct, payload + 8, sizeof (ct));
  j = k[0] & 0xF;
  for (guint t = 0; t < sizeof (ct); t++)
    {
      ct[t] ^= k[j];
      j = (j - 1) & 0xF;
    }
  for (guint t = 0; t < sizeof (ct); t++)
    plain[t] = ct[t] ^ k[(15 - t) & 0xF];
}

/* Events from the capture commands that just mean "try again" */
static gboolean
event_is_retry (guint8 event)
{
  switch (event)
    {
    case 0x02:
    case 0x03:
    case 0x05:
    case 0xA0:         /* image not qualified */
    case 0xA1:         /* fake finger */
    case 0xAA:         /* verify: feature extraction failed */
    case 0xAB:
    case 0xAC:
      return TRUE;

    default:
      return FALSE;
    }
}

static GError *
event_error (guint8 event, const char *what)
{
  return fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                   "%s failed with event 0x%02x", what, event);
}

#define RETURN_IF_ERROR(self, error) \
  G_STMT_START { \
    if (error) \
      { \
        fpi_ssm_mark_failed ((self)->task_ssm, (error)); \
        return; \
      } \
  } G_STMT_END

/* Shared cleanup state: tell the sensor to cancel whatever it was doing. */
static void
cancel_cb (FpiDevicePetaicMoc *self,
           const PetaicResp   *resp,
           GError             *error,
           gpointer            user_data)
{
  if (error)
    {
      fp_warn ("CANCEL failed: %s", error->message);
      g_error_free (error);
    }

  fpi_ssm_next_state (self->task_ssm);
}

/* Poll DETECT_FINGER_STATUS until the finger has left the sensor. */
static void
wait_finger_up_cb (FpiDevicePetaicMoc *self,
                   const PetaicResp   *resp,
                   GError             *error,
                   gpointer            user_data)
{
  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Finger detection"));
      return;
    }

  if (resp->payload[0] == 1)
    fpi_ssm_jump_to_state_delayed (self->task_ssm,
                                   fpi_ssm_get_cur_state (self->task_ssm),
                                   PETAIC_POLL_INTERVAL);
  else
    fpi_ssm_next_state (self->task_ssm);
}

static void
poll_finger_up (FpiDevicePetaicMoc *self)
{
  GError *error = NULL;

  /* These polls are not cancellable transfers, so look for cancellation here */
  if (g_cancellable_set_error_if_cancelled (fpi_device_get_cancellable (FP_DEVICE (self)), &error))
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  petaic_cmd (self, PETAIC_CMD_DETECT_FINGER, NULL, 0, FALSE, wait_finger_up_cb, NULL);
}

/******************************************************************************
 *
 *  Identify / verify
 *
 *****************************************************************************/

static void
identify_capture_cb (FpiDevicePetaicMoc *self,
                     const PetaicResp   *resp,
                     GError             *error,
                     gpointer            user_data)
{
  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      if (event_is_retry (resp->event))
        fpi_ssm_mark_failed (self->task_ssm, fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
      else
        fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Capture"));
      return;
    }

  fpi_device_report_finger_status_changes (FP_DEVICE (self),
                                           FP_FINGER_STATUS_PRESENT,
                                           FP_FINGER_STATUS_NONE);
  fpi_ssm_next_state (self->task_ssm);
}

static void
identify_result_cb (FpiDevicePetaicMoc *self,
                    const PetaicResp   *resp,
                    GError             *error,
                    gpointer            user_data)
{
  FpDevice *device = FP_DEVICE (self);
  FpPrint *new_scan = NULL;   /* floating, owned by fpi_device_identify_report() */
  FpPrint *matching = NULL;
  guint8 plain[100];
  guint32 result, decision;

  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Matching"));
      return;
    }

  decode_verify_result (resp->payload, plain);
  result = plain[0] | (plain[1] << 8) | (plain[2] << 16) | ((guint32) plain[3] << 24);
  decision = plain[4] | (plain[5] << 8) | (plain[6] << 16) | ((guint32) plain[7] << 24);

  fp_dbg ("Match result %u decision %u record %u", result, decision, plain[8]);

  if (result == 0 && decision == 1)
    {
      const guint8 *identity = plain + 12;

      if (identity_is_ours (identity))
        {
          GPtrArray *gallery = NULL;
          guint idx;

          new_scan = print_from_identity (self, identity);
          fpi_device_get_identify_data (device, &gallery);
          if (g_ptr_array_find_with_equal_func (gallery, new_scan,
                                                (GEqualFunc) fp_print_equal, &idx))
            matching = g_ptr_array_index (gallery, idx);
        }
      else
        {
          /* A template created by other software that shares the sensor: not ours to match */
          fp_dbg ("Matched a record that was not enrolled by this driver, ignoring");
        }
    }

  fpi_device_identify_report (device, matching, new_scan, NULL);
  fpi_ssm_next_state (self->task_ssm);
}

static void
identify_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case PETAIC_IDENTIFY_WAIT_FINGER_UP:
      poll_finger_up (self);
      break;

    case PETAIC_IDENTIFY_CAPTURE:
      fpi_device_report_finger_status_changes (device,
                                               FP_FINGER_STATUS_NEEDED,
                                               FP_FINGER_STATUS_NONE);
      petaic_cmd (self, PETAIC_CMD_VERIFY_CAPTURE, NULL, 0, TRUE,
                  identify_capture_cb, NULL);
      break;

    case PETAIC_IDENTIFY_RESULT:
      petaic_cmd (self, PETAIC_CMD_VERIFY_RESULT, NULL, 0, FALSE,
                  identify_result_cb, NULL);
      break;

    case PETAIC_IDENTIFY_CANCEL:
      petaic_cmd (self, PETAIC_CMD_CANCEL, NULL, 0, FALSE, cancel_cb, NULL);
      break;
    }
}

static void
identify_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (dev);

  self->task_ssm = NULL;

  if (error && error->domain == FP_DEVICE_RETRY)
    fpi_device_identify_report (dev, NULL, NULL, g_steal_pointer (&error));

  fpi_device_identify_complete (dev, error);
}

static void
petaic_identify (FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  self->task_ssm = fpi_ssm_new_full (device, identify_run_state,
                                     PETAIC_IDENTIFY_NUM_STATES,
                                     PETAIC_IDENTIFY_CANCEL,
                                     "identify");
  fpi_ssm_start (self->task_ssm, identify_ssm_done);
}

/******************************************************************************
 *
 *  Enroll
 *
 *****************************************************************************/

static void
enroll_db_size_cb (FpiDevicePetaicMoc *self,
                   const PetaicResp   *resp,
                   GError             *error,
                   gpointer            user_data)
{
  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Reading database size"));
      return;
    }

  self->db_free = resp->payload[0];
  self->db_used = resp->payload[8];
  fp_dbg ("Sensor database: %u used, %u free", self->db_used, self->db_free);

  if (self->db_free == 0)
    {
      fpi_ssm_mark_failed (self->task_ssm, fpi_device_error_new (FP_DEVICE_ERROR_DATA_FULL));
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
enroll_create_cb (FpiDevicePetaicMoc *self,
                  const PetaicResp   *resp,
                  GError             *error,
                  gpointer            user_data)
{
  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Creating enrollment"));
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
enroll_capture_cb (FpiDevicePetaicMoc *self,
                   const PetaicResp   *resp,
                   GError             *error,
                   gpointer            user_data)
{
  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      if (!event_is_retry (resp->event) ||
          ++self->capture_attempts > PETAIC_MAX_CAPTURE_ATTEMPTS)
        {
          fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Capture"));
          return;
        }

      fpi_device_enroll_progress (FP_DEVICE (self), self->enroll_accepted, NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
      fpi_ssm_jump_to_state (self->task_ssm, PETAIC_ENROLL_WAIT_FINGER_UP);
      return;
    }

  fpi_device_report_finger_status_changes (FP_DEVICE (self),
                                           FP_FINGER_STATUS_PRESENT,
                                           FP_FINGER_STATUS_NONE);
  fpi_ssm_next_state (self->task_ssm);
}

static void
enroll_apply_cb (FpiDevicePetaicMoc *self,
                 const PetaicResp   *resp,
                 GError             *error,
                 gpointer            user_data)
{
  FpDevice *device = FP_DEVICE (self);

  RETURN_IF_ERROR (self, error);

  if (resp->event == PETAIC_EV_ENROLL_NEED_MORE || resp->event == PETAIC_EV_ENROLL_DONE)
    {
      gint accepted = resp->payload[7];

      fp_dbg ("Enroll image accepted: %d/%d", accepted, resp->payload[4]);

      if (accepted > self->enroll_accepted)
        {
          self->enroll_accepted = accepted;
          fpi_device_enroll_progress (device, accepted, NULL, NULL);
        }

      if (resp->event == PETAIC_EV_ENROLL_DONE)
        fpi_ssm_next_state (self->task_ssm);
      else
        fpi_ssm_jump_to_state (self->task_ssm, PETAIC_ENROLL_WAIT_FINGER_UP);
      return;
    }

  /* Any other event rejects this image (overlap, quality, ...): ask for another try */
  if (++self->capture_attempts > PETAIC_MAX_CAPTURE_ATTEMPTS)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Enrollment"));
      return;
    }

  fp_dbg ("Enroll image rejected, event 0x%02x", resp->event);
  fpi_device_enroll_progress (device, self->enroll_accepted, NULL,
                              fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
  fpi_ssm_jump_to_state (self->task_ssm, PETAIC_ENROLL_WAIT_FINGER_UP);
}

static void
enroll_commit_cb (FpiDevicePetaicMoc *self,
                  const PetaicResp   *resp,
                  GError             *error,
                  gpointer            user_data)
{
  FpPrint *print = NULL;

  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Committing enrollment"));
      return;
    }

  fpi_device_get_enroll_data (FP_DEVICE (self), &print);
  set_print_data (print,
                  self->enroll_identity[PETAIC_IDENTITY_SUBFACTOR_OFFSET],
                  self->enroll_identity + 4);

  fpi_ssm_next_state (self->task_ssm);
}

static void
enroll_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case PETAIC_ENROLL_DB_SIZE:
      petaic_cmd (self, PETAIC_CMD_DB_GET_SIZE, NULL, 0, FALSE, enroll_db_size_cb, NULL);
      break;

    case PETAIC_ENROLL_CREATE:
      {
        FpPrint *print = NULL;
        guint8 guid[PETAIC_GUID_SIZE];

        /* The identity is how we recognise our own templates in the chip */
        fpi_device_get_enroll_data (device, &print);
        memcpy (guid, PETAIC_GUID_PREFIX, PETAIC_GUID_PREFIX_LEN);
        if (fpi_device_emulation_mode_enabled (device))
          {
            /* Tests replay recorded USB traffic, so the bytes sent have to be reproducible */
            memcpy (guid + PETAIC_GUID_PREFIX_LEN, "emulated-dev", PETAIC_GUID_SIZE - PETAIC_GUID_PREFIX_LEN);
          }
        else
          {
            for (guint i = PETAIC_GUID_PREFIX_LEN; i < PETAIC_GUID_SIZE; i += 4)
              {
                guint32 r = g_random_int ();
                memcpy (guid + i, &r, 4);
              }
          }
        identity_make (self->enroll_identity, guid, (guint8) fp_print_get_finger (print));

        petaic_cmd (self, PETAIC_CMD_ENROLL_CREATE, NULL, 0, FALSE, enroll_create_cb, NULL);
      }
      break;

    case PETAIC_ENROLL_WAIT_FINGER_UP:
      poll_finger_up (self);
      break;

    case PETAIC_ENROLL_CAPTURE:
      fpi_device_report_finger_status_changes (device,
                                               FP_FINGER_STATUS_NEEDED,
                                               FP_FINGER_STATUS_NONE);
      petaic_cmd (self, PETAIC_CMD_ENROLL_CAPTURE, NULL, 0, TRUE, enroll_capture_cb, NULL);
      break;

    case PETAIC_ENROLL_APPLY:
      petaic_cmd (self, PETAIC_CMD_ENROLL_APPLY, NULL, 0, FALSE, enroll_apply_cb, NULL);
      break;

    case PETAIC_ENROLL_COMMIT:
      petaic_cmd (self, PETAIC_CMD_ENROLL_COMMIT,
                  self->enroll_identity, PETAIC_IDENTITY_SIZE, FALSE,
                  enroll_commit_cb, NULL);
      break;

    case PETAIC_ENROLL_CANCEL:
      petaic_cmd (self, PETAIC_CMD_CANCEL, NULL, 0, FALSE, cancel_cb, NULL);
      break;
    }
}

static void
enroll_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (dev);
  FpPrint *print = NULL;

  self->task_ssm = NULL;

  if (error)
    {
      fpi_device_enroll_complete (dev, NULL, error);
      return;
    }

  fpi_device_get_enroll_data (dev, &print);
  fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}

static void
petaic_enroll (FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  self->enroll_accepted = 0;
  self->capture_attempts = 0;

  self->task_ssm = fpi_ssm_new_full (device, enroll_run_state,
                                     PETAIC_ENROLL_NUM_STATES,
                                     PETAIC_ENROLL_CANCEL,
                                     "enroll");
  fpi_ssm_start (self->task_ssm, enroll_ssm_done);
}

/******************************************************************************
 *
 *  List
 *
 *****************************************************************************/

static void
list_db_size_cb (FpiDevicePetaicMoc *self,
                 const PetaicResp   *resp,
                 GError             *error,
                 gpointer            user_data)
{
  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Reading database size"));
      return;
    }

  self->db_free = resp->payload[0];
  self->db_used = MIN (resp->payload[8], PETAIC_MAX_SLOTS);
  self->list_index = 0;
  self->list_result = g_ptr_array_new_with_free_func (g_object_unref);

  if (self->db_used == 0)
    fpi_ssm_jump_to_state (self->task_ssm, PETAIC_LIST_CANCEL);
  else
    fpi_ssm_next_state (self->task_ssm);
}

static void
list_dump_cb (FpiDevicePetaicMoc *self,
              const PetaicResp   *resp,
              GError             *error,
              gpointer            user_data)
{
  RETURN_IF_ERROR (self, error);

  if (resp->event != PETAIC_EV_OK)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Reading record"));
      return;
    }

  /* Records created by other software sharing the sensor are not ours and stay hidden */
  if (identity_is_ours (resp->payload + 4))
    g_ptr_array_add (self->list_result,
                     g_object_ref_sink (print_from_identity (self, resp->payload + 4)));

  if (++self->list_index < self->db_used)
    fpi_ssm_jump_to_state (self->task_ssm, PETAIC_LIST_DUMP);
  else
    fpi_ssm_next_state (self->task_ssm);
}

static void
list_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case PETAIC_LIST_DB_SIZE:
      petaic_cmd (self, PETAIC_CMD_DB_GET_SIZE, NULL, 0, FALSE, list_db_size_cb, NULL);
      break;

    case PETAIC_LIST_DUMP:
      {
        guint8 index = self->list_index;

        petaic_cmd (self, PETAIC_CMD_DB_DUMP_BY_INDEX, &index, 1, FALSE, list_dump_cb, NULL);
      }
      break;

    case PETAIC_LIST_CANCEL:
      petaic_cmd (self, PETAIC_CMD_CANCEL, NULL, 0, FALSE, cancel_cb, NULL);
      break;
    }
}

static void
list_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (dev);

  self->task_ssm = NULL;

  if (error)
    {
      g_clear_pointer (&self->list_result, g_ptr_array_unref);
      fpi_device_list_complete (dev, NULL, error);
      return;
    }

  fpi_device_list_complete (dev, g_steal_pointer (&self->list_result), NULL);
}

static void
petaic_list (FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  self->task_ssm = fpi_ssm_new_full (device, list_run_state,
                                     PETAIC_LIST_NUM_STATES,
                                     PETAIC_LIST_CANCEL,
                                     "list");
  fpi_ssm_start (self->task_ssm, list_ssm_done);
}

/******************************************************************************
 *
 *  Delete
 *
 *****************************************************************************/

static void
delete_record_cb (FpiDevicePetaicMoc *self,
                  const PetaicResp   *resp,
                  GError             *error,
                  gpointer            user_data)
{
  RETURN_IF_ERROR (self, error);

  if (resp->event == PETAIC_EV_RECORD_NOT_FOUND)
    {
      fpi_ssm_mark_failed (self->task_ssm, fpi_device_error_new (FP_DEVICE_ERROR_DATA_NOT_FOUND));
      return;
    }

  if (resp->event != PETAIC_EV_OK)
    {
      fpi_ssm_mark_failed (self->task_ssm, event_error (resp->event, "Deleting record"));
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
delete_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case PETAIC_DELETE_RECORD:
      {
        FpPrint *print = NULL;
        guint8 identity[PETAIC_IDENTITY_SIZE];

        fpi_device_get_delete_data (device, &print);

        /* Only records created by this driver can be deleted, never the ones of other OSes */
        if (!identity_from_print (print, identity))
          {
            fpi_ssm_mark_failed (ssm, fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
            return;
          }

        petaic_cmd (self, PETAIC_CMD_DB_DELETE, identity, PETAIC_IDENTITY_SIZE, FALSE,
                    delete_record_cb, NULL);
      }
      break;

    case PETAIC_DELETE_CANCEL:
      petaic_cmd (self, PETAIC_CMD_CANCEL, NULL, 0, FALSE, cancel_cb, NULL);
      break;
    }
}

static void
delete_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (dev);

  self->task_ssm = NULL;
  fpi_device_delete_complete (dev, error);
}

static void
petaic_delete (FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  self->task_ssm = fpi_ssm_new_full (device, delete_run_state,
                                     PETAIC_DELETE_NUM_STATES,
                                     PETAIC_DELETE_CANCEL,
                                     "delete");
  fpi_ssm_start (self->task_ssm, delete_ssm_done);
}

/******************************************************************************
 *
 *  Open / close / probe
 *
 *****************************************************************************/

static void
petaic_probe (FpDevice *device)
{
  GUsbDevice *usb_dev = fpi_device_get_usb_device (device);
  GError *error = NULL;

  if (!g_usb_device_open (usb_dev, &error))
    {
      fpi_device_probe_complete (device, NULL, NULL, error);
      return;
    }

  g_usb_device_close (usb_dev, NULL);
  fpi_device_probe_complete (device, NULL, NULL, NULL);
}

static void
open_cancel_cb (FpiDevicePetaicMoc *self,
                const PetaicResp   *resp,
                GError             *error,
                gpointer            user_data)
{
  FpDevice *device = FP_DEVICE (self);

  if (error)
    {
      g_autoptr(GError) release_error = NULL;

      g_usb_device_release_interface (fpi_device_get_usb_device (device), 0, 0, &release_error);
      fpi_device_open_complete (device, error);
      return;
    }

  fpi_device_open_complete (device, NULL);
}

static void
petaic_open (FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);
  GError *error = NULL;

  self->setup_done = FALSE;

  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (device), 0, 0, &error))
    {
      fpi_device_open_complete (device, error);
      return;
    }

  /* Bring the sensor into a known idle state (the Windows driver does the same) */
  petaic_cmd (self, PETAIC_CMD_CANCEL, NULL, 0, FALSE, open_cancel_cb, NULL);
}

static void
close_cancel_cb (FpiDevicePetaicMoc *self,
                 const PetaicResp   *resp,
                 GError             *error,
                 gpointer            user_data)
{
  FpDevice *device = FP_DEVICE (self);

  g_autoptr(GError) release_error = NULL;

  if (error)
    {
      fp_warn ("CANCEL on close failed: %s", error->message);
      g_error_free (error);
    }

  g_usb_device_release_interface (fpi_device_get_usb_device (device), 0, 0, &release_error);
  fpi_device_close_complete (device, g_steal_pointer (&release_error));
}

static void
petaic_close (FpDevice *device)
{
  FpiDevicePetaicMoc *self = FPI_DEVICE_PETAICMOC (device);

  petaic_cmd (self, PETAIC_CMD_CANCEL, NULL, 0, FALSE, close_cancel_cb, NULL);
}

/******************************************************************************
 *
 *  Class definition
 *
 *****************************************************************************/

static const FpIdEntry id_table[] = {
  { .vid = 0x28e9, .pid = 0x5151, },
  { .vid = 0, .pid = 0, .driver_data = 0 },   /* terminating entry */
};

static void
fpi_device_petaicmoc_init (FpiDevicePetaicMoc *self)
{
}

static void
fpi_device_petaicmoc_class_init (FpiDevicePetaicMocClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = "petaicmoc";
  dev_class->full_name = "Petaic MOC Fingerprint Sensor";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->id_table = id_table;
  dev_class->nr_enroll_stages = PETAIC_ENROLL_STAGES;
  dev_class->temp_hot_seconds = -1;

  dev_class->probe = petaic_probe;
  dev_class->open = petaic_open;
  dev_class->close = petaic_close;
  dev_class->enroll = petaic_enroll;
  dev_class->identify = petaic_identify;
  dev_class->list = petaic_list;
  dev_class->delete = petaic_delete;
  /* No clear_storage: it would also wipe templates created by other software */

  fpi_device_class_auto_initialize_features (dev_class);
}
