/*
 * Petaic match-on-chip driver for libfprint
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

#include "fpi-device.h"
#include "fpi-ssm.h"

G_DECLARE_FINAL_TYPE (FpiDevicePetaicMoc, fpi_device_petaicmoc, FPI, DEVICE_PETAICMOC, FpDevice)

/* Frames are always 128 bytes in both directions */
#define PETAIC_FRAME_SIZE 128
#define PETAIC_PAYLOAD_OFFSET 6
#define PETAIC_MAX_PAYLOAD 120
#define PETAIC_CHECKSUM_OFFSET 126

/* Vendor control request that must return PETAIC_SETUP_MAGIC before commands */
#define PETAIC_SETUP_REQUEST 0xC0
#define PETAIC_SETUP_MAGIC 0x5C

/* Sub-commands. Only the commands the Windows driver uses during normal operation. */
#define PETAIC_CMD_CANCEL 0x0C
#define PETAIC_CMD_DETECT_FINGER 0x30
#define PETAIC_CMD_VERIFY_CAPTURE 0xC0
#define PETAIC_CMD_VERIFY_RESULT 0xC1
#define PETAIC_CMD_ENROLL_CREATE 0xC2
#define PETAIC_CMD_ENROLL_COMMIT 0xC3
#define PETAIC_CMD_ENROLL_CAPTURE 0xC4
#define PETAIC_CMD_ENROLL_APPLY 0xC5
#define PETAIC_CMD_DB_GET_SIZE 0xD5
#define PETAIC_CMD_DB_DUMP_BY_INDEX 0xD9
#define PETAIC_CMD_DB_DELETE 0xDF

/* Response event codes (response byte 4) */
#define PETAIC_EV_OK 0x01
#define PETAIC_EV_ENROLLED_TABLE_IS_EMPTY 0x10
#define PETAIC_EV_RECORD_NOT_FOUND 0x26
#define PETAIC_EV_ENROLL_NEED_MORE 0xA6
#define PETAIC_EV_ENROLL_DONE 0xA7

/* Identity record stored in the chip (WINBIO_IDENTITY + subfactor) */
#define PETAIC_IDENTITY_SIZE 80
#define PETAIC_IDENTITY_TYPE_GUID 2
#define PETAIC_GUID_SIZE 16
#define PETAIC_GUID_PREFIX "LNX0"
#define PETAIC_GUID_PREFIX_LEN 4
#define PETAIC_IDENTITY_SUBFACTOR_OFFSET 76

typedef enum {
  PETAIC_ENROLL_DB_SIZE = 0,
  PETAIC_ENROLL_CREATE,
  PETAIC_ENROLL_WAIT_FINGER_UP,
  PETAIC_ENROLL_CAPTURE,
  PETAIC_ENROLL_APPLY,
  PETAIC_ENROLL_COMMIT,
  PETAIC_ENROLL_CANCEL,    /* cleanup state, always runs last */
  PETAIC_ENROLL_NUM_STATES,
} PetaicEnrollState;

typedef enum {
  PETAIC_IDENTIFY_WAIT_FINGER_UP = 0,
  PETAIC_IDENTIFY_CAPTURE,
  PETAIC_IDENTIFY_RESULT,
  PETAIC_IDENTIFY_CANCEL,  /* cleanup state, always runs last */
  PETAIC_IDENTIFY_NUM_STATES,
} PetaicIdentifyState;

typedef enum {
  PETAIC_LIST_DB_SIZE = 0,
  PETAIC_LIST_DUMP,
  PETAIC_LIST_CANCEL,      /* cleanup state, always runs last */
  PETAIC_LIST_NUM_STATES,
} PetaicListState;

typedef enum {
  PETAIC_DELETE_RECORD = 0,
  PETAIC_DELETE_CANCEL,    /* cleanup state, always runs last */
  PETAIC_DELETE_NUM_STATES,
} PetaicDeleteState;
