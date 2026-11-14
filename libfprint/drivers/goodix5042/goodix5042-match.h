/*
 * Fingerprint matcher for small capacitive area sensors.
 *
 * Images go through ridge enhancement, scale-space keypoints with
 * gradient-histogram descriptors are extracted, and two feature sets are
 * compared by descriptor matching followed by a RANSAC similarity model;
 * the score is the number of geometrically consistent matches.
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

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FM_DESC_LEN 128

typedef struct
{
  float   x, y;                 /* in enhanced-image pixels */
  float   angle;                /* dominant gradient orientation, rad */
  float   size;                 /* scale (sigma), enhanced-image pixels */
  uint8_t desc[FM_DESC_LEN];
} FmKeypoint;

typedef struct
{
  int         n;
  FmKeypoint *kp;
  int         coverage;         /* % of the sensor area touched */
  int         quality;          /* 0..100, ridge clarity */
} FmFeatures;

typedef struct
{
  /* enhancement */
  float upscale;                /* image scale before keypoint detection */
  float contrast;               /* 8-bit gain of the normalized ridges */
  /* keypoints */
  float contrast_threshold;
  float edge_threshold;
  int   double_image;           /* start the pyramid at 2x */
  int   max_keypoints;
  /* matching */
  float ratio;
  float ransac_threshold;       /* px of the original sensor image */
  float max_scale_change;
  float angle_tolerance;        /* keypoint orientation vs model, degrees */
  float size_tolerance;         /* |log(size ratio / model scale)|, 0 = off */
} FmParams;

void fm_params_default (FmParams *p);

/* base/frame: h x w raw sensor values, larger = no finger. Pixels outside
 * the active area must be excluded by the caller (pass the active area). */
int  fm_extract (const FmParams *p,
                 const uint16_t *base,
                 const uint16_t *frame,
                 int             w,
                 int             h,
                 FmFeatures     *out);
void fm_features_clear (FmFeatures *f);

/* number of consistent matches between two feature sets (>= 0) */
int  fm_match (const FmParams   *p,
               const FmFeatures *a,
               const FmFeatures *b);

/* serialization (little endian) */
size_t fm_serialized_size (const FmFeatures *f);
size_t fm_serialize (const FmFeatures *f,
                     uint8_t          *buf);
int    fm_deserialize (const uint8_t *buf,
                       size_t         len,
                       FmFeatures    *f,
                       size_t        *used);

#ifdef __cplusplus
}
#endif
