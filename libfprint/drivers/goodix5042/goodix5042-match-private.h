/*
 * Fingerprint matcher - internal helpers.
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

#include "goodix5042-match.h"

#define FM_MASK_THRESHOLD 0.15f

void fm_gaussian (const float *src,
                  float       *dst,
                  int          w,
                  int          h,
                  float        sigma);
void fm_resize_cubic (const float *src,
                      int          w,
                      int          h,
                      float       *dst,
                      int          dw,
                      int          dh);
void fm_enhance (const uint16_t *base,
                 const uint16_t *frame,
                 int             w,
                 int             h,
                 float          *ridges,
                 uint8_t        *mask,
                 int            *coverage,
                 int            *quality);

/* keypoints of @img (w x h, 0..1) inside @mask (same size) */
int  fm_keypoints (const FmParams *p,
                   const float    *img,
                   const uint8_t  *mask,
                   int             w,
                   int             h,
                   FmKeypoint    **out);
