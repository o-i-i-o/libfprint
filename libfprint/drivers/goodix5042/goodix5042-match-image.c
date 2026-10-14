/*
 * Fingerprint matcher - image helpers and ridge enhancement.
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
#include "goodix5042-match-private.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* mirror index (d c b a | a b c d | d c b a) */
static inline int
reflect (int i, int n)
{
  if (n == 1)
    return 0;
  while (i < 0 || i >= n)
    i = i < 0 ? -i - 1 : 2 * n - i - 1;
  return i;
}

static void
convolve_1d (const float *src, float *dst, int w, int h, const float *k,
             int r, int horizontal)
{
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float s = 0;

        for (int i = -r; i <= r; i++)
          s += k[i + r] * (horizontal ? src[y * w + reflect (x + i, w)] :
                           src[reflect (y + i, h) * w + x]);
        dst[y * w + x] = s;
      }
}

void
fm_gaussian (const float *src, float *dst, int w, int h, float sigma)
{
  int r = (int) (4.0f * sigma + 0.5f);
  float *k, *tmp, sum = 0;

  if (r < 1)
    {
      memmove (dst, src, sizeof (float) * w * h);
      return;
    }
  k = malloc (sizeof (float) * (2 * r + 1));
  tmp = malloc (sizeof (float) * w * h);
  for (int j = 0; j < 2 * r + 1; j++)
    {
      int i = j - r;

      k[j] = expf (-0.5f * i * i / (sigma * sigma));
      sum += k[j];
    }
  for (int j = 0; j < 2 * r + 1; j++)
    k[j] /= sum;
  convolve_1d (src, tmp, w, h, k, r, 1);
  convolve_1d (tmp, dst, w, h, k, r, 0);
  free (tmp);
  free (k);
}

static void
box_mean (const float *src, float *dst, int w, int h, int size)
{
  int r = size / 2;
  float *k = malloc (sizeof (float) * size);
  float *tmp = malloc (sizeof (float) * w * h);

  for (int i = 0; i < size; i++)
    k[i] = 1.0f / size;
  convolve_1d (src, tmp, w, h, k, r, 1);
  convolve_1d (tmp, dst, w, h, k, r, 0);
  free (tmp);
  free (k);
}

/* erosion/dilation with a size x size square; outside = @border */
static void
morph (const uint8_t *src, uint8_t *dst, int w, int h, int size, int erode,
       int border)
{
  int r = size / 2;

  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        int v = erode;

        for (int dy = -r; dy <= r && v == erode; dy++)
          for (int dx = -r; dx <= r; dx++)
            {
              int yy = y + dy, xx = x + dx;
              int s = (yy < 0 || yy >= h || xx < 0 || xx >= w) ? border :
                      src[yy * w + xx];
              if (erode ? !s : s)
                {
                  v = !erode;
                  break;
                }
            }
        dst[y * w + x] = v;
      }
}

/* bicubic (a = -0.75) resize of a float image by @s */
void
fm_resize_cubic (const float *src, int w, int h, float *dst, int dw, int dh)
{
  const float a = -0.75f;
  float sx = (float) w / dw, sy = (float) h / dh;

  for (int y = 0; y < dh; y++)
    {
      float fy = (y + 0.5f) * sy - 0.5f;
      int iy = (int) floorf (fy);
      float ty = fy - iy, cy[4];

      for (int x = 0; x < dw; x++)
        {
          float fx = (x + 0.5f) * sx - 0.5f;
          int ix = (int) floorf (fx);
          float tx = fx - ix, cx[4], s = 0;

          for (int i = 0; i < 4; i++)
            {
              float d = fabsf ((i - 1) - tx);
              float e = fabsf ((i - 1) - ty);

              cx[i] = d <= 1 ? ((a + 2) * d - (a + 3)) * d * d + 1 :
                      d < 2 ? ((a * d - 5 * a) * d + 8 * a) * d - 4 * a : 0;
              cy[i] = e <= 1 ? ((a + 2) * e - (a + 3)) * e * e + 1 :
                      e < 2 ? ((a * e - 5 * a) * e + 8 * a) * e - 4 * a : 0;
            }
          for (int j = 0; j < 4; j++)
            {
              int yy = iy - 1 + j;
              float row = 0;

              yy = yy < 0 ? 0 : yy >= h ? h - 1 : yy;
              for (int i = 0; i < 4; i++)
                {
                  int xx = ix - 1 + i;

                  xx = xx < 0 ? 0 : xx >= w ? w - 1 : xx;
                  row += cx[i] * src[yy * w + xx];
                }
              s += cy[j] * row;
            }
          dst[y * dw + x] = s;
        }
    }
}

/*
 * Ridge enhancement. Output: @ridges (w x h, zero mean, unit local energy,
 * 0 outside the finger) and @mask (finger area).
 */
void
fm_enhance (const uint16_t *base, const uint16_t *frame, int w, int h,
            float *ridges, uint8_t *mask, int *coverage, int *quality)
{
  int n = w * h;
  float *d = calloc (n, sizeof (float));
  float *t = malloc (sizeof (float) * n);
  float *u = malloc (sizeof (float) * n);
  uint8_t *m = calloc (n, 1);
  int cnt = 0;
  double coh_sum = 0;

  /* contact strength, relative to the per-pixel base (gain) */
  for (int i = 0; i < n; i++)
    d[i] = ((float) base[i] - frame[i]) / (base[i] > 0 ? base[i] : 1);

  /* finger mask: smoothed contact above threshold, opened, eroded */
  box_mean (d, t, w, h, 9);
  for (int i = 0; i < n; i++)
    m[i] = t[i] > FM_MASK_THRESHOLD;
  morph (m, mask, w, h, 5, 1, 0);
  morph (mask, m, w, h, 5, 0, 0);
  morph (m, mask, w, h, 5, 1, 1);

  /* sensor lines (columns) have their own offset */
  for (int x = 0; x < w; x++)
    {
      double s = 0;
      int c = 0;

      for (int y = 0; y < h; y++)
        if (mask[y * w + x])
          {
            s += d[y * w + x];
            c++;
          }
      if (c > 4)
        for (int y = 0; y < h; y++)
          d[y * w + x] -= (float) (s / c);
    }

  /* band-pass around the ridge period (~8 px) */
  fm_gaussian (d, t, w, h, 1.0f);
  fm_gaussian (d, u, w, h, 4.0f);
  for (int i = 0; i < n; i++)
    d[i] = t[i] - u[i];

  /* local energy normalization */
  for (int i = 0; i < n; i++)
    t[i] = d[i] * d[i];
  fm_gaussian (t, u, w, h, 6.0f);
  for (int i = 0; i < n; i++)
    {
      ridges[i] = mask[i] ? d[i] / sqrtf (u[i] + 1e-9f) : 0;
      cnt += mask[i];
    }

  /* quality: mean orientation coherence of the ridges inside the finger */
  {
    float *gxx = calloc (n, sizeof (float));
    float *gyy = calloc (n, sizeof (float));
    float *gxy = calloc (n, sizeof (float));

    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
        {
          int xm = reflect (x - 1, w), xp = reflect (x + 1, w);
          int ym = reflect (y - 1, h), yp = reflect (y + 1, h);
          float gx = ridges[y * w + xp] - ridges[y * w + xm];
          float gy = ridges[yp * w + x] - ridges[ym * w + x];

          gxx[y * w + x] = gx * gx;
          gyy[y * w + x] = gy * gy;
          gxy[y * w + x] = gx * gy;
        }
    fm_gaussian (gxx, t, w, h, 4.0f);
    memcpy (gxx, t, sizeof (float) * n);
    fm_gaussian (gyy, t, w, h, 4.0f);
    memcpy (gyy, t, sizeof (float) * n);
    fm_gaussian (gxy, t, w, h, 4.0f);
    for (int i = 0; i < n; i++)
      if (mask[i])
        {
          float a = gxx[i] - gyy[i], b = 2 * t[i];

          coh_sum += sqrtf (a * a + b * b) / (gxx[i] + gyy[i] + 1e-9f);
        }
    free (gxx);
    free (gyy);
    free (gxy);
  }

  *coverage = (int) (100.0 * cnt / n + 0.5);
  *quality = cnt ? (int) (100.0 * coh_sum / cnt + 0.5) : 0;
  free (d);
  free (t);
  free (u);
  free (m);
}
