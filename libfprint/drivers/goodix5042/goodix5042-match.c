/*
 * Fingerprint matcher - feature extraction, matching, serialization.
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

#define RANSAC_ITERS 1000

void
fm_params_default (FmParams *p)
{
  p->upscale = 2.0f;
  p->contrast = 45.0f;
  p->contrast_threshold = 0.04f;
  p->edge_threshold = 20.0f;
  p->double_image = 1;
  p->max_keypoints = 0;
  p->ratio = 0.8f;
  p->ransac_threshold = 3.0f;
  p->max_scale_change = 0.15f;
  p->angle_tolerance = 20;
  p->size_tolerance = 0;
}

int
fm_extract (const FmParams *p, const uint16_t *base, const uint16_t *frame,
            int w, int h, FmFeatures *out)
{
  int n = w * h;
  int uw = (int) roundf (w * p->upscale), uh = (int) roundf (h * p->upscale);
  float *ridges = malloc (sizeof (float) * n);
  float *img = malloc (sizeof (float) * n);
  float *up = malloc (sizeof (float) * uw * uh);
  uint8_t *mask = malloc (n);
  uint8_t *umask = malloc (uw * uh);

  memset (out, 0, sizeof *out);
  fm_enhance (base, frame, w, h, ridges, mask, &out->coverage, &out->quality);

  /* 8-bit like dynamic range, then 0..1 */
  for (int i = 0; i < n; i++)
    {
      float v = ridges[i] * p->contrast + 128.0f;

      v = v < 0 ? 0 : v > 255 ? 255 : v;
      img[i] = floorf (v) / 255.0f;
    }
  fm_resize_cubic (img, w, h, up, uw, uh);
  for (int y = 0; y < uh; y++)
    for (int x = 0; x < uw; x++)
      {
        int sy = (int) floorf (y * (float) h / uh), sx = (int) floorf (x * (float) w / uw);

        umask[y * uw + x] = mask[sy * w + sx];
      }

  out->n = fm_keypoints (p, up, umask, uw, uh, &out->kp);

  free (ridges);
  free (img);
  free (up);
  free (mask);
  free (umask);
  return 0;
}

void
fm_features_clear (FmFeatures *f)
{
  free (f->kp);
  memset (f, 0, sizeof *f);
}

/* ------------------------------------------------------------------ match */

typedef struct
{
  float ax, ay, bx, by;
  float dangle;                 /* angle b - angle a */
  float dsize;                  /* log (size b / size a) */
} Pair;

typedef struct
{
  float thr2, atol, stol;
} Tol;

static inline float
wrap_pi (float a)
{
  while (a > (float) M_PI)
    a -= 2 * (float) M_PI;
  while (a < -(float) M_PI)
    a += 2 * (float) M_PI;
  return a;
}

static inline uint32_t
dist2 (const uint8_t *a, const uint8_t *b)
{
  uint32_t s = 0;

  for (int i = 0; i < FM_DESC_LEN; i++)
    {
      int d = (int) a[i] - (int) b[i];

      s += (uint32_t) (d * d);
    }
  return s;
}

/* similarity b = s R a + t from two pairs */
static int
model2 (const Pair *p, const Pair *q, float m[4])
{
  float ax = q->ax - p->ax, ay = q->ay - p->ay;
  float bx = q->bx - p->bx, by = q->by - p->by;
  float den = ax * ax + ay * ay;

  if (den < 1e-3f)
    return 0;
  /* [a -b; b a] */
  m[0] = (ax * bx + ay * by) / den;
  m[1] = (ax * by - ay * bx) / den;
  m[2] = p->bx - (m[0] * p->ax - m[1] * p->ay);
  m[3] = p->by - (m[1] * p->ax + m[0] * p->ay);
  return 1;
}

static int
count_inliers (const Pair *pr, int n, const float m[4], const Tol *t, uint8_t *in)
{
  float rot = atan2f (m[1], m[0]);
  float lsc = logf (fmaxf (hypotf (m[0], m[1]), 1e-6f));
  int c = 0;

  for (int i = 0; i < n; i++)
    {
      float x = m[0] * pr[i].ax - m[1] * pr[i].ay + m[2] - pr[i].bx;
      float y = m[1] * pr[i].ax + m[0] * pr[i].ay + m[3] - pr[i].by;
      int ok = x * x + y * y <= t->thr2;

      /* the gradient orientation is measured with y pointing up */
      if (ok && t->atol > 0)
        ok = fabsf (wrap_pi (pr[i].dangle + rot)) <= t->atol;
      if (ok && t->stol > 0)
        ok = fabsf (pr[i].dsize - lsc) <= t->stol;

      if (in)
        in[i] = ok;
      c += ok;
    }
  return c;
}

/* least squares similarity on the inliers */
static int
refit (const Pair *pr, int n, const uint8_t *in, float m[4])
{
  double sax = 0, say = 0, sbx = 0, sby = 0, c = 0;
  double num_a = 0, num_b = 0, den = 0;

  for (int i = 0; i < n; i++)
    if (in[i])
      {
        sax += pr[i].ax;
        say += pr[i].ay;
        sbx += pr[i].bx;
        sby += pr[i].by;
        c++;
      }
  if (c < 2)
    return 0;
  sax /= c;
  say /= c;
  sbx /= c;
  sby /= c;
  for (int i = 0; i < n; i++)
    if (in[i])
      {
        double ax = pr[i].ax - sax, ay = pr[i].ay - say;
        double bx = pr[i].bx - sbx, by = pr[i].by - sby;

        num_a += ax * bx + ay * by;
        num_b += ax * by - ay * bx;
        den += ax * ax + ay * ay;
      }
  if (den < 1e-6)
    return 0;
  m[0] = (float) (num_a / den);
  m[1] = (float) (num_b / den);
  m[2] = (float) (sbx - (m[0] * sax - m[1] * say));
  m[3] = (float) (sby - (m[1] * sax + m[0] * say));
  return 1;
}

int
fm_match (const FmParams *p, const FmFeatures *a, const FmFeatures *b)
{
  Pair *pr;
  uint8_t *in;
  int np = 0, best = 0;
  float bm[4] = { 1, 0, 0, 0 };
  float thr = p->ransac_threshold * p->upscale;
  Tol tol = { thr * thr, p->angle_tolerance * (float) M_PI / 180, p->size_tolerance };
  float r2 = p->ratio * p->ratio;
  uint32_t rng = 0x12345678u;

  if (a->n < 2 || b->n < 2)
    return 0;
  pr = malloc (sizeof (Pair) * a->n);
  in = malloc (a->n);

  for (int i = 0; i < a->n; i++)
    {
      uint32_t d1 = UINT32_MAX, d2 = UINT32_MAX;
      int j1 = -1;

      for (int j = 0; j < b->n; j++)
        {
          uint32_t d = dist2 (a->kp[i].desc, b->kp[j].desc);

          if (d < d1)
            {
              d2 = d1;
              d1 = d;
              j1 = j;
            }
          else if (d < d2)
            {
              d2 = d;
            }
        }
      if (j1 >= 0 && (float) d1 < r2 * (float) d2)
        {
          pr[np].ax = a->kp[i].x;
          pr[np].ay = a->kp[i].y;
          pr[np].bx = b->kp[j1].x;
          pr[np].by = b->kp[j1].y;
          pr[np].dangle = wrap_pi (b->kp[j1].angle - a->kp[i].angle);
          pr[np].dsize = logf (b->kp[j1].size / a->kp[i].size);
          np++;
        }
    }

  if (np >= 2)
    {
      for (int it = 0; it < RANSAC_ITERS; it++)
        {
          float m[4];
          int i, j, c;

          rng = rng * 1664525u + 1013904223u;
          i = (int) ((rng >> 8) % (uint32_t) np);
          rng = rng * 1664525u + 1013904223u;
          j = (int) ((rng >> 8) % (uint32_t) np);
          if (i == j || !model2 (&pr[i], &pr[j], m))
            continue;
          c = count_inliers (pr, np, m, &tol, NULL);
          if (c > best)
            {
              best = c;
              memcpy (bm, m, sizeof bm);
            }
        }
      if (best >= 2)
        {
          float m[4];

          count_inliers (pr, np, bm, &tol, in);
          if (refit (pr, np, in, m))
            {
              int c = count_inliers (pr, np, m, &tol, NULL);

              if (c >= best)
                {
                  best = c;
                  memcpy (bm, m, sizeof bm);
                }
            }
        }
      if (fabsf (hypotf (bm[0], bm[1]) - 1.0f) > p->max_scale_change)
        best = 0;
    }

  free (pr);
  free (in);
  return best < 2 ? 0 : best;
}

/* ---------------------------------------------------------- serialization */

#define KP_BYTES (16 + FM_DESC_LEN)

size_t
fm_serialized_size (const FmFeatures *f)
{
  return 12 + (size_t) f->n * KP_BYTES;
}

static void
put32 (uint8_t *b, uint32_t v)
{
  b[0] = v;
  b[1] = v >> 8;
  b[2] = v >> 16;
  b[3] = v >> 24;
}

static uint32_t
get32 (const uint8_t *b)
{
  return b[0] | (b[1] << 8) | (b[2] << 16) | ((uint32_t) b[3] << 24);
}

size_t
fm_serialize (const FmFeatures *f, uint8_t *buf)
{
  uint8_t *p = buf;

  put32 (p, (uint32_t) f->n);
  put32 (p + 4, (uint32_t) f->coverage);
  put32 (p + 8, (uint32_t) f->quality);
  p += 12;
  for (int i = 0; i < f->n; i++)
    {
      uint32_t v[4];

      memcpy (&v[0], &f->kp[i].x, 4);
      memcpy (&v[1], &f->kp[i].y, 4);
      memcpy (&v[2], &f->kp[i].angle, 4);
      memcpy (&v[3], &f->kp[i].size, 4);
      for (int j = 0; j < 4; j++)
        put32 (p + 4 * j, v[j]);
      memcpy (p + 16, f->kp[i].desc, FM_DESC_LEN);
      p += KP_BYTES;
    }
  return (size_t) (p - buf);
}

int
fm_deserialize (const uint8_t *buf, size_t len, FmFeatures *f, size_t *used)
{
  uint32_t n;

  memset (f, 0, sizeof *f);
  if (len < 12)
    return -1;
  n = get32 (buf);
  if (n > 100000 || len < 12 + (size_t) n * KP_BYTES)
    return -1;
  f->n = (int) n;
  f->coverage = (int) get32 (buf + 4);
  f->quality = (int) get32 (buf + 8);
  f->kp = calloc (n ? n : 1, sizeof (FmKeypoint));
  for (uint32_t i = 0; i < n; i++)
    {
      const uint8_t *p = buf + 12 + (size_t) i * KP_BYTES;
      uint32_t v[4];

      for (int j = 0; j < 4; j++)
        v[j] = get32 (p + 4 * j);
      memcpy (&f->kp[i].x, &v[0], 4);
      memcpy (&f->kp[i].y, &v[1], 4);
      memcpy (&f->kp[i].angle, &v[2], 4);
      memcpy (&f->kp[i].size, &v[3], 4);
      memcpy (f->kp[i].desc, p + 16, FM_DESC_LEN);
    }
  if (used)
    *used = 12 + (size_t) n * KP_BYTES;
  return 0;
}
