/*
 * Fingerprint matcher - scale-space keypoints and descriptors.
 *
 * Difference-of-Gaussian extrema with sub-pixel refinement, a dominant
 * gradient orientation per keypoint and a 4x4x8 gradient-orientation
 * histogram descriptor (the classic scale invariant feature scheme).
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

#define SCALES 3                 /* intervals per octave */
#define SIGMA0 1.6f
#define INIT_SIGMA 0.5f
#define BORDER 5
#define MAX_INTERP 5
#define ORI_BINS 36
#define ORI_RADIUS (3 * 1.5f)
#define ORI_PEAK 0.8f
#define DESC_W 4
#define DESC_BINS 8
#define DESC_SCALE 3.0f
#define DESC_CLAMP 0.2f

typedef struct
{
  int    w, h;
  float *g[SCALES + 3];          /* Gaussian images */
  float *d[SCALES + 2];          /* differences */
} Octave;

typedef struct
{
  FmKeypoint *v;
  int         n, cap;
} KpList;

static FmKeypoint *
kp_push (KpList *l)
{
  if (l->n == l->cap)
    {
      l->cap = l->cap ? 2 * l->cap : 256;
      l->v = realloc (l->v, sizeof (FmKeypoint) * l->cap);
    }
  return &l->v[l->n++];
}

static void
downsample (const float *src, int w, int h, float *dst)
{
  int dw = w / 2, dh = h / 2;

  for (int y = 0; y < dh; y++)
    for (int x = 0; x < dw; x++)
      dst[y * dw + x] = src[2 * y * w + 2 * x];
}

static void
build_octaves (const float *base, int w, int h, Octave *oct, int noct)
{
  float sig[SCALES + 3];
  float k = powf (2.0f, 1.0f / SCALES);

  sig[0] = SIGMA0;
  for (int i = 1; i < SCALES + 3; i++)
    {
      float prev = powf (k, i - 1) * SIGMA0, total = prev * k;

      sig[i] = sqrtf (total * total - prev * prev);
    }

  for (int o = 0; o < noct; o++)
    {
      Octave *O = &oct[o];

      O->w = o ? oct[o - 1].w / 2 : w;
      O->h = o ? oct[o - 1].h / 2 : h;
      for (int i = 0; i < SCALES + 3; i++)
        O->g[i] = malloc (sizeof (float) * O->w * O->h);
      if (o == 0)
        memcpy (O->g[0], base, sizeof (float) * w * h);
      else
        downsample (oct[o - 1].g[SCALES], oct[o - 1].w, oct[o - 1].h, O->g[0]);
      for (int i = 1; i < SCALES + 3; i++)
        fm_gaussian (O->g[i - 1], O->g[i], O->w, O->h, sig[i]);
      for (int i = 0; i < SCALES + 2; i++)
        {
          O->d[i] = malloc (sizeof (float) * O->w * O->h);
          for (int j = 0; j < O->w * O->h; j++)
            O->d[i][j] = O->g[i + 1][j] - O->g[i][j];
        }
    }
}

static void
free_octaves (Octave *oct, int noct)
{
  for (int o = 0; o < noct; o++)
    {
      for (int i = 0; i < SCALES + 3; i++)
        free (oct[o].g[i]);
      for (int i = 0; i < SCALES + 2; i++)
        free (oct[o].d[i]);
    }
}

#define D(s, y, x) (O->d[(s)][(y) * w + (x)])

static int
is_extremum (const Octave *O, int s, int y, int x, float thr)
{
  int w = O->w;
  float v = D (s, y, x);

  if (fabsf (v) <= thr)
    return 0;
  for (int ds = -1; ds <= 1; ds++)
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++)
        {
          float n;

          if (!ds && !dy && !dx)
            continue;
          n = D (s + ds, y + dy, x + dx);
          if ((v > 0 && n >= v) || (v < 0 && n <= v))
            return 0;
        }
  return 1;
}

/* quadratic refinement; returns 0 if the point is rejected */
static int
refine (const FmParams *p, const Octave *O, int *ps, int *py, int *px,
        float *os, float *oy, float *ox, float *contrast)
{
  int w = O->w, h = O->h, s = *ps, y = *py, x = *px;
  float xs = 0, xy = 0, xx = 0;
  float dx, dy, dsg, dxx, dyy, dss, dxy, dxs, dys, v;
  int it;

  for (it = 0; it < MAX_INTERP; it++)
    {
      float H[3][3], g[3], det, inv[3][3];

      v = D (s, y, x);
      dx = (D (s, y, x + 1) - D (s, y, x - 1)) * 0.5f;
      dy = (D (s, y + 1, x) - D (s, y - 1, x)) * 0.5f;
      dsg = (D (s + 1, y, x) - D (s - 1, y, x)) * 0.5f;
      dxx = D (s, y, x + 1) + D (s, y, x - 1) - 2 * v;
      dyy = D (s, y + 1, x) + D (s, y - 1, x) - 2 * v;
      dss = D (s + 1, y, x) + D (s - 1, y, x) - 2 * v;
      dxy = (D (s, y + 1, x + 1) - D (s, y + 1, x - 1) - D (s, y - 1, x + 1) + D (s, y - 1, x - 1)) * 0.25f;
      dxs = (D (s + 1, y, x + 1) - D (s + 1, y, x - 1) - D (s - 1, y, x + 1) + D (s - 1, y, x - 1)) * 0.25f;
      dys = (D (s + 1, y + 1, x) - D (s + 1, y - 1, x) - D (s - 1, y + 1, x) + D (s - 1, y - 1, x)) * 0.25f;

      H[0][0] = dxx;
      H[0][1] = dxy;
      H[0][2] = dxs;
      H[1][0] = dxy;
      H[1][1] = dyy;
      H[1][2] = dys;
      H[2][0] = dxs;
      H[2][1] = dys;
      H[2][2] = dss;
      g[0] = dx;
      g[1] = dy;
      g[2] = dsg;
      det = H[0][0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1]) -
            H[0][1] * (H[1][0] * H[2][2] - H[1][2] * H[2][0]) +
            H[0][2] * (H[1][0] * H[2][1] - H[1][1] * H[2][0]);
      if (fabsf (det) < 1e-12f)
        return 0;
      inv[0][0] = (H[1][1] * H[2][2] - H[1][2] * H[2][1]) / det;
      inv[0][1] = (H[0][2] * H[2][1] - H[0][1] * H[2][2]) / det;
      inv[0][2] = (H[0][1] * H[1][2] - H[0][2] * H[1][1]) / det;
      inv[1][0] = (H[1][2] * H[2][0] - H[1][0] * H[2][2]) / det;
      inv[1][1] = (H[0][0] * H[2][2] - H[0][2] * H[2][0]) / det;
      inv[1][2] = (H[0][2] * H[1][0] - H[0][0] * H[1][2]) / det;
      inv[2][0] = (H[1][0] * H[2][1] - H[1][1] * H[2][0]) / det;
      inv[2][1] = (H[0][1] * H[2][0] - H[0][0] * H[2][1]) / det;
      inv[2][2] = (H[0][0] * H[1][1] - H[0][1] * H[1][0]) / det;
      xx = -(inv[0][0] * g[0] + inv[0][1] * g[1] + inv[0][2] * g[2]);
      xy = -(inv[1][0] * g[0] + inv[1][1] * g[1] + inv[1][2] * g[2]);
      xs = -(inv[2][0] * g[0] + inv[2][1] * g[1] + inv[2][2] * g[2]);
      if (fabsf (xs) < 0.5f && fabsf (xy) < 0.5f && fabsf (xx) < 0.5f)
        break;
      x += (int) roundf (xx);
      y += (int) roundf (xy);
      s += (int) roundf (xs);
      if (s < 1 || s > SCALES || x < BORDER || x >= w - BORDER ||
          y < BORDER || y >= h - BORDER)
        return 0;
    }
  if (it >= MAX_INTERP)
    return 0;

  v = D (s, y, x);
  *contrast = v + 0.5f * (dx * xx + dy * xy + dsg * xs);
  if (fabsf (*contrast) * SCALES < p->contrast_threshold)
    return 0;

  /* principal curvature ratio (edges are unstable) */
  {
    float tr = dxx + dyy, det = dxx * dyy - dxy * dxy;
    float r = p->edge_threshold;

    if (det <= 0 || tr * tr * r >= (r + 1) * (r + 1) * det)
      return 0;
  }

  *ps = s;
  *py = y;
  *px = x;
  *os = xs;
  *oy = xy;
  *ox = xx;
  return 1;
}

#undef D

static int
orientations (const float *img, int w, int h, int x, int y, float sigma,
              float *angles)
{
  float hist[ORI_BINS] = { 0 }, sm[ORI_BINS];
  int r = (int) roundf (ORI_RADIUS * sigma);
  float wf = -1.0f / (2.0f * (1.5f * sigma) * (1.5f * sigma));
  float maxv = 0;
  int n = 0;

  for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++)
      {
        int yy = y + dy, xx = x + dx;
        float gx, gy, mag, ang;
        int bin;

        if (yy <= 0 || yy >= h - 1 || xx <= 0 || xx >= w - 1)
          continue;
        gx = img[yy * w + xx + 1] - img[yy * w + xx - 1];
        gy = img[(yy - 1) * w + xx] - img[(yy + 1) * w + xx];
        mag = sqrtf (gx * gx + gy * gy);
        ang = atan2f (gy, gx);
        bin = (int) roundf (ORI_BINS * (ang + (float) M_PI) / (2 * (float) M_PI));
        bin = (bin % ORI_BINS + ORI_BINS) % ORI_BINS;
        hist[bin] += expf ((dx * dx + dy * dy) * wf) * mag;
      }
  for (int i = 0; i < ORI_BINS; i++)
    {
      sm[i] = (hist[(i + ORI_BINS - 2) % ORI_BINS] + hist[(i + 2) % ORI_BINS]) * (1.0f / 16) +
              (hist[(i + ORI_BINS - 1) % ORI_BINS] + hist[(i + 1) % ORI_BINS]) * (4.0f / 16) +
              hist[i] * (6.0f / 16);
      if (sm[i] > maxv)
        maxv = sm[i];
    }
  for (int i = 0; i < ORI_BINS && n < 4; i++)
    {
      float l = sm[(i + ORI_BINS - 1) % ORI_BINS], c = sm[i], rr = sm[(i + 1) % ORI_BINS];

      if (c > l && c > rr && c >= ORI_PEAK * maxv)
        {
          float b = i + 0.5f * (l - rr) / (l - 2 * c + rr);

          angles[n++] = b * 2 * (float) M_PI / ORI_BINS - (float) M_PI;
        }
    }
  return n;
}

static void
descriptor (const float *img, int w, int h, float fx, float fy, float sigma,
            float angle, uint8_t *out)
{
  float hist[(DESC_W + 2) * (DESC_W + 2) * (DESC_BINS + 2)];
  float hw = DESC_SCALE * sigma;
  int r = (int) roundf (hw * 1.41421356f * (DESC_W + 1) * 0.5f);
  float c = cosf (angle) / hw, s = sinf (angle) / hw;
  float wf = -1.0f / (0.5f * DESC_W * DESC_W);
  float v[DESC_W * DESC_W * DESC_BINS], norm = 0, thr;
  int x = (int) roundf (fx), y = (int) roundf (fy);

  memset (hist, 0, sizeof hist);
  for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++)
      {
        float rx = c * dx - s * dy, ry = s * dx + c * dy;
        float rbin = ry + DESC_W / 2 - 0.5f, cbin = rx + DESC_W / 2 - 0.5f;
        int yy = y + dy, xx = x + dx;
        float gx, gy, mag, ori, obin;
        int r0, c0, o0;
        float dr, dc, dob;

        if (rbin <= -1 || rbin >= DESC_W || cbin <= -1 || cbin >= DESC_W ||
            yy <= 0 || yy >= h - 1 || xx <= 0 || xx >= w - 1)
          continue;
        gx = img[yy * w + xx + 1] - img[yy * w + xx - 1];
        gy = img[(yy - 1) * w + xx] - img[(yy + 1) * w + xx];
        mag = sqrtf (gx * gx + gy * gy) * expf ((rx * rx + ry * ry) * wf);
        ori = atan2f (gy, gx) - angle;
        while (ori < 0)
          ori += 2 * (float) M_PI;
        while (ori >= 2 * (float) M_PI)
          ori -= 2 * (float) M_PI;
        obin = ori * DESC_BINS / (2 * (float) M_PI);

        r0 = (int) floorf (rbin);
        c0 = (int) floorf (cbin);
        o0 = (int) floorf (obin);
        dr = rbin - r0;
        dc = cbin - c0;
        dob = obin - o0;
        for (int i = 0; i < 2; i++)
          for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
              {
                float wv = mag * (i ? dr : 1 - dr) * (j ? dc : 1 - dc) * (k ? dob : 1 - dob);
                int ri = r0 + i + 1, ci = c0 + j + 1, oi = (o0 + k) % DESC_BINS;

                hist[(ri * (DESC_W + 2) + ci) * (DESC_BINS + 2) + oi] += wv;
              }
      }

  for (int i = 0; i < DESC_W; i++)
    for (int j = 0; j < DESC_W; j++)
      for (int k = 0; k < DESC_BINS; k++)
        {
          float t = hist[((i + 1) * (DESC_W + 2) + j + 1) * (DESC_BINS + 2) + k];

          v[(i * DESC_W + j) * DESC_BINS + k] = t;
          norm += t * t;
        }
  thr = sqrtf (norm) * DESC_CLAMP;
  norm = 0;
  for (int i = 0; i < DESC_W * DESC_W * DESC_BINS; i++)
    {
      if (v[i] > thr)
        v[i] = thr;
      norm += v[i] * v[i];
    }
  norm = 512.0f / fmaxf (sqrtf (norm), 1e-7f);
  for (int i = 0; i < DESC_W * DESC_W * DESC_BINS; i++)
    {
      float t = v[i] * norm;

      out[i] = t > 255 ? 255 : (uint8_t) t;
    }
}

static int
cmp_contrast (const void *a, const void *b)
{
  float x = ((const float *) a)[0], y = ((const float *) b)[0];

  return x < y ? 1 : x > y ? -1 : 0;
}

int
fm_keypoints (const FmParams *p, const float *img, const uint8_t *mask,
              int w, int h, FmKeypoint **out)
{
  float scale0 = p->double_image ? 2.0f : 1.0f;
  int bw = (int) (w * scale0), bh = (int) (h * scale0);
  float *base = malloc (sizeof (float) * bw * bh);
  float *tmp = malloc (sizeof (float) * bw * bh);
  int noct = (int) floorf (log2f ((float) (bw < bh ? bw : bh))) - 2;
  Octave oct[8];
  KpList kl = { 0 };

  float (*rank)[2] = NULL;
  float thr = 0.5f * p->contrast_threshold / SCALES;

  if (noct > 8)
    noct = 8;
  if (noct < 1)
    noct = 1;

  /* initial blur: the input is assumed to have INIT_SIGMA already */
  if (p->double_image)
    fm_resize_cubic (img, w, h, tmp, bw, bh);
  else
    memcpy (tmp, img, sizeof (float) * w * h);
  {
    float s0 = INIT_SIGMA * scale0;

    fm_gaussian (tmp, base, bw, bh, sqrtf (fmaxf (SIGMA0 * SIGMA0 - s0 * s0, 0.01f)));
  }
  build_octaves (base, bw, bh, oct, noct);

  for (int o = 0; o < noct; o++)
    {
      Octave *O = &oct[o];
      float ofs = powf (2.0f, (float) o) / scale0;   /* octave px -> input px */

      for (int s = 1; s <= SCALES; s++)
        for (int y = BORDER; y < O->h - BORDER; y++)
          for (int x = BORDER; x < O->w - BORDER; x++)
            {
              int ss = s, yy = y, xx = x;
              float os, oy, ox, contrast, sigma, angles[4];
              int mx, my, na;

              if (!is_extremum (O, s, y, x, thr))
                continue;
              if (!refine (p, O, &ss, &yy, &xx, &os, &oy, &ox, &contrast))
                continue;
              mx = (int) roundf ((xx + ox) * ofs);
              my = (int) roundf ((yy + oy) * ofs);
              if (mx < 0 || mx >= w || my < 0 || my >= h || !mask[my * w + mx])
                continue;
              sigma = SIGMA0 * powf (2.0f, (ss + os) / SCALES);
              na = orientations (O->g[ss], O->w, O->h, xx, yy, sigma, angles);
              for (int a = 0; a < na; a++)
                {
                  FmKeypoint *k = kp_push (&kl);

                  k->x = (xx + ox) * ofs;
                  k->y = (yy + oy) * ofs;
                  k->angle = angles[a];
                  k->size = sigma * ofs;
                  descriptor (O->g[ss], O->w, O->h, xx + ox, yy + oy, sigma,
                              angles[a], k->desc);
                  rank = realloc (rank, sizeof (*rank) * kl.cap);
                  rank[kl.n - 1][0] = fabsf (contrast);
                  rank[kl.n - 1][1] = (float) (kl.n - 1);
                }
            }
    }

  /* keep the strongest keypoints */
  if (p->max_keypoints > 0 && kl.n > p->max_keypoints)
    {
      FmKeypoint *v = malloc (sizeof (FmKeypoint) * p->max_keypoints);

      qsort (rank, kl.n, sizeof (*rank), cmp_contrast);
      for (int i = 0; i < p->max_keypoints; i++)
        v[i] = kl.v[(int) rank[i][1]];
      free (kl.v);
      kl.v = v;
      kl.n = p->max_keypoints;
    }

  free_octaves (oct, noct);
  free (rank);
  free (base);
  free (tmp);
  *out = kl.v;
  return kl.n;
}
