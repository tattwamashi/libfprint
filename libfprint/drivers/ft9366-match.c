/*
 * FocalTech FT9366 image processing and matching
 *
 * Copyright (C) 2026 v8v88v8v88
 * Copyright (C) 2026 Tattwamashi Nayak
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

#include "ft9366-match.h"

#include <math.h>
#include <string.h>

#define FT9366_NORM_RADIUS 7     /* local contrast window: (2r+1)^2 */
#define FT9366_MASK_RATIO 0.3    /* below this share of the strongest contrast = no finger */
#define FT9366_MIN_SIGNAL 40     /* raw spread of background - frame below this = no finger */

/* ---- image processing --------------------------------------------------- */

void
ft9366_dead_pixels (const guint16 *background, guint8 *dead)
{
  guint64 sum = 0;

  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    sum += background[i];
  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    dead[i] = background[i] < sum / FT9366_FRAME_PIXELS / 4;
}

gdouble
ft9366_process_frame (const guint16 *frame, const guint16 *background,
                      const guint8 *dead, guint8 *image)
{
  const gint w = FT9366_FRAME_WIDTH, h = FT9366_FRAME_HEIGHT, r = FT9366_NORM_RADIUS;
  g_autofree gint *diff = g_new (gint, FT9366_FRAME_PIXELS);
  g_autofree gdouble *mean = g_new (gdouble, FT9366_FRAME_PIXELS);
  g_autofree gdouble *sd = g_new (gdouble, FT9366_FRAME_PIXELS);
  gdouble sd_max = 1.0;
  guint covered = 0;

  gdouble rsum = 0, rsq = 0;
  guint rn = 0;

  /* background - frame: ridges come out dark, valleys light */
  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    diff[i] = (gint) background[i] - (gint) frame[i];

  /* An empty sensor still has noise, which the contrast normalisation below
   * would stretch into texture. Real touches spread the raw difference by
   * well over a hundred counts, empty frames by ~10. */
  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    if (!dead[i])
      {
        rsum += diff[i];
        rsq += (gdouble) diff[i] * diff[i];
        rn++;
      }
  if (rn == 0 || sqrt (rsq / rn - (rsum / rn) * (rsum / rn)) < FT9366_MIN_SIGNAL)
    {
      memset (image, FT9366_PX_NONE, FT9366_FRAME_PIXELS);
      return 0;
    }

  /* fill dead pixels from their live neighbours so they don't skew the
   * normalisation around them; they are masked out again below */
  for (gint y = 0; y < h; y++)
    for (gint x = 0; x < w; x++)
      {
        gint sum = 0, n = 0;

        if (!dead[y * w + x])
          continue;
        for (gint dy = MAX (y - 2, 0); dy <= MIN (y + 2, h - 1); dy++)
          for (gint dx = MAX (x - 2, 0); dx <= MIN (x + 2, w - 1); dx++)
            if (!dead[dy * w + dx])
              {
                sum += diff[dy * w + dx];
                n++;
              }
        diff[y * w + x] = n ? sum / n : 0;
      }

  for (gint y = 0; y < h; y++)
    for (gint x = 0; x < w; x++)
      {
        gint64 sum = 0, sq = 0;
        gint n = 0;
        gdouble m;

        for (gint dy = MAX (y - r, 0); dy <= MIN (y + r, h - 1); dy++)
          for (gint dx = MAX (x - r, 0); dx <= MIN (x + r, w - 1); dx++)
            {
              gint p = diff[dy * w + dx];
              sum += p;
              sq += (gint64) p * p;
              n++;
            }
        m = (gdouble) sum / n;
        mean[y * w + x] = m;
        sd[y * w + x] = sqrt (MAX ((gdouble) sq / n - m * m, 1.0));
        sd_max = MAX (sd_max, sd[y * w + x]);
      }

  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    {
      gdouble v = 128 + (diff[i] - mean[i]) * 64 / sd[i];

      if (sd[i] < sd_max * FT9366_MASK_RATIO || dead[i])
        {
          image[i] = FT9366_PX_NONE;
          continue;
        }
      image[i] = CLAMP (v, 1, 254);
      covered++;
    }

  return (gdouble) covered / FT9366_FRAME_PIXELS;
}

/* ---- matcher ------------------------------------------------------------ */
/*
 * The sensor sees a ~3x4 mm patch: too few minutiae for NBIS, and whole-image
 * correlation is fooled by the periodic ridges of any finger. Instead, local
 * features that survive rotation and partial overlap:
 *
 * 1. scale-space keypoints (difference of Gaussians, Lowe's SIFT) on the
 *    image upsampled 2x, with a low contrast threshold and a permissive edge
 *    test: on ridges, most structure is edge-like
 * 2. a dominant gradient orientation per keypoint and a 4x4x8 gradient
 *    histogram descriptor relative to it
 * 3. probe/enrolled keypoint pairs that pass the nearest/second-nearest
 *    ratio test
 * 4. each pair proposes a rotation (from the orientation difference) and a
 *    translation; the score is the largest set of pairs that agree on one
 *    pose in both position and orientation
 *
 * Parameters follow the published SIFT defaults except the contrast and edge
 * thresholds, which are tuned for this sensor: fingerprint ridges make most
 * structure edge-like, and the small image leaves few strong extrema.
 */

#define SIFT_INTVLS 3
#define SIFT_SIGMA 1.6
#define SIFT_INIT_SIGMA 0.5
#ifndef SIFT_OCTAVES
#define SIFT_OCTAVES 3
#endif
#ifndef SIFT_CONTR_THR
#define SIFT_CONTR_THR 0.02
#endif
#ifndef SIFT_CURV_THR
#define SIFT_CURV_THR 15.0
#endif
#define SIFT_BORDER 3
#define SIFT_MAX_INTERP 5
#define SIFT_ORI_BINS 36
#define SIFT_ORI_SIG 1.5
#define SIFT_ORI_RADIUS (3.0 * SIFT_ORI_SIG)
#define SIFT_ORI_PEAK 0.8
#define SIFT_DESCR_WIDTH 4
#define SIFT_DESCR_BINS 8
#define SIFT_DESCR_SCL 3.0
#define SIFT_DESCR_MAG_THR 0.2
#define DESCR_DIM (SIFT_DESCR_WIDTH * SIFT_DESCR_WIDTH * SIFT_DESCR_BINS)

#ifndef FT9366_MAX_KP
#define FT9366_MAX_KP 256
#endif
#ifndef FT9366_MASK_MARGIN
#define FT9366_MASK_MARGIN 3     /* px of finger a keypoint needs around it */
#endif
#ifndef FT9366_RATIO
#define FT9366_RATIO 0.8         /* nearest / second-nearest descriptor distance */
#endif
#ifndef FT9366_POS_TOL
#define FT9366_POS_TOL 2.5       /* px, pose agreement */
#endif
#ifndef FT9366_ORI_TOL
#define FT9366_ORI_TOL (20 * G_PI / 180)
#endif
#ifndef FT9366_MAX_ROT
#define FT9366_MAX_ROT (60 * G_PI / 180)
#endif

typedef struct
{
  gfloat x, y;     /* sensor pixels */
  gfloat ori;      /* radians */
  gfloat d[DESCR_DIM];
} Ft9366Kp;

struct _Ft9366Features
{
  guint    n;
  Ft9366Kp kp[FT9366_MAX_KP];
  guint8   image[FT9366_FRAME_PIXELS];   /* for the ridge-overlap check */
};

typedef struct
{
  gint    w, h;
  gfloat *p;
} Img;

static Img *
img_new (gint w, gint h)
{
  Img *im = g_new (Img, 1);

  im->w = w;
  im->h = h;
  im->p = g_new0 (gfloat, w * h);
  return im;
}

static void
img_free (Img *im)
{
  if (im)
    g_free (im->p);
  g_free (im);
}

static inline gfloat
px (const Img *im, gint x, gint y)
{
  return im->p[CLAMP (y, 0, im->h - 1) * im->w + CLAMP (x, 0, im->w - 1)];
}

/* separable Gaussian, clamped borders */
static Img *
img_blur (const Img *src, gdouble sigma)
{
  gint r = MAX (1, (gint) ceil (3 * sigma));
  g_autofree gfloat *k = g_new (gfloat, 2 * r + 1);
  Img *tmp = img_new (src->w, src->h), *dst = img_new (src->w, src->h);
  gdouble sum = 0;

  for (gint i = -r; i <= r; i++)
    sum += k[i + r] = exp (-(i * i) / (2 * sigma * sigma));
  for (gint i = 0; i <= 2 * r; i++)
    k[i] /= sum;

  for (gint y = 0; y < src->h; y++)
    for (gint x = 0; x < src->w; x++)
      {
        gfloat v = 0;
        for (gint i = -r; i <= r; i++)
          v += k[i + r] * px (src, x + i, y);
        tmp->p[y * src->w + x] = v;
      }
  for (gint y = 0; y < src->h; y++)
    for (gint x = 0; x < src->w; x++)
      {
        gfloat v = 0;
        for (gint i = -r; i <= r; i++)
          v += k[i + r] * px (tmp, x, y + i);
        dst->p[y * src->w + x] = v;
      }
  img_free (tmp);
  return dst;
}

static Img *
img_half (const Img *src)
{
  Img *dst = img_new (src->w / 2, src->h / 2);

  for (gint y = 0; y < dst->h; y++)
    for (gint x = 0; x < dst->w; x++)
      dst->p[y * dst->w + x] = src->p[2 * y * src->w + 2 * x];
  return dst;
}

static Img *
img_double (const Img *src)
{
  Img *dst = img_new (src->w * 2, src->h * 2);

  for (gint y = 0; y < dst->h; y++)
    for (gint x = 0; x < dst->w; x++)
      {
        gdouble sx = x / 2.0, sy = y / 2.0;
        gint x0 = (gint) sx, y0 = (gint) sy;
        gdouble fx = sx - x0, fy = sy - y0;

        dst->p[y * dst->w + x] = (1 - fy) * ((1 - fx) * px (src, x0, y0) + fx * px (src, x0 + 1, y0)) +
                                 fy * ((1 - fx) * px (src, x0, y0 + 1) + fx * px (src, x0 + 1, y0 + 1));
      }
  return dst;
}

typedef struct
{
  Img *gauss[SIFT_OCTAVES][SIFT_INTVLS + 3];
  Img *dog[SIFT_OCTAVES][SIFT_INTVLS + 2];
} Pyramid;

static void
pyramid_build (Pyramid *pyr, const Img *base)
{
  gdouble k = pow (2.0, 1.0 / SIFT_INTVLS), sig[SIFT_INTVLS + 3];

  sig[0] = SIFT_SIGMA;
  sig[1] = SIFT_SIGMA * sqrt (k * k - 1);
  for (gint i = 2; i < SIFT_INTVLS + 3; i++)
    sig[i] = sig[i - 1] * k;

  for (gint o = 0; o < SIFT_OCTAVES; o++)
    for (gint i = 0; i < SIFT_INTVLS + 3; i++)
      {
        if (o == 0 && i == 0)
          {
            /* the doubled image counts as blurred by 2 * SIFT_INIT_SIGMA */
            gdouble s = sqrt (MAX (SIFT_SIGMA * SIFT_SIGMA - 4 * SIFT_INIT_SIGMA * SIFT_INIT_SIGMA, 0.01));
            pyr->gauss[o][i] = img_blur (base, s);
          }
        else if (i == 0)
          {
            pyr->gauss[o][i] = img_half (pyr->gauss[o - 1][SIFT_INTVLS]);
          }
        else
          {
            pyr->gauss[o][i] = img_blur (pyr->gauss[o][i - 1], sig[i]);
          }
      }

  for (gint o = 0; o < SIFT_OCTAVES; o++)
    for (gint i = 0; i < SIFT_INTVLS + 2; i++)
      {
        const Img *a = pyr->gauss[o][i], *b = pyr->gauss[o][i + 1];
        Img *d = img_new (a->w, a->h);

        for (gint j = 0; j < a->w * a->h; j++)
          d->p[j] = b->p[j] - a->p[j];
        pyr->dog[o][i] = d;
      }
}

static void
pyramid_free (Pyramid *pyr)
{
  for (gint o = 0; o < SIFT_OCTAVES; o++)
    {
      for (gint i = 0; i < SIFT_INTVLS + 3; i++)
        img_free (pyr->gauss[o][i]);
      for (gint i = 0; i < SIFT_INTVLS + 2; i++)
        img_free (pyr->dog[o][i]);
    }
}

#define DOG(o, i, x, y) (pyr->dog[o][i]->p[(y) * pyr->dog[o][i]->w + (x)])

static gboolean
is_extremum (const Pyramid *pyr, gint o, gint i, gint x, gint y)
{
  gfloat v = DOG (o, i, x, y);
  gboolean is_max = TRUE, is_min = TRUE;

  for (gint di = -1; di <= 1; di++)
    for (gint dy = -1; dy <= 1; dy++)
      for (gint dx = -1; dx <= 1; dx++)
        {
          gfloat n;

          if (!di && !dy && !dx)
            continue;
          n = DOG (o, i + di, x + dx, y + dy);
          is_max &= v > n;
          is_min &= v < n;
        }
  return is_max || is_min;
}

/* refine (x, y, i) with a quadratic fit; FALSE if it drifts away or the
 * refined contrast is too low */
static gboolean
interp_extremum (const Pyramid *pyr, gint o, gint *i, gint *x, gint *y,
                 gdouble *xi, gdouble *xr, gdouble *xc)
{
  const gint w = pyr->dog[o][0]->w, h = pyr->dog[o][0]->h;
  gdouble dx, dy, ds, v;
  gint n;

  for (n = 0; n < SIFT_MAX_INTERP; n++)
    {
      gdouble dxx, dyy, dss, dxy, dxs, dys;
      gdouble H[3][3], g[3], det, inv[3][3];

      v = DOG (o, *i, *x, *y);
      dx = (DOG (o, *i, *x + 1, *y) - DOG (o, *i, *x - 1, *y)) / 2;
      dy = (DOG (o, *i, *x, *y + 1) - DOG (o, *i, *x, *y - 1)) / 2;
      ds = (DOG (o, *i + 1, *x, *y) - DOG (o, *i - 1, *x, *y)) / 2;
      dxx = DOG (o, *i, *x + 1, *y) + DOG (o, *i, *x - 1, *y) - 2 * v;
      dyy = DOG (o, *i, *x, *y + 1) + DOG (o, *i, *x, *y - 1) - 2 * v;
      dss = DOG (o, *i + 1, *x, *y) + DOG (o, *i - 1, *x, *y) - 2 * v;
      dxy = (DOG (o, *i, *x + 1, *y + 1) - DOG (o, *i, *x - 1, *y + 1) -
             DOG (o, *i, *x + 1, *y - 1) + DOG (o, *i, *x - 1, *y - 1)) / 4;
      dxs = (DOG (o, *i + 1, *x + 1, *y) - DOG (o, *i + 1, *x - 1, *y) -
             DOG (o, *i - 1, *x + 1, *y) + DOG (o, *i - 1, *x - 1, *y)) / 4;
      dys = (DOG (o, *i + 1, *x, *y + 1) - DOG (o, *i + 1, *x, *y - 1) -
             DOG (o, *i - 1, *x, *y + 1) + DOG (o, *i - 1, *x, *y - 1)) / 4;

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
      g[2] = ds;

      det = H[0][0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1]) -
            H[0][1] * (H[1][0] * H[2][2] - H[1][2] * H[2][0]) +
            H[0][2] * (H[1][0] * H[2][1] - H[1][1] * H[2][0]);
      if (fabs (det) < 1e-12)
        return FALSE;
      inv[0][0] = (H[1][1] * H[2][2] - H[1][2] * H[2][1]) / det;
      inv[0][1] = (H[0][2] * H[2][1] - H[0][1] * H[2][2]) / det;
      inv[0][2] = (H[0][1] * H[1][2] - H[0][2] * H[1][1]) / det;
      inv[1][0] = (H[1][2] * H[2][0] - H[1][0] * H[2][2]) / det;
      inv[1][1] = (H[0][0] * H[2][2] - H[0][2] * H[2][0]) / det;
      inv[1][2] = (H[0][2] * H[1][0] - H[0][0] * H[1][2]) / det;
      inv[2][0] = (H[1][0] * H[2][1] - H[1][1] * H[2][0]) / det;
      inv[2][1] = (H[0][1] * H[2][0] - H[0][0] * H[2][1]) / det;
      inv[2][2] = (H[0][0] * H[1][1] - H[0][1] * H[1][0]) / det;

      *xc = -(inv[0][0] * g[0] + inv[0][1] * g[1] + inv[0][2] * g[2]);
      *xr = -(inv[1][0] * g[0] + inv[1][1] * g[1] + inv[1][2] * g[2]);
      *xi = -(inv[2][0] * g[0] + inv[2][1] * g[1] + inv[2][2] * g[2]);

      if (fabs (*xi) < 0.5 && fabs (*xr) < 0.5 && fabs (*xc) < 0.5)
        break;

      *x += (gint) round (*xc);
      *y += (gint) round (*xr);
      *i += (gint) round (*xi);
      if (*i < 1 || *i > SIFT_INTVLS || *x < SIFT_BORDER || *y < SIFT_BORDER ||
          *x >= w - SIFT_BORDER || *y >= h - SIFT_BORDER)
        return FALSE;
    }
  if (n >= SIFT_MAX_INTERP)
    return FALSE;

  /* contrast at the refined position */
  v = DOG (o, *i, *x, *y) + 0.5 * (dx * *xc + dy * *xr + ds * *xi);
  return fabs (v) >= SIFT_CONTR_THR / SIFT_INTVLS;
}

static gboolean
is_edge_like (const Pyramid *pyr, gint o, gint i, gint x, gint y)
{
  gdouble v = DOG (o, i, x, y);
  gdouble dxx = DOG (o, i, x + 1, y) + DOG (o, i, x - 1, y) - 2 * v;
  gdouble dyy = DOG (o, i, x, y + 1) + DOG (o, i, x, y - 1) - 2 * v;
  gdouble dxy = (DOG (o, i, x + 1, y + 1) - DOG (o, i, x - 1, y + 1) -
                 DOG (o, i, x + 1, y - 1) + DOG (o, i, x - 1, y - 1)) / 4;
  gdouble tr = dxx + dyy, det = dxx * dyy - dxy * dxy;

  return det <= 0 || tr * tr / det >= (SIFT_CURV_THR + 1) * (SIFT_CURV_THR + 1) / SIFT_CURV_THR;
}

static gboolean
grad (const Img *im, gint x, gint y, gdouble *mag, gdouble *ori)
{
  gdouble dx, dy;

  if (x < 1 || y < 1 || x >= im->w - 1 || y >= im->h - 1)
    return FALSE;
  dx = im->p[y * im->w + x + 1] - im->p[y * im->w + x - 1];
  dy = im->p[(y + 1) * im->w + x] - im->p[(y - 1) * im->w + x];
  *mag = sqrt (dx * dx + dy * dy);
  *ori = atan2 (dy, dx);
  return TRUE;
}

/* dominant orientations at (x, y): up to SIFT_ORI_BINS, written to oris */
static gint
orientations (const Img *im, gint x, gint y, gdouble scl, gdouble *oris)
{
  gdouble hist[SIFT_ORI_BINS] = { 0 }, tmp[SIFT_ORI_BINS], sig = SIFT_ORI_SIG * scl, mx = 0;
  gint r = (gint) round (SIFT_ORI_RADIUS * scl), n = 0;

  for (gint dy = -r; dy <= r; dy++)
    for (gint dx = -r; dx <= r; dx++)
      {
        gdouble mag, ori;
        gint bin;

        if (!grad (im, x + dx, y + dy, &mag, &ori))
          continue;
        bin = (gint) round (SIFT_ORI_BINS * (ori + G_PI) / (2 * G_PI));
        hist[bin % SIFT_ORI_BINS] += exp (-(dx * dx + dy * dy) / (2 * sig * sig)) * mag;
      }

  for (gint pass = 0; pass < 2; pass++)
    {
      for (gint b = 0; b < SIFT_ORI_BINS; b++)
        tmp[b] = 0.25 * hist[(b + SIFT_ORI_BINS - 1) % SIFT_ORI_BINS] + 0.5 * hist[b] +
                 0.25 * hist[(b + 1) % SIFT_ORI_BINS];
      memcpy (hist, tmp, sizeof hist);
    }
  for (gint b = 0; b < SIFT_ORI_BINS; b++)
    mx = MAX (mx, hist[b]);

  for (gint b = 0; b < SIFT_ORI_BINS; b++)
    {
      gdouble l = hist[(b + SIFT_ORI_BINS - 1) % SIFT_ORI_BINS], c = hist[b];
      gdouble rr = hist[(b + 1) % SIFT_ORI_BINS];

      if (c > l && c > rr && c >= SIFT_ORI_PEAK * mx)
        {
          gdouble off = 0.5 * (l - rr) / (l - 2 * c + rr);
          oris[n++] = remainder ((b + off) * 2 * G_PI / SIFT_ORI_BINS - G_PI, 2 * G_PI);
        }
    }
  return n;
}

static void
descriptor (const Img *im, gdouble x, gdouble y, gdouble ori, gdouble scl, gfloat *d)
{
  const gint nd = SIFT_DESCR_WIDTH, nb = SIFT_DESCR_BINS;
  gdouble hist[SIFT_DESCR_WIDTH][SIFT_DESCR_WIDTH][SIFT_DESCR_BINS] = { { { 0 } } };
  gdouble cs = cos (ori), sn = sin (ori), hw = SIFT_DESCR_SCL * scl, norm = 0;
  gint r = (gint) (hw * G_SQRT2 * (nd + 1) * 0.5 + 0.5);
  gint ix = (gint) round (x), iy = (gint) round (y);

  for (gint i = -r; i <= r; i++)
    for (gint j = -r; j <= r; j++)
      {
        gdouble c_rot = (j * cs + i * sn) / hw, r_rot = (-j * sn + i * cs) / hw;
        gdouble rbin = r_rot + nd / 2.0 - 0.5, cbin = c_rot + nd / 2.0 - 0.5;
        gdouble mag, go, obin, w;
        gint r0, c0, o0;
        gdouble dr, dc, dob;

        if (rbin <= -1 || rbin >= nd || cbin <= -1 || cbin >= nd)
          continue;
        if (!grad (im, ix + j, iy + i, &mag, &go))
          continue;
        go = fmod (go - ori + 4 * G_PI, 2 * G_PI);
        obin = go * nb / (2 * G_PI);
        w = exp (-(c_rot * c_rot + r_rot * r_rot) / (2 * (0.5 * nd) * (0.5 * nd))) * mag;

        r0 = (gint) floor (rbin);
        c0 = (gint) floor (cbin);
        o0 = (gint) floor (obin);
        dr = rbin - r0;
        dc = cbin - c0;
        dob = obin - o0;
        for (gint a = 0; a <= 1; a++)
          for (gint b = 0; b <= 1; b++)
            for (gint c = 0; c <= 1; c++)
              {
                gint rr = r0 + a, cc = c0 + b, oo = (o0 + c) % nb;

                if (rr < 0 || rr >= nd || cc < 0 || cc >= nd)
                  continue;
                hist[rr][cc][oo] += w * (a ? dr : 1 - dr) * (b ? dc : 1 - dc) * (c ? dob : 1 - dob);
              }
      }

  for (gint k = 0; k < DESCR_DIM; k++)
    norm += ((gdouble *) hist)[k] * ((gdouble *) hist)[k];
  norm = sqrt (norm) + 1e-12;
  for (gint k = 0; k < DESCR_DIM; k++)
    d[k] = MIN (((gdouble *) hist)[k] / norm, SIFT_DESCR_MAG_THR);
  norm = 0;
  for (gint k = 0; k < DESCR_DIM; k++)
    norm += d[k] * d[k];
  norm = sqrt (norm) + 1e-12;
  for (gint k = 0; k < DESCR_DIM; k++)
    d[k] /= norm;
}

Ft9366Features *
ft9366_features_new (const guint8 *image)
{
  const gint w = FT9366_FRAME_WIDTH, h = FT9366_FRAME_HEIGHT, m = FT9366_MASK_MARGIN;
  Ft9366Features *f = g_new0 (Ft9366Features, 1);
  g_autofree guint8 *valid = g_new0 (guint8, FT9366_FRAME_PIXELS);

  memcpy (f->image, image, FT9366_FRAME_PIXELS);
  Img *src = img_new (w, h), *base;
  Pyramid pyramid, *pyr = &pyramid;

  /* untouched pixels become mid-grey; keypoints near them are dropped */
  for (gint i = 0; i < FT9366_FRAME_PIXELS; i++)
    src->p[i] = image[i] == FT9366_PX_NONE ? 0.5 : image[i] / 255.0;
  for (gint y = m; y < h - m; y++)
    for (gint x = m; x < w - m; x++)
      {
        gboolean ok = TRUE;

        for (gint dy = -m; dy <= m && ok; dy++)
          for (gint dx = -m; dx <= m && ok; dx++)
            ok = image[(y + dy) * w + x + dx] != FT9366_PX_NONE;
        valid[y * w + x] = ok;
      }

  base = img_double (src);
  img_free (src);
  pyramid_build (pyr, base);
  img_free (base);

  for (gint o = 0; o < SIFT_OCTAVES && f->n < FT9366_MAX_KP; o++)
    {
      const gint ow = pyr->dog[o][0]->w, oh = pyr->dog[o][0]->h;
      const gdouble to_sensor = pow (2.0, o) / 2;   /* octave px -> sensor px */

      for (gint i = 1; i <= SIFT_INTVLS; i++)
        for (gint y = SIFT_BORDER; y < oh - SIFT_BORDER; y++)
          for (gint x = SIFT_BORDER; x < ow - SIFT_BORDER; x++)
            {
              gint ii = i, xx = x, yy = y, sx, sy, no;
              gdouble xi = 0, xr = 0, xc = 0, scl, oris[SIFT_ORI_BINS];

              if (fabs (DOG (o, i, x, y)) <= 0.5 * SIFT_CONTR_THR / SIFT_INTVLS ||
                  !is_extremum (pyr, o, i, x, y))
                continue;
              if (!interp_extremum (pyr, o, &ii, &xx, &yy, &xi, &xr, &xc) ||
                  is_edge_like (pyr, o, ii, xx, yy))
                continue;

              sx = (gint) round ((xx + xc) * to_sensor);
              sy = (gint) round ((yy + xr) * to_sensor);
              if (sx < 0 || sy < 0 || sx >= w || sy >= h || !valid[sy * w + sx])
                continue;

              scl = SIFT_SIGMA * pow (2.0, (ii + xi) / SIFT_INTVLS);
              no = orientations (pyr->gauss[o][ii], xx, yy, scl, oris);
              for (gint k = 0; k < no && f->n < FT9366_MAX_KP; k++)
                {
                  Ft9366Kp *kp = &f->kp[f->n++];

                  kp->x = (xx + xc) * to_sensor;
                  kp->y = (yy + xr) * to_sensor;
                  kp->ori = oris[k];
                  descriptor (pyr->gauss[o][ii], xx + xc, yy + xr, oris[k], scl, kp->d);
                }
            }
    }

  pyramid_free (pyr);
  return f;
}

void
ft9366_features_free (Ft9366Features *features)
{
  g_free (features);
}

guint
ft9366_features_count (const Ft9366Features *features)
{
  return features->n;
}

/* rigid pose: (x, y) -> (c x - s y + tx, s x + c y + ty), rotation atan2 (s, c) */
typedef struct
{
  gdouble c, s, tx, ty;
} Pose;

static const Pose pose_identity = { 1, 0, 0, 0 };

static void
pose_apply (const Pose *p, gdouble x, gdouble y, gdouble *ox, gdouble *oy)
{
  *ox = p->c * x - p->s * y + p->tx;
  *oy = p->s * x + p->c * y + p->ty;
}

/* a then b */
static Pose
pose_compose (const Pose *a, const Pose *b)
{
  Pose r = { b->c * a->c - b->s * a->s, b->s * a->c + b->c * a->s, 0, 0 };

  pose_apply (b, a->tx, a->ty, &r.tx, &r.ty);
  return r;
}

static Pose
pose_invert (const Pose *p)
{
  Pose r = { p->c, -p->s, 0, 0 };

  r.tx = -(p->c * p->tx + p->s * p->ty);
  r.ty = -(-p->s * p->tx + p->c * p->ty);
  return r;
}

/* a probe keypoint paired with an enrolled one, the latter already mapped
 * into the frame the pose search works in */
typedef struct
{
  const Ft9366Kp *p;
  gdouble         tx, ty, tori;
} Pair;

static gdouble
dist2 (const gfloat *a, const gfloat *b)
{
  gdouble s = 0;

  for (gint k = 0; k < DESCR_DIM; k++)
    s += (a[k] - b[k]) * (a[k] - b[k]);
  return s;
}

/* ratio-test pairs between probe and one enrolled image, with the enrolled
 * keypoints mapped through to_frame; appended to pairs */
static void
collect_pairs (const Ft9366Features *probe, const Ft9366Features *enrolled,
               const Pose *to_frame, GArray *pairs)
{
  gdouble rot = atan2 (to_frame->s, to_frame->c);

  for (guint i = 0; i < probe->n; i++)
    {
      gdouble d1 = G_MAXDOUBLE, d2 = G_MAXDOUBLE;
      const Ft9366Kp *bt = NULL;

      for (guint j = 0; j < enrolled->n; j++)
        {
          gdouble d = dist2 (probe->kp[i].d, enrolled->kp[j].d);

          if (d < d1)
            {
              d2 = d1;
              d1 = d;
              bt = &enrolled->kp[j];
            }
          else if (d < d2)
            {
              d2 = d;
            }
        }
      if (bt && d1 < FT9366_RATIO * FT9366_RATIO * d2)
        {
          Pair pr = { &probe->kp[i], 0, 0, bt->ori + rot };

          pose_apply (to_frame, bt->x, bt->y, &pr.tx, &pr.ty);
          g_array_append_val (pairs, pr);
        }
    }
}

static gboolean
pair_agrees (const Pair *pr, const Pose *pose, gdouble rot)
{
  gdouble x, y;

  pose_apply (pose, pr->p->x, pr->p->y, &x, &y);
  return hypot (x - pr->tx, y - pr->ty) <= FT9366_POS_TOL &&
         fabs (remainder (pr->tori - pr->p->ori - rot, 2 * G_PI)) <= FT9366_ORI_TOL;
}

/* Each pair proposes a pose from its orientation difference; keep the one
 * most pairs agree with, counting every probe keypoint once, then refine it
 * by least squares over its inliers. Returns the inlier count. */
static guint
best_pose (const GArray *pairs, Pose *out)
{
  g_autoptr(GHashTable) seen = g_hash_table_new (NULL, NULL);
  guint best = 0;
  Pose best_pose = pose_identity;
  gdouble best_rot = 0;

  for (guint a = 0; a < pairs->len; a++)
    {
      const Pair *pa = &g_array_index (pairs, Pair, a);
      gdouble rot = remainder (pa->tori - pa->p->ori, 2 * G_PI);
      Pose pose = { cos (rot), sin (rot), 0, 0 };
      guint inliers = 0;

      if (fabs (rot) > FT9366_MAX_ROT)
        continue;
      pose.tx = pa->tx - (pose.c * pa->p->x - pose.s * pa->p->y);
      pose.ty = pa->ty - (pose.s * pa->p->x + pose.c * pa->p->y);

      g_hash_table_remove_all (seen);
      for (guint k = 0; k < pairs->len; k++)
        {
          const Pair *pr = &g_array_index (pairs, Pair, k);

          if (!g_hash_table_contains (seen, pr->p) && pair_agrees (pr, &pose, rot))
            {
              g_hash_table_add (seen, (gpointer) pr->p);
              inliers++;
            }
        }
      if (inliers > best)
        {
          best = inliers;
          best_pose = pose;
          best_rot = rot;
        }
    }

  if (best >= 2)
    {
      /* least-squares rigid fit (2D Procrustes) over the inliers */
      gdouble mpx = 0, mpy = 0, mtx = 0, mty = 0, sxx = 0, sxy = 0;
      guint n = 0;

      for (guint k = 0; k < pairs->len; k++)
        {
          const Pair *pr = &g_array_index (pairs, Pair, k);

          if (!pair_agrees (pr, &best_pose, best_rot))
            continue;
          mpx += pr->p->x;
          mpy += pr->p->y;
          mtx += pr->tx;
          mty += pr->ty;
          n++;
        }
      mpx /= n;
      mpy /= n;
      mtx /= n;
      mty /= n;
      for (guint k = 0; k < pairs->len; k++)
        {
          const Pair *pr = &g_array_index (pairs, Pair, k);
          gdouble ppx, py, tx, ty;

          if (!pair_agrees (pr, &best_pose, best_rot))
            continue;
          ppx = pr->p->x - mpx;
          py = pr->p->y - mpy;
          tx = pr->tx - mtx;
          ty = pr->ty - mty;
          sxx += ppx * tx + py * ty;
          sxy += ppx * ty - py * tx;
        }
      best_rot = atan2 (sxy, sxx);
      best_pose.c = cos (best_rot);
      best_pose.s = sin (best_rot);
      best_pose.tx = mtx - (best_pose.c * mpx - best_pose.s * mpy);
      best_pose.ty = mty - (best_pose.s * mpx + best_pose.c * mpy);
    }

  if (out)
    *out = best_pose;
  return best;
}

static guint
match_pose (const Ft9366Features *probe, const Ft9366Features *enrolled, Pose *pose)
{
  g_autoptr(GArray) pairs = g_array_new (FALSE, FALSE, sizeof (Pair));

  collect_pairs (probe, enrolled, &pose_identity, pairs);
  return best_pose (pairs, pose);
}

guint
ft9366_match (const Ft9366Features *probe, const Ft9366Features *enrolled)
{
  return match_pose (probe, enrolled, NULL);
}

/* ---- multi-image template ----------------------------------------------- */
/*
 * Each touch sees a different 3x4 mm part of the finger, so a new touch often
 * overlaps no single enrolled image enough to match it. The enrolled images
 * are therefore registered to each other: every pair is matched, pairs with
 * a confident match become edges, and a maximum spanning tree over those
 * edges chains every image's pose into one shared frame per connected group.
 * At verify time the probe's pairs with all images of a group are pooled in
 * that frame, so a touch straddling several enrolled images collects
 * evidence from all of them under a single pose.
 */

#define FT9366_LINK_MIN 10   /* inliers for two enrolled images to be registered */

struct _Ft9366Template
{
  guint            n;
  Ft9366Features **features;
  Pose            *to_frame;   /* image -> its group's frame */
  guint           *group;
};

Ft9366Template *
ft9366_template_new (const guint8 * const *images, guint n)
{
  Ft9366Template *tpl = g_new0 (Ft9366Template, 1);
  g_autofree guint *links = g_new0 (guint, n * n);
  g_autofree Pose *poses = g_new (Pose, n * n);   /* poses[i * n + j]: i -> j */
  g_autofree gboolean *placed = g_new0 (gboolean, n);
  guint groups = 0;

  tpl->n = n;
  tpl->features = g_new (Ft9366Features *, n);
  tpl->to_frame = g_new (Pose, n);
  tpl->group = g_new (guint, n);
  for (guint i = 0; i < n; i++)
    tpl->features[i] = ft9366_features_new (images[i]);

  for (guint i = 0; i < n; i++)
    for (guint j = i + 1; j < n; j++)
      {
        Pose p;
        guint inl = match_pose (tpl->features[i], tpl->features[j], &p);

        links[i * n + j] = links[j * n + i] = inl >= FT9366_LINK_MIN ? inl : 0;
        poses[i * n + j] = p;
        poses[j * n + i] = pose_invert (&p);
      }

  /* Prim's maximum spanning tree, one tree per connected group, rooted at
   * the best-connected image left */
  for (;;)
    {
      guint root = n, root_w = 0;

      for (guint i = 0; i < n; i++)
        {
          guint w = 0;

          if (placed[i])
            continue;
          for (guint j = 0; j < n; j++)
            w += links[i * n + j];
          if (root == n || w > root_w)
            {
              root = i;
              root_w = w;
            }
        }
      if (root == n)
        break;

      placed[root] = TRUE;
      tpl->group[root] = groups;
      tpl->to_frame[root] = pose_identity;
      for (;;)
        {
          guint bu = n, bv = n, bw = 0;

          for (guint u = 0; u < n; u++)
            if (placed[u] && tpl->group[u] == groups)
              {
                for (guint v = 0; v < n; v++)
                  if (!placed[v] && links[u * n + v] > bw)
                    {
                      bw = links[u * n + v];
                      bu = u;
                      bv = v;
                    }
              }
          if (bv == n)
            break;
          placed[bv] = TRUE;
          tpl->group[bv] = groups;
          tpl->to_frame[bv] = pose_compose (&poses[bv * n + bu], &tpl->to_frame[bu]);
        }
      groups++;
    }

  return tpl;
}

void
ft9366_template_free (Ft9366Template *tpl)
{
  if (!tpl)
    return;
  for (guint i = 0; i < tpl->n; i++)
    ft9366_features_free (tpl->features[i]);
  g_free (tpl->features);
  g_free (tpl->to_frame);
  g_free (tpl->group);
  g_free (tpl);
}

guint
ft9366_template_groups (const Ft9366Template *tpl)
{
  guint g = 0;

  for (guint i = 0; i < tpl->n; i++)
    g = MAX (g, tpl->group[i] + 1);
  return g;
}

/* bilinear sample of a processed image; FALSE outside or next to untouched pixels */
static gboolean
sample (const guint8 *img, gdouble x, gdouble y, gdouble *v)
{
  const gint w = FT9366_FRAME_WIDTH;
  gint x0 = (gint) floor (x), y0 = (gint) floor (y);
  gdouble fx = x - x0, fy = y - y0;
  guint8 a, b, c, d;

  if (x0 < 0 || y0 < 0 || x0 >= w - 1 || y0 >= FT9366_FRAME_HEIGHT - 1)
    return FALSE;
  a = img[y0 * w + x0];
  b = img[y0 * w + x0 + 1];
  c = img[(y0 + 1) * w + x0];
  d = img[(y0 + 1) * w + x0 + 1];
  if (a == FT9366_PX_NONE || b == FT9366_PX_NONE || c == FT9366_PX_NONE || d == FT9366_PX_NONE)
    return FALSE;
  *v = (1 - fy) * ((1 - fx) * a + fx * b) + fy * ((1 - fx) * c + fx * d);
  return TRUE;
}

/*
 * Ridge-overlap check: with the probe placed by the keypoint pose, compare
 * pixel values against every enrolled image of the group that covers them.
 * A true alignment lines ridges up (high correlation); a chance alignment of
 * another finger's keypoints does not. overlap is the share of the probe's
 * finger area that the group covers.
 */
static void
overlap_check (const Ft9366Template *tpl, guint g, const Pose *probe_to_frame,
               const guint8 *probe, gdouble *ncc, gdouble *overlap)
{
  const gint w = FT9366_FRAME_WIDTH, h = FT9366_FRAME_HEIGHT;
  g_autofree Pose *from_frame = g_new (Pose, tpl->n);
  gdouble sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
  guint n = 0, area = 0, covered = 0;

  for (guint i = 0; i < tpl->n; i++)
    from_frame[i] = pose_invert (&tpl->to_frame[i]);

  for (gint y = 0; y < h; y++)
    for (gint x = 0; x < w; x++)
      {
        gdouble fx, fy, p = probe[y * w + x];
        gboolean hit = FALSE;

        if (probe[y * w + x] == FT9366_PX_NONE)
          continue;
        area++;
        pose_apply (probe_to_frame, x, y, &fx, &fy);
        for (guint i = 0; i < tpl->n; i++)
          {
            gdouble ix, iy, v;

            if (tpl->group[i] != g)
              continue;
            pose_apply (&from_frame[i], fx, fy, &ix, &iy);
            if (!sample (tpl->features[i]->image, ix, iy, &v))
              continue;
            sx += p;
            sy += v;
            sxx += p * p;
            syy += v * v;
            sxy += p * v;
            n++;
            hit = TRUE;
          }
        covered += hit;
      }

  *overlap = area ? (gdouble) covered / area : 0;
  *ncc = 0;
  if (n >= 100)
    {
      gdouble vx = sxx - sx * sx / n, vy = syy - sy * sy / n;

      if (vx > 0 && vy > 0)
        *ncc = (sxy - sx * sy / n) / sqrt (vx * vy);
    }
}

guint
ft9366_template_match_info (const Ft9366Template *tpl, const Ft9366Features *probe,
                            Ft9366MatchInfo *info)
{
  guint best = 0, best_group = 0, groups = ft9366_template_groups (tpl);
  Pose best_p = pose_identity;
  Ft9366MatchInfo mi = { 0 };

  for (guint g = 0; g < groups; g++)
    {
      g_autoptr(GArray) pairs = g_array_new (FALSE, FALSE, sizeof (Pair));
      Pose pose;
      guint inl;

      for (guint i = 0; i < tpl->n; i++)
        if (tpl->group[i] == g)
          collect_pairs (probe, tpl->features[i], &tpl->to_frame[i], pairs);
      inl = best_pose (pairs, &pose);
      if (inl > best)
        {
          best = inl;
          best_group = g;
          best_p = pose;
        }
    }

  mi.inliers = best;
  if (best >= 2)
    overlap_check (tpl, best_group, &best_p, probe->image, &mi.ncc, &mi.overlap);
  if (info)
    *info = mi;
  return best;
}

guint
ft9366_template_match (const Ft9366Template *tpl, const Ft9366Features *probe)
{
  return ft9366_template_match_info (tpl, probe, NULL);
}

gboolean
ft9366_match_accept (const Ft9366MatchInfo *info)
{
  return (info->inliers >= FT9366_MIN_INLIERS && info->ncc >= FT9366_MIN_NCC) ||
         info->inliers >= FT9366_SURE_INLIERS;
}

#define FT9366_BG_EDGE 2
#define FT9366_BG_SATURATED 4090
#define FT9366_BG_ZERO 10
#define FT9366_BG_TARGET 1900
#define FT9366_BG_MIN 1000
#define FT9366_BG_MAX 3000
#define FT9366_BG_RAIL_SHARE 0.02
#define FT9366_DAC_COUNTS_PER_STEP 615
#define FT9366_DAC_MAX_STEP 10
#define FT9366_DAC_RAIL_STEP 5

void
ft9366_background_stats (const guint16 *background, Ft9366BgStats *stats)
{
  guint n = 0, saturated = 0, zero = 0;
  gdouble sum = 0;

  for (gint y = 0; y < FT9366_FRAME_HEIGHT; y++)
    for (gint x = FT9366_BG_EDGE; x < FT9366_FRAME_WIDTH - FT9366_BG_EDGE; x++)
      {
        guint16 v = background[y * FT9366_FRAME_WIDTH + x];

        sum += v;
        n++;
        if (v >= FT9366_BG_SATURATED)
          saturated++;
        if (v <= FT9366_BG_ZERO)
          zero++;
      }
  stats->mean = sum / n;
  stats->saturated = (gdouble) saturated / n;
  stats->zero = (gdouble) zero / n;
}

gboolean
ft9366_background_ok (const Ft9366BgStats *stats)
{
  return stats->mean >= FT9366_BG_MIN && stats->mean <= FT9366_BG_MAX &&
         stats->saturated <= FT9366_BG_RAIL_SHARE && stats->zero <= FT9366_BG_RAIL_SHARE;
}

guint
ft9366_next_dac (guint dac, const Ft9366BgStats *stats)
{
  gint step;

  if (ft9366_background_ok (stats))
    return dac;

  if (stats->zero > FT9366_BG_RAIL_SHARE && stats->mean < FT9366_BG_MIN)
    {
      step = FT9366_DAC_RAIL_STEP;
    }
  else if (stats->saturated > FT9366_BG_RAIL_SHARE && stats->mean > FT9366_BG_MAX)
    {
      step = -FT9366_DAC_RAIL_STEP;
    }
  else
    {
      step = (gint) round ((FT9366_BG_TARGET - stats->mean) / FT9366_DAC_COUNTS_PER_STEP);
      step = CLAMP (step, -FT9366_DAC_MAX_STEP, FT9366_DAC_MAX_STEP);
      if (step == 0)
        step = stats->mean < FT9366_BG_TARGET ? 1 : -1;
    }

  return CLAMP ((gint) dac + step, FT9366_DAC_MIN, FT9366_DAC_MAX);
}
