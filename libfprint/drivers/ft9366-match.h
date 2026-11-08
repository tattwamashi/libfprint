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

#pragma once

#include <glib.h>

/*
 * Hardware-independent half of the driver: turning a raw frame into an
 * image, and deciding whether two images come from the same finger. Kept
 * free of libfprint/USB types so it can also be built into offline tools
 * that evaluate the matcher on recorded captures.
 */

#define FT9366_FRAME_WIDTH 64
#define FT9366_FRAME_HEIGHT 80
#define FT9366_FRAME_PIXELS (FT9366_FRAME_WIDTH * FT9366_FRAME_HEIGHT)

#define FT9366_PX_NONE 255       /* image pixel not touched by the finger */

/* dead pixels (and the unused edge columns) read ~0 on an empty sensor */
void ft9366_dead_pixels (const guint16 *background,
                         guint8        *dead);

/* background - frame, dead pixel fill, local contrast normalisation into
 * image (1..254, FT9366_PX_NONE where untouched). Returns the share of the
 * sensor the finger covers. */
gdouble ft9366_process_frame (const guint16 *frame,
                              const guint16 *background,
                              const guint8  *dead,
                              guint8        *image);

typedef struct _Ft9366Features Ft9366Features;

Ft9366Features *ft9366_features_new (const guint8 *image);
void            ft9366_features_free (Ft9366Features *features);
guint           ft9366_features_count (const Ft9366Features *features);

G_DEFINE_AUTOPTR_CLEANUP_FUNC (Ft9366Features, ft9366_features_free)

/* similarity of a probe to one enrolled image; higher is more similar */
guint ft9366_match (const Ft9366Features *probe,
                    const Ft9366Features *enrolled);

/* enrolled images registered into shared frames (see ft9366-match.c) */
typedef struct _Ft9366Template Ft9366Template;

Ft9366Template *ft9366_template_new (const guint8 * const *images,
                                     guint                 n);
void            ft9366_template_free (Ft9366Template *tpl);
guint           ft9366_template_groups (const Ft9366Template *tpl);
guint           ft9366_template_match (const Ft9366Template *tpl,
                                       const Ft9366Features *probe);

typedef struct
{
  guint   inliers;   /* keypoint pairs agreeing on the best pose */
  gdouble ncc;       /* ridge correlation over the aligned overlap, -1..1 */
  gdouble overlap;   /* share of the probe's finger area the template covers */
} Ft9366MatchInfo;

guint           ft9366_template_match_info (const Ft9366Template *tpl,
                                            const Ft9366Features *probe,
                                            Ft9366MatchInfo      *info);

G_DEFINE_AUTOPTR_CLEANUP_FUNC (Ft9366Template, ft9366_template_free)

/* Same finger when the keypoint pose is backed by the ridges themselves,
 * or when so many keypoints agree that chance is out of the question
 * (on recorded test sets, other fingers reached at most 6 inliers and 0.56
 * correlation). */
#define FT9366_MIN_INLIERS 4
#define FT9366_MIN_NCC 0.65
#define FT9366_SURE_INLIERS 12

gboolean ft9366_match_accept (const Ft9366MatchInfo *info);

#define FT9366_DAC_MIN 0x28
#define FT9366_DAC_MAX 0x58

typedef struct
{
  gdouble mean;
  gdouble saturated;
  gdouble zero;
} Ft9366BgStats;

void     ft9366_background_stats (const guint16 *background,
                                  Ft9366BgStats *stats);
gboolean ft9366_background_ok (const Ft9366BgStats *stats);
guint    ft9366_next_dac (guint                dac,
                          const Ft9366BgStats *stats);
