/*
 * FocalTech FT9366 matcher regression tests: synthetic data only, no hardware
 * and no real fingerprints. They guard the properties a fingerprint login
 * depends on: an empty or flat frame never matches, noise never matches, and
 * the same ridges do match.
 */

#include <glib.h>
#include <math.h>
#include <string.h>

#include "ft9366-match.h"

#define W FT9366_FRAME_WIDTH
#define H FT9366_FRAME_HEIGHT
#define ENROLLED 6

/* A ridge pattern whose orientation and spacing drift across the image, like
 * a fingertip. seed picks the finger, (sx, sy) shifts it across the sensor. */
static void
make_print (guint8 *image, guint seed, gint sx, gint sy)
{
  GRand *rng = g_rand_new_with_seed (seed);
  gdouble a = g_rand_double_range (rng, 0.0, G_PI);
  gdouble b = g_rand_double_range (rng, 0.6, 1.4);
  gdouble c = g_rand_double_range (rng, 0.04, 0.09);
  gdouble d = g_rand_double_range (rng, 0.04, 0.09);
  gdouble period = g_rand_double_range (rng, 7.0, 9.0);

  for (gint y = 0; y < H; y++)
    for (gint x = 0; x < W; x++)
      {
        gdouble fx = x + sx, fy = y + sy;
        gdouble theta = a + b * sin (c * fx) * cos (d * fy);
        gdouble phase = 2 * G_PI * (cos (theta) * fx + sin (theta) * fy) / period;
        gdouble v = 128 + 95 * sin (phase);

        image[y * W + x] = CLAMP ((gint) v, 1, 254);
      }
  g_rand_free (rng);
}

static Ft9366Template *
enroll (guint seed)
{
  guint8 images[ENROLLED][FT9366_FRAME_PIXELS];
  const guint8 *ptrs[ENROLLED];

  for (guint i = 0; i < ENROLLED; i++)
    {
      make_print (images[i], seed, (gint) i * 2 - 4, (gint) i - 2);
      ptrs[i] = images[i];
    }
  return ft9366_template_new (ptrs, ENROLLED);
}

static gboolean
accepted (const Ft9366Template *tpl, const guint8 *image)
{
  g_autoptr(Ft9366Features) probe = ft9366_features_new (image);
  Ft9366MatchInfo info = { 0 };

  ft9366_template_match_info (tpl, probe, &info);
  g_test_message ("inliers %u, ncc %.2f, overlap %.2f", info.inliers, info.ncc, info.overlap);
  return ft9366_match_accept (&info);
}

static void
test_same_finger_matches (void)
{
  g_autoptr(Ft9366Template) tpl = enroll (1);
  guint8 probe[FT9366_FRAME_PIXELS];

  make_print (probe, 1, 1, 0);
  g_assert_true (accepted (tpl, probe));
}

static void
test_other_finger_rejected (void)
{
  g_autoptr(Ft9366Template) tpl = enroll (1);
  guint8 probe[FT9366_FRAME_PIXELS];

  for (guint seed = 2; seed < 12; seed++)
    {
      make_print (probe, seed, 0, 0);
      g_assert_false (accepted (tpl, probe));
    }
}

static void
test_untouched_image_never_matches (void)
{
  g_autoptr(Ft9366Template) tpl = enroll (1);
  guint8 probe[FT9366_FRAME_PIXELS];

  memset (probe, FT9366_PX_NONE, sizeof probe);
  g_assert_false (accepted (tpl, probe));
}

static void
test_noise_never_matches (void)
{
  g_autoptr(Ft9366Template) tpl = enroll (1);
  GRand *rng = g_rand_new_with_seed (99);
  guint8 probe[FT9366_FRAME_PIXELS];

  for (guint run = 0; run < 10; run++)
    {
      for (guint i = 0; i < sizeof probe; i++)
        probe[i] = g_rand_int_range (rng, 1, 255);
      g_assert_false (accepted (tpl, probe));
    }
  g_rand_free (rng);
}

/* An empty sensor still has noise: the processing must not stretch it into
 * texture the matcher could take for a finger. */
static void
test_flat_frame_has_no_coverage (void)
{
  guint16 background[FT9366_FRAME_PIXELS], frame[FT9366_FRAME_PIXELS];
  guint8 dead[FT9366_FRAME_PIXELS], image[FT9366_FRAME_PIXELS];
  GRand *rng = g_rand_new_with_seed (7);

  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    {
      background[i] = 2000 + g_rand_int_range (rng, 0, 6);
      frame[i] = 2000 + g_rand_int_range (rng, 0, 6);
    }
  ft9366_dead_pixels (background, dead);

  g_assert_cmpfloat (ft9366_process_frame (frame, background, dead, image), ==, 0.0);
  for (guint i = 0; i < sizeof image; i++)
    g_assert_cmpuint (image[i], ==, FT9366_PX_NONE);
  g_rand_free (rng);
}

/* A frame identical to its background, the case a chip stuck in the wrong
 * mode produces, must not enrol or match anything. */
static void
test_frame_equal_to_background_has_no_coverage (void)
{
  guint16 background[FT9366_FRAME_PIXELS];
  guint8 dead[FT9366_FRAME_PIXELS], image[FT9366_FRAME_PIXELS];

  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    background[i] = 1500 + (i * 37) % 400;
  ft9366_dead_pixels (background, dead);

  g_assert_cmpfloat (ft9366_process_frame (background, background, dead, image), ==, 0.0);
}

/* The raw path: a touch darkens the background where ridges are. */
static void
test_raw_touch_covers (void)
{
  guint16 background[FT9366_FRAME_PIXELS], frame[FT9366_FRAME_PIXELS];
  guint8 dead[FT9366_FRAME_PIXELS], image[FT9366_FRAME_PIXELS], ridges[FT9366_FRAME_PIXELS];

  make_print (ridges, 1, 1, 0);
  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    {
      background[i] = 2500;
      frame[i] = 2500 - (ridges[i] - 28);
    }
  ft9366_dead_pixels (background, dead);

  g_assert_cmpfloat (ft9366_process_frame (frame, background, dead, image), >, 0.4);
}

#define COUNTS_PER_DAC_STEP 615.0
#define MAX_ADJUSTMENTS 4

static Ft9366BgStats
simulated_background (guint dac, gint zero_point)
{
  guint16 background[FT9366_FRAME_PIXELS];
  guint16 level = CLAMP ((gint) (COUNTS_PER_DAC_STEP * ((gint) dac - zero_point)), 0, 4092);
  Ft9366BgStats stats;

  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    background[i] = level;
  ft9366_background_stats (background, &stats);
  return stats;
}

static guint
calibrate (gint zero_point, guint dac, guint *adjustments)
{
  *adjustments = 0;
  for (;;)
    {
      Ft9366BgStats stats = simulated_background (dac, zero_point);
      guint next;

      if (ft9366_background_ok (&stats) || *adjustments >= MAX_ADJUSTMENTS)
        return dac;
      next = ft9366_next_dac (dac, &stats);
      if (next == dac)
        return dac;
      dac = next;
      (*adjustments)++;
    }
}

static void
test_background_stats (void)
{
  Ft9366BgStats stats;

  stats = simulated_background (0x20, 0x33);
  g_assert_cmpfloat (stats.mean, ==, 0.0);
  g_assert_cmpfloat (stats.zero, ==, 1.0);

  stats = simulated_background (0x50, 0x33);
  g_assert_cmpfloat (stats.saturated, ==, 1.0);
  g_assert_cmpfloat (stats.zero, ==, 0.0);

  stats = simulated_background (0x36, 0x33);
  g_assert_cmpfloat (stats.saturated, ==, 0.0);
  g_assert_cmpfloat (stats.zero, ==, 0.0);
  g_assert_true (ft9366_background_ok (&stats));
}

static void
test_calibration_leaves_a_healthy_sensor_alone (void)
{
  guint adjustments;

  for (guint dac = 0x35; dac <= 0x37; dac++)
    {
      g_assert_cmpuint (calibrate (0x33, dac, &adjustments), ==, dac);
      g_assert_cmpuint (adjustments, ==, 0);
    }
}

static void
test_calibration_converges_on_other_units (void)
{
  for (gint zero_point = 0x2c; zero_point <= 0x44; zero_point++)
    {
      guint adjustments;
      guint dac = calibrate (zero_point, 0x36, &adjustments);
      Ft9366BgStats stats = simulated_background (dac, zero_point);

      g_test_message ("zero point 0x%02x: DAC 0x%02x after %u adjustments", zero_point, dac, adjustments);
      g_assert_cmpfloat (stats.mean, >=, 1000.0);
      g_assert_cmpfloat (stats.mean, <=, 3000.0);
      g_assert_cmpfloat (stats.saturated, ==, 0.0);
      g_assert_cmpfloat (stats.zero, ==, 0.0);
      g_assert_cmpuint (adjustments, <=, MAX_ADJUSTMENTS);
      g_assert_cmpuint (dac, >=, FT9366_DAC_MIN);
      g_assert_cmpuint (dac, <=, FT9366_DAC_MAX);
    }
}

static void
test_calibration_stays_in_bounds (void)
{
  Ft9366BgStats stats = simulated_background (FT9366_DAC_MAX, 0x7f);

  g_assert_cmpuint (ft9366_next_dac (FT9366_DAC_MAX, &stats), ==, FT9366_DAC_MAX);

  stats = simulated_background (FT9366_DAC_MIN, 0);
  g_assert_cmpuint (ft9366_next_dac (FT9366_DAC_MIN, &stats), ==, FT9366_DAC_MIN);
}

int
main (int argc, char *argv[])
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/ft9366-match/same-finger-matches", test_same_finger_matches);
  g_test_add_func ("/ft9366-match/other-finger-rejected", test_other_finger_rejected);
  g_test_add_func ("/ft9366-match/untouched-image-never-matches", test_untouched_image_never_matches);
  g_test_add_func ("/ft9366-match/noise-never-matches", test_noise_never_matches);
  g_test_add_func ("/ft9366-match/flat-frame-has-no-coverage", test_flat_frame_has_no_coverage);
  g_test_add_func ("/ft9366-match/frame-equal-to-background-has-no-coverage",
                   test_frame_equal_to_background_has_no_coverage);
  g_test_add_func ("/ft9366-match/raw-touch-covers", test_raw_touch_covers);
  g_test_add_func ("/ft9366-match/background-stats", test_background_stats);
  g_test_add_func ("/ft9366-match/calibration-leaves-healthy-sensor-alone",
                   test_calibration_leaves_a_healthy_sensor_alone);
  g_test_add_func ("/ft9366-match/calibration-converges-on-other-units",
                   test_calibration_converges_on_other_units);
  g_test_add_func ("/ft9366-match/calibration-stays-in-bounds", test_calibration_stays_in_bounds);

  return g_test_run ();
}
