/**
 * @file stability_test.c
 * @brief Host unit tests for the hold-stability score (services/stability).
 *
 * The unmodified firmware source is compiled with a desktop GCC against the
 * `stubs/esp_log.h` stand-in and fed 50 Hz frames. Expected values are
 * derived independently of the code under test: from the documented formulas
 * (hand calculation) and from the filter response computed with SciPy
 * (see tools/analysis.py), never copied from the module's own output.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "host_log.h"
#include "sim_signal.h"
#include "stability/stability.h"
#include "test_util.h"

#define FRAME_HZ 50 /**< rate of the decimated stream the score runs on */

static const float ZERO3[3] = {0.0f, 0.0f, 0.0f};
static const float Q_IDENTITY[4] = {0.0f, 0.0f, 0.0f, 1.0f};

static void reset(void) {
  host_log_reset();
  stability_init();
}

/** Unit quaternion [i,j,k,w] for a rotation of @p angle rad about x. */
static void quat_about_x(double angle, float q[4]) {
  q[0] = (float)sin(angle / 2.0);
  q[1] = 0.0f;
  q[2] = 0.0f;
  q[3] = (float)cos(angle / 2.0);
}

/** Feed @p n identical frames; return the output of the last one. */
static stability_output_t feed(int n, const float gyro[3],
                               const float accel[3], const float quat[4]) {
  stability_output_t out;
  memset(&out, 0, sizeof(out));
  for (int i = 0; i < n; i++) {
    stability_tick(gyro, accel, quat, &out);
  }
  return out;
}

/**
 * @brief Feed @p seconds of a sway signal |a| = offset + amp*sin(2*pi*f*t)
 *        and return the sway RMS averaged over the last @p avg_s seconds.
 */
static double sway_rms(double seconds, double avg_s, double offset, double amp,
                       double freq_hz) {
  int n = (int)(seconds * FRAME_HZ);
  int n_avg = (int)(avg_s * FRAME_HZ);
  double acc = 0.0;
  for (int i = 0; i < n; i++) {
    double t = (double)i / FRAME_HZ;
    float a[3] = {(float)(offset + amp * sin(2.0 * SIM_PI * freq_hz * t)), 0.0f,
                  0.0f};
    stability_output_t out;
    stability_tick(ZERO3, a, Q_IDENTITY, &out);
    if (i >= n - n_avg) {
      acc += (double)out.rms_accel_lf;
    }
  }
  return acc / n_avg;
}

/* ---- configuration --------------------------------------------------- */

static void test_defaults(void) {
  reset();
  stability_config_t c;
  memset(&c, 0xA5, sizeof(c));
  stability_get_config(&c);
  CHECK_NEAR(c.w_omega, 0.45, 1e-6);
  CHECK_NEAR(c.w_accel_lf, 0.10, 1e-6);
  CHECK_NEAR(c.w_disp, 0.45, 1e-6);
  CHECK_NEAR(c.w_omega + c.w_accel_lf + c.w_disp, 1.0, 1e-6);
  CHECK_NEAR(c.map_k, log(2.0), 1e-4); /* I = 1 halves the score */
  CHECK_NEAR(c.baseline_omega, 0.05, 1e-6);
  CHECK_NEAR(c.baseline_accel_lf, 0.20, 1e-6);
  CHECK_NEAR(c.baseline_disp, 0.10, 1e-6);
  CHECK_NEAR(c.alpha_omega, 0.20, 1e-6);
  CHECK_NEAR(c.alpha_accel_lf, 0.10, 1e-6);
  CHECK_NEAR(c.alpha_disp, 0.10, 1e-6);
  CHECK_NEAR(c.alpha_qref, 0.04, 1e-6);
  CHECK_NEAR(c.smoothing_alpha, 0.25, 1e-6);
  CHECK(host_log_find_last("ready") != NULL);
}

static void test_null_arguments_are_safe(void) {
  reset();
  stability_config_t before;
  stability_get_config(&before);
  stability_get_config(NULL);
  stability_set_config(NULL);
  stability_config_t after;
  stability_get_config(&after);
  CHECK(memcmp(&before, &after, sizeof(before)) == 0);

  /* The output pointer is optional. */
  stability_tick(ZERO3, ZERO3, Q_IDENTITY, NULL);
  stability_output_t out = feed(1, ZERO3, ZERO3, Q_IDENTITY);
  CHECK_NEAR(out.score, 100.0, 1e-4);
}

static void test_set_config_replaces_all_fields(void) {
  reset();
  stability_config_t custom = {0.5f,  0.2f,  0.3f,  1.0f,  0.08f, 0.30f,
                               0.15f, 0.30f, 0.20f, 0.15f, 0.05f, 0.50f};
  stability_set_config(&custom);
  stability_config_t c;
  stability_get_config(&c);
  CHECK(memcmp(&custom, &c, sizeof(c)) == 0);
  CHECK(host_log_find_last("config:") != NULL);
}

/* ---- score mapping --------------------------------------------------- */

static void test_still_device_scores_100(void) {
  reset();
  stability_output_t out = feed(5 * FRAME_HZ, ZERO3, ZERO3, Q_IDENTITY);
  CHECK_NEAR(out.score, 100.0, 1e-4);
  CHECK_NEAR(out.instability, 0.0, 1e-6);
  CHECK_NEAR(out.rms_omega, 0.0, 1e-6);
  CHECK_NEAR(out.rms_accel_lf, 0.0, 1e-6);
  CHECK_NEAR(out.rms_disp, 0.0, 1e-6);
}

static void test_hand_calculated_second_frame(void) {
  /* Frame 1: everything is zero, so every estimator is seeded with 0 and
   * the score with 100. Frame 2: |w| = 0.05 rad/s.
   *   mean_sq = 0 + 0.20 * (0.05^2 - 0)        = 0.0005
   *   RMS_w   = sqrt(0.0005)                   = 0.0223607 rad/s
   *   I       = 0.45 * 0.0223607 / 0.05        = 0.201246
   *   raw     = 100 * exp(-0.6931 * 0.201246)  = 86.9807
   *   score   = 100 + 0.25 * (86.9807 - 100)   = 96.7452              */
  reset();
  const float gyro[3] = {0.05f, 0.0f, 0.0f};
  feed(1, ZERO3, ZERO3, Q_IDENTITY);
  stability_output_t out = feed(1, gyro, ZERO3, Q_IDENTITY);
  CHECK_NEAR(out.rms_omega, 0.0223607, 1e-6);
  CHECK_NEAR(out.instability, 0.201246, 1e-5);
  CHECK_NEAR(out.score, 96.7452, 1e-3);
}

static void test_baseline_tremor_steady_state(void) {
  /* A constant |w| equal to the baseline: RMS_w -> 0.05, I -> 0.45,
   * score -> 100 * exp(-0.6931 * 0.45) = 73.206. */
  reset();
  const float gyro[3] = {0.03f, 0.04f, 0.0f}; /* |w| = 0.05 rad/s */
  stability_output_t out = feed(4 * FRAME_HZ, gyro, ZERO3, Q_IDENTITY);
  CHECK_NEAR(out.rms_omega, 0.05, 1e-5);
  CHECK_NEAR(out.instability, 0.45, 1e-4);
  CHECK_NEAR(out.score, 73.206, 0.01);
}

static void test_unit_instability_halves_score(void) {
  /* With the whole weight on the rate term and the input at the baseline,
   * I = 1 and the exponential map must give 50. */
  reset();
  stability_config_t c;
  stability_get_config(&c);
  c.w_omega = 1.0f;
  c.w_accel_lf = 0.0f;
  c.w_disp = 0.0f;
  stability_set_config(&c);
  const float gyro[3] = {0.05f, 0.0f, 0.0f};
  stability_output_t out = feed(4 * FRAME_HZ, gyro, ZERO3, Q_IDENTITY);
  CHECK_NEAR(out.instability, 1.0, 1e-4);
  CHECK_NEAR(out.score, 50.0, 0.01);

  /* Twice the baseline: I = 2, the score halves again. */
  const float gyro2[3] = {0.10f, 0.0f, 0.0f};
  out = feed(4 * FRAME_HZ, gyro2, ZERO3, Q_IDENTITY);
  CHECK_NEAR(out.instability, 2.0, 1e-4);
  CHECK_NEAR(out.score, 25.0, 0.01);
}

static void test_score_decreases_with_tremor(void) {
  const float levels[] = {0.02f, 0.05f, 0.10f, 0.20f, 0.40f};
  double previous = 101.0;
  for (unsigned i = 0; i < sizeof(levels) / sizeof(levels[0]); i++) {
    reset();
    const float gyro[3] = {levels[i], 0.0f, 0.0f};
    stability_output_t out = feed(4 * FRAME_HZ, gyro, ZERO3, Q_IDENTITY);
    CHECK(out.score < previous);
    CHECK(out.score > 0.0f);
    previous = out.score;
  }
}

static void test_instability_is_weighted_sum(void) {
  /* On a mixed motion the reported index must equal the weighted sum of
   * the three reported RMS terms, each divided by its baseline, and the
   * score must stay finite and inside 0..100 on every frame. */
  reset();
  stability_config_t c;
  stability_get_config(&c);
  sim_rng_t rng;
  sim_rng_seed(&rng, 7u);
  double worst = 0.0;
  bool bounded = true;
  for (int i = 0; i < 20 * FRAME_HZ; i++) {
    double t = (double)i / FRAME_HZ;
    float g[3];
    float a[3];
    float q[4];
    sim_noise3(&rng, 0.3f, g);
    sim_noise3(&rng, 2.0f, a);
    a[0] += (float)(1.0 + 0.5 * sin(2.0 * SIM_PI * 1.2 * t));
    quat_about_x(0.15 * sin(2.0 * SIM_PI * 0.4 * t), q);
    stability_output_t out;
    stability_tick(g, a, q, &out);
    double expected = c.w_omega * out.rms_omega / c.baseline_omega +
                      c.w_accel_lf * out.rms_accel_lf / c.baseline_accel_lf +
                      c.w_disp * out.rms_disp / c.baseline_disp;
    double err = fabs((double)out.instability - expected);
    if (err > worst) {
      worst = err;
    }
    if (!isfinite(out.score) || out.score < 0.0f || out.score > 100.0f) {
      bounded = false;
    }
  }
  CHECK(worst < 1e-3);
  CHECK(bounded);
}

/* ---- time response --------------------------------------------------- */

static void test_rate_term_response_time(void) {
  /* After the motion stops, the mean square decays as (1 - 0.20)^n:
   * 0.8^4 = 0.41 is still above 1/e, 0.8^5 = 0.33 is below. The rate term
   * therefore reacts within 80-100 ms, as the header promises. */
  reset();
  const float moving[3] = {0.5f, 0.0f, 0.0f};
  stability_output_t out = feed(2 * FRAME_HZ, moving, ZERO3, Q_IDENTITY);
  double start_sq = (double)out.rms_omega * (double)out.rms_omega;
  CHECK_NEAR(out.rms_omega, 0.5, 1e-4);

  out = feed(4, ZERO3, ZERO3, Q_IDENTITY);
  double after4 = (double)out.rms_omega * (double)out.rms_omega / start_sq;
  out = feed(1, ZERO3, ZERO3, Q_IDENTITY);
  double after5 = (double)out.rms_omega * (double)out.rms_omega / start_sq;
  CHECK_NEAR(after4, 0.4096, 1e-3);
  CHECK_NEAR(after5, 0.32768, 1e-3);
  CHECK(after4 > exp(-1.0) && after5 < exp(-1.0));
}

static void test_score_recovers_after_motion(void) {
  /* Live biofeedback: one second after the bow is steadied, the displayed
   * score must be back above 95. */
  reset();
  const float moving[3] = {0.5f, 0.0f, 0.0f};
  stability_output_t out = feed(2 * FRAME_HZ, moving, ZERO3, Q_IDENTITY);
  CHECK(out.score < 5.0f);
  out = feed(FRAME_HZ, ZERO3, ZERO3, Q_IDENTITY);
  CHECK(out.score > 95.0f);
}

/* ---- sway term (0.5-3 Hz band) --------------------------------------- */

static void test_sway_band_selectivity(void) {
  /* Theoretical gain of the 0.5-3 Hz bandpass (SciPy, tools/analysis.py):
   * 0.9993 at 1.5 Hz and 0.0135 at 15 Hz; a constant offset is removed. */
  const double amp = 0.40;
  const double rms_in = amp / sqrt(2.0);

  reset();
  double dc = sway_rms(20.0, 5.0, 3.0, 0.0, 0.0);
  CHECK(dc < 0.005);

  reset();
  double in_band = sway_rms(20.0, 5.0, 1.0, amp, 1.5);
  CHECK_NEAR(in_band, rms_in * 0.9993, 0.03);

  reset();
  double out_band = sway_rms(20.0, 5.0, 1.0, amp, 15.0);
  CHECK_NEAR(out_band, rms_in * 0.0135, 0.004);
  CHECK(in_band > 20.0 * out_band);

  printf("    sway RMS for a 0.40 m/s^2 sine: 1.5 Hz -> %.4f, 15 Hz -> %.4f, "
         "constant -> %.4f m/s^2\n",
         in_band, out_band, dc);
}

/* ---- drift term (orientation) ---------------------------------------- */

static void test_drift_step_and_recovery(void) {
  /* The bow is re-aimed by 0.2 rad in one frame. On that frame the angle
   * to the reference is exactly 0.2 rad:
   *   mean_sq = 0 + 0.10 * (0.2^2 - 0) = 0.004 -> RMS_d = 0.063246 rad.
   * The reference then follows the new orientation (tau ~0.5 s), so the
   * drift term fades out: a new aiming point is not an instability. */
  reset();
  float q_new[4];
  quat_about_x(0.2, q_new);
  feed(FRAME_HZ, ZERO3, ZERO3, Q_IDENTITY);
  stability_output_t out = feed(1, ZERO3, ZERO3, q_new);
  CHECK_NEAR(out.rms_disp, 0.063246, 2e-4);

  out = feed(5 * FRAME_HZ, ZERO3, ZERO3, q_new);
  CHECK(out.rms_disp < 0.002f);
  CHECK(out.score > 99.0f);
}

static void test_continuous_drift_lowers_score(void) {
  /* A bow that keeps drifting at 0.3 rad/s never lets the reference
   * catch up: the lag settles near w * (1 - a) / (a * fs) = 0.144 rad. */
  reset();
  stability_output_t out;
  memset(&out, 0, sizeof(out));
  for (int i = 0; i < 6 * FRAME_HZ; i++) {
    float q[4];
    quat_about_x(0.3 * (double)i / FRAME_HZ, q);
    stability_tick(ZERO3, ZERO3, q, &out);
  }
  CHECK_NEAR(out.rms_disp, 0.144, 0.01);
  CHECK(out.score < 70.0f);
}

static void test_quaternion_double_cover(void) {
  /* q and -q describe the same orientation; a sign flip of the sensor
   * output must not be read as a rotation. */
  reset();
  float q[4];
  quat_about_x(0.7, q);
  const float q_neg[4] = {-q[0], -q[1], -q[2], -q[3]};
  feed(FRAME_HZ, ZERO3, ZERO3, q);
  stability_output_t out = feed(FRAME_HZ, ZERO3, ZERO3, q_neg);
  CHECK(out.rms_disp < 1e-3f);
  CHECK(out.score > 99.9f);
  out = feed(FRAME_HZ, ZERO3, ZERO3, q);
  CHECK(out.rms_disp < 1e-3f);
  CHECK(out.score > 99.9f);
}

/* ---- robustness ------------------------------------------------------ */

static void test_non_unit_quaternion_is_clamped(void) {
  /* A slightly over-length quaternion gives a dot product above 1; without
   * the clamp acosf() would return NaN. */
  reset();
  const float q_long[4] = {0.0f, 0.0f, 0.0f, 1.2f};
  stability_output_t out = feed(FRAME_HZ, ZERO3, ZERO3, q_long);
  CHECK(isfinite(out.score));
  CHECK(isfinite(out.rms_disp));
  CHECK_NEAR(out.rms_disp, 0.0, 1e-6);
  CHECK_NEAR(out.score, 100.0, 1e-3);
}

static void test_zero_quaternion_stays_finite(void) {
  /* An all-zero quaternion (sensor not reporting yet): the reference
   * cannot be normalised, the frame must still produce finite numbers. */
  reset();
  const float q_zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  stability_output_t out = feed(FRAME_HZ, ZERO3, ZERO3, q_zero);
  CHECK(isfinite(out.score));
  CHECK(isfinite(out.instability));
  CHECK_NEAR(out.rms_disp, SIM_PI, 1e-3); /* 2 * acos(0) */
  CHECK(out.score >= 0.0f && out.score < 1.0f);

  /* Once real data arrives, the score recovers. */
  out = feed(10 * FRAME_HZ, ZERO3, ZERO3, Q_IDENTITY);
  CHECK(out.score > 99.0f);
}

static void test_nan_score_does_not_stick(void) {
  /* A non-finite config value poisons one frame; after the config is
   * repaired the smoothed score must re-seed instead of staying NaN. */
  reset();
  stability_config_t good;
  stability_get_config(&good);
  stability_config_t bad = good;
  bad.map_k = NAN;
  stability_set_config(&bad);
  stability_output_t out = feed(3, ZERO3, ZERO3, Q_IDENTITY);
  CHECK(isnan(out.score));

  stability_set_config(&good);
  out = feed(1, ZERO3, ZERO3, Q_IDENTITY);
  CHECK(isfinite(out.score));
  CHECK_NEAR(out.score, 100.0, 1e-3);
}

static void test_zero_baselines_do_not_divide_by_zero(void) {
  reset();
  stability_config_t c;
  stability_get_config(&c);
  c.baseline_omega = 0.0f;
  c.baseline_accel_lf = 0.0f;
  c.baseline_disp = 0.0f;
  stability_set_config(&c);

  stability_output_t out = feed(FRAME_HZ, ZERO3, ZERO3, Q_IDENTITY);
  CHECK(isfinite(out.instability));
  CHECK_NEAR(out.score, 100.0, 1e-3);

  const float gyro[3] = {0.05f, 0.0f, 0.0f};
  out = feed(FRAME_HZ, gyro, ZERO3, Q_IDENTITY);
  CHECK(isfinite(out.instability));
  CHECK(isfinite(out.score));
  CHECK(out.score >= 0.0f && out.score < 1.0f);
}

static void test_diagnostic_log_once_per_second(void) {
  reset();
  unsigned before = host_log_count();
  feed(FRAME_HZ - 1, ZERO3, ZERO3, Q_IDENTITY);
  CHECK_EQ_INT(host_log_count() - before, 0);
  feed(1, ZERO3, ZERO3, Q_IDENTITY);
  CHECK_EQ_INT(host_log_count() - before, 1);
  CHECK(host_log_find_last("score=100") != NULL);
  feed(3 * FRAME_HZ, ZERO3, ZERO3, Q_IDENTITY);
  CHECK_EQ_INT(host_log_count() - before, 4);
}

int main(void) {
  printf("stability host tests\n");
  RUN(test_defaults);
  RUN(test_null_arguments_are_safe);
  RUN(test_set_config_replaces_all_fields);
  RUN(test_still_device_scores_100);
  RUN(test_hand_calculated_second_frame);
  RUN(test_baseline_tremor_steady_state);
  RUN(test_unit_instability_halves_score);
  RUN(test_score_decreases_with_tremor);
  RUN(test_instability_is_weighted_sum);
  RUN(test_rate_term_response_time);
  RUN(test_score_recovers_after_motion);
  RUN(test_sway_band_selectivity);
  RUN(test_drift_step_and_recovery);
  RUN(test_continuous_drift_lowers_score);
  RUN(test_quaternion_double_cover);
  RUN(test_non_unit_quaternion_is_clamped);
  RUN(test_zero_quaternion_stays_finite);
  RUN(test_nan_score_does_not_stick);
  RUN(test_zero_baselines_do_not_divide_by_zero);
  RUN(test_diagnostic_log_once_per_second);
  return tu_summary("stability");
}
