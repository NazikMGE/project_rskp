#include "stability/stability.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

/**
 * @file stability.c
 * @brief Hold-stability score — implementation.
 *
 * Per 50 Hz frame, three EMA-weighted RMS estimates drive the score:
 *   - RMS_omega: |gyro|, single-pole EMA on the squared magnitude     (tremor)
 *   - RMS_accel: bandpass(|a|), single-pole EMA on the squared mag    (sway)
 *   - RMS_disp:  geodesic |q ⊖ qref|, single-pole EMA on the squared
 *                angle, where qref is a slow EMA of the orientation   (drift)
 *
 * Each term is normalised to a per-archer baseline, combined with
 * expert weights into an instability index I, and mapped to 0..100 by
 * an exponential decay (score = 100*exp(-k*I): I=0 -> 100, I=1 -> 50).
 * A display EMA smooths the result.
 *
 * Single-pole EMA replaces the earlier rectangular-window running mean
 * for one reason: live biofeedback. A rectangular window holds old
 * samples at equal weight until they evict — a step "moving → still"
 * stays elevated for a full window-width (~1 s here) before settling,
 * which broke the user-perceived feedback loop. An EMA reaches 63 % of
 * a step response in one tau and weighs recent samples more, so the
 * user sees the metric react in 100-200 ms rather than 1-2 s. The
 * trade-off is a noisier output sample-to-sample; the display EMA
 * (smoothing_alpha) absorbs that. Note: the per-frame output is a
 * weighted RMS, not a rectangular-window RMS — research-grade windowed
 * statistics are computed offline from the raw 50 Hz IMU stream that
 * the mobile app records per session.
 *
 * Constants here are starting points; the weights, the decay rate,
 * the baselines and the EMA alphas are all exposed through
 * @ref stability_config_t for the planned tuning UI.
 */

static const char *TAG = "stability";

/* ---- rates ---------------------------------------------------------- */

/** Frame rate the score and its filter coefficients assume. */
#define STAB_FS_HZ 50

/* ---- low-frequency accel bandpass ----------------------------------- */

/*
 * 0.5-3 Hz Butterworth bandpass at fs = 50 Hz, as a cascade of two
 * Direct Form II biquads. From
 *   scipy.signal.butter(2, [0.5, 3.0], 'band', fs=50, output='sos')
 * Run at 50 Hz on purpose: at 200 Hz the 0.5 Hz edge puts the poles at
 * radius ~0.99 (float32-fragile); at 50 Hz they sit at ~0.96.
 */
#define STAB_BP0_B0 0.0200833656f
#define STAB_BP0_B1 0.0401667311f
#define STAB_BP0_B2 0.0200833656f
#define STAB_BP0_A1 -1.5977831770f
#define STAB_BP0_A2 0.6929224132f
#define STAB_BP1_B0 1.0000000000f
#define STAB_BP1_B1 -2.0000000000f
#define STAB_BP1_B2 1.0000000000f
#define STAB_BP1_A1 -1.9207935713f
#define STAB_BP1_A2 0.9255748202f

/* ---- single-pole EMA RMS estimator ---------------------------------- */

/**
 * @brief Exponentially-weighted RMS estimator. O(1) state, no buffer.
 *
 * `mean_sq` carries the running EMA of the squared input; `sqrt` of it
 * is the current RMS. Seeded on the first sample to avoid the slow
 * ramp-up from zero a cold EMA would otherwise have.
 */
typedef struct {
  float mean_sq;
  bool seeded;
} ema_rms_t;

/* ---- Forward declarations -----------------------------------------
 *
 * Defined later in this file; declared up front so a future reorder
 * stays compile-safe. */

static void ema_rms_init(ema_rms_t *r);
static float ema_rms_push(ema_rms_t *r, float x, float alpha);
static float stab_bandpass(float x);

static void ema_rms_init(ema_rms_t *r) {
  r->mean_sq = 0.0f;
  r->seeded = false;
}

/**
 * @brief Push one sample, return updated RMS.
 *
 * The EMA carries float rounding error; a long run of near-zero samples
 * can leave `mean_sq` a hair below zero, so clamp before the square
 * root — otherwise sqrtf() yields NaN and poisons the score.
 */
static float ema_rms_push(ema_rms_t *r, float x, float alpha) {
  float sq = x * x;
  if (!r->seeded) {
    r->mean_sq = sq;
    r->seeded = true;
  } else {
    r->mean_sq += alpha * (sq - r->mean_sq);
  }
  return sqrtf(fmaxf(r->mean_sq, 0.0f));
}

/* ---- state ---------------------------------------------------------- */

static struct {
  /* low-frequency accel bandpass (2 cascaded biquads) */
  float bp0_w0, bp0_w1;
  float bp1_w0, bp1_w1;

  /* per-component EMA-RMS estimators */
  ema_rms_t omega_rms;
  ema_rms_t accel_rms;
  ema_rms_t disp_rms;

  /* low-passed reference orientation for the drift term */
  float qref[4];
  bool qref_seeded;

  /* display smoothing */
  float score_smoothed;
  bool score_seeded;

  stability_config_t cfg;
  uint32_t tick_count;
} stab;

/** @brief Two-section 0.5-3 Hz bandpass, Direct Form II. */
static float stab_bandpass(float x) {
  float d0 = x - STAB_BP0_A1 * stab.bp0_w0 - STAB_BP0_A2 * stab.bp0_w1;
  float y0 =
      STAB_BP0_B0 * d0 + STAB_BP0_B1 * stab.bp0_w0 + STAB_BP0_B2 * stab.bp0_w1;
  stab.bp0_w1 = stab.bp0_w0;
  stab.bp0_w0 = d0;

  float d1 = y0 - STAB_BP1_A1 * stab.bp1_w0 - STAB_BP1_A2 * stab.bp1_w1;
  float y1 =
      STAB_BP1_B0 * d1 + STAB_BP1_B1 * stab.bp1_w0 + STAB_BP1_B2 * stab.bp1_w1;
  stab.bp1_w1 = stab.bp1_w0;
  stab.bp1_w0 = d1;
  return y1;
}

void stability_init(void) {
  memset(&stab, 0, sizeof(stab));
  ema_rms_init(&stab.omega_rms);
  ema_rms_init(&stab.accel_rms);
  ema_rms_init(&stab.disp_rms);

  /* Defaults. Weights and map_k are expert-chosen; the baselines are
   * nominal placeholders — a per-archer calibration replaces them via
   * stability_set_config. map_k = ln(2): I=0 -> 100, I=1 -> 50.
   * baseline_disp is kept well above the ~0.013 rad game-RV noise
   * floor so a near-still input scores near the top.
   *
   * EMA alphas tuned for live biofeedback responsiveness — 63 % step
   * response in ~100-200 ms, perceived response well under 500 ms:
   *   - alpha_omega = 0.20  -> tau ~100 ms (tremor band is high-freq)
   *   - alpha_accel_lf = 0.10 -> tau ~200 ms (sway is sub-Hz)
   *   - alpha_disp = 0.10   -> tau ~200 ms (drift RMS)
   *   - alpha_qref = 0.04   -> tau ~500 ms (trailing reference orientation;
   *                                          slower so the drift term sees
   *                                          actual deviation, not noise)
   *   - smoothing_alpha = 0.25 -> tau ~80 ms (cosmetic; kills per-frame
   *                                            flicker without adding lag) */
  stab.cfg.w_omega = 0.45f;
  stab.cfg.w_accel_lf = 0.10f;
  stab.cfg.w_disp = 0.45f;
  stab.cfg.map_k = 0.6931f;
  stab.cfg.baseline_omega = 0.05f;
  stab.cfg.baseline_accel_lf = 0.20f;
  stab.cfg.baseline_disp = 0.10f;
  stab.cfg.alpha_omega = 0.20f;
  stab.cfg.alpha_accel_lf = 0.10f;
  stab.cfg.alpha_disp = 0.10f;
  stab.cfg.alpha_qref = 0.04f;
  stab.cfg.smoothing_alpha = 0.25f;

  ESP_LOGI(TAG, "ready — EMA-weighted hold score @ %d Hz", STAB_FS_HZ);
}

void stability_get_config(stability_config_t *cfg) {
  if (cfg != NULL) {
    *cfg = stab.cfg;
  }
}

void stability_set_config(const stability_config_t *cfg) {
  if (cfg == NULL) {
    return;
  }
  stab.cfg = *cfg;
  ESP_LOGI(TAG,
           "config: w[omega=%.2f accelLF=%.2f disp=%.2f] mapK=%.4f "
           "base[omega=%.4f accelLF=%.4f disp=%.4f] "
           "alpha[omega=%.3f accelLF=%.3f disp=%.3f qref=%.3f smooth=%.3f]",
           (double)stab.cfg.w_omega, (double)stab.cfg.w_accel_lf,
           (double)stab.cfg.w_disp, (double)stab.cfg.map_k,
           (double)stab.cfg.baseline_omega, (double)stab.cfg.baseline_accel_lf,
           (double)stab.cfg.baseline_disp, (double)stab.cfg.alpha_omega,
           (double)stab.cfg.alpha_accel_lf, (double)stab.cfg.alpha_disp,
           (double)stab.cfg.alpha_qref, (double)stab.cfg.smoothing_alpha);
}

void stability_tick(const float gyro_rad[3], const float lin_accel[3],
                    const float quat[4], stability_output_t *out) {
  stab.tick_count++;

  /* --- rate term: EMA-RMS of gyro magnitude --- */
  float w_mag = sqrtf(gyro_rad[0] * gyro_rad[0] + gyro_rad[1] * gyro_rad[1] +
                      gyro_rad[2] * gyro_rad[2]);
  float rms_omega = ema_rms_push(&stab.omega_rms, w_mag, stab.cfg.alpha_omega);

  /* --- sway term: EMA-RMS of bandpassed accel magnitude --- */
  float a_mag =
      sqrtf(lin_accel[0] * lin_accel[0] + lin_accel[1] * lin_accel[1] +
            lin_accel[2] * lin_accel[2]);
  float a_lf = stab_bandpass(a_mag);
  float rms_accel =
      ema_rms_push(&stab.accel_rms, a_lf, stab.cfg.alpha_accel_lf);

  /* --- drift term: geodesic angle from a low-passed reference --- */
  if (!stab.qref_seeded) {
    stab.qref[0] = quat[0];
    stab.qref[1] = quat[1];
    stab.qref[2] = quat[2];
    stab.qref[3] = quat[3];
    stab.qref_seeded = true;
  }
  /* 4-D dot product = cos(half the rotation angle between q and qref). */
  float qd = quat[0] * stab.qref[0] + quat[1] * stab.qref[1] +
             quat[2] * stab.qref[2] + quat[3] * stab.qref[3];
  float qd_abs = fabsf(qd);
  if (qd_abs > 1.0f) {
    qd_abs = 1.0f;
  }
  float ang_disp = 2.0f * acosf(qd_abs);
  float rms_disp = ema_rms_push(&stab.disp_rms, ang_disp, stab.cfg.alpha_disp);

  /* Low-pass the reference toward the current orientation; sign-align
   * for the quaternion double cover, then renormalise to unit. */
  float sign = (qd < 0.0f) ? -1.0f : 1.0f;
  for (int i = 0; i < 4; i++) {
    stab.qref[i] += stab.cfg.alpha_qref * (sign * quat[i] - stab.qref[i]);
  }
  float qn = sqrtf(stab.qref[0] * stab.qref[0] + stab.qref[1] * stab.qref[1] +
                   stab.qref[2] * stab.qref[2] + stab.qref[3] * stab.qref[3]);
  if (qn > 1e-6f) {
    stab.qref[0] /= qn;
    stab.qref[1] /= qn;
    stab.qref[2] /= qn;
    stab.qref[3] /= qn;
  }

  /* --- normalise, combine, map --- */
  float r_omega = rms_omega / fmaxf(stab.cfg.baseline_omega, 1e-6f);
  float r_accel = rms_accel / fmaxf(stab.cfg.baseline_accel_lf, 1e-6f);
  float r_disp = rms_disp / fmaxf(stab.cfg.baseline_disp, 1e-6f);
  float instability = stab.cfg.w_omega * r_omega +
                      stab.cfg.w_accel_lf * r_accel + stab.cfg.w_disp * r_disp;

  /* Exponential map: I=0 -> 100 (ideal), I=1 -> 50 (baseline); halves
   * every unit of I. expf of a large negative underflows to 0, so the
   * score stays within [0, 100] for any non-negative instability. */
  float score = 100.0f * expf(-stab.cfg.map_k * instability);

  /* Seed on the first tick; also re-seed if the smoothed value was ever
   * left non-finite, so a stray NaN can never stick permanently. */
  if (!stab.score_seeded || !isfinite(stab.score_smoothed)) {
    stab.score_smoothed = score;
    stab.score_seeded = true;
  } else {
    stab.score_smoothed +=
        stab.cfg.smoothing_alpha * (score - stab.score_smoothed);
  }

  if (out != NULL) {
    out->score = stab.score_smoothed;
    out->rms_omega = rms_omega;
    out->rms_accel_lf = rms_accel;
    out->rms_disp = rms_disp;
    out->instability = instability;
  }

  /* 1 Hz diagnostic — the data the weights/baselines are tuned against. */
  if (stab.tick_count % STAB_FS_HZ == 0) {
    ESP_LOGI(TAG, "score=%.0f I=%.2f | RMS w=%.4f a=%.4f d=%.4f",
             stab.score_smoothed, instability, rms_omega, rms_accel, rms_disp);
  }
}
