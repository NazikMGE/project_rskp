#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file stability.h
 * @brief Hold-stability score — a per-frame 0..100 quality metric.
 *
 * A frequency-decomposed instability index from three components — RMS
 * angular rate (tremor band), RMS low-frequency linear accel (postural
 * sway), and RMS angular displacement (orientation drift) — each
 * normalised to a per-archer baseline, combined with expert weights,
 * and mapped to 0..100 by an exponential decay (100 at the ideal).
 *
 * The three RMS values are single-pole EMA estimates of the squared
 * signal, NOT rectangular-window RMS. EMA was picked over windowed RMS
 * to keep the live biofeedback loop tight — 63 % step response in
 * 100-200 ms versus 1-2 s for a window the size of the previous
 * implementation. The trade-off is a noisier per-frame number; for
 * research-grade windowed statistics, the mobile app records the raw
 * 50 Hz IMU stream and post-hoc analysis runs there.
 *
 * This is a DISPLAY metric: no real-time decision depends on it (the
 * shot detector carries its own hold gate). It runs on the decimated
 * 50 Hz stream — the 0.5 Hz bandpass edge is well-conditioned there,
 * and the game-rotation-vector quaternion is natively 50 Hz. O(1) per
 * frame, no buffers; imu_task ticks it from the 50 Hz path.
 *
 * Tunable parameters live in @ref stability_config_t so the planned
 * experiment/tuning UI can adjust them without a firmware change.
 */

/**
 * @brief Runtime-tunable parameters.
 *
 * Defaults are applied by @ref stability_init. The baselines are
 * nominal placeholders — real values come from a per-archer
 * calibration pushed via @ref stability_set_config. The EMA alphas
 * control responsiveness: larger alpha = faster reaction, noisier
 * output. tau ≈ frame_period / alpha; perceived response (63 %) ≈ tau,
 * full settle (95 %) ≈ 3·tau.
 */
typedef struct {
  float w_omega;           /**< composite weight, rate term */
  float w_accel_lf;        /**< composite weight, sway term */
  float w_disp;            /**< composite weight, drift term */
  float map_k;             /**< score = 100 * exp(-map_k * I); the decay rate */
  float baseline_omega;    /**< baseline RMS angular rate (rad/s) */
  float baseline_accel_lf; /**< baseline RMS sway accel (m/s^2) */
  float baseline_disp;     /**< baseline RMS angular displacement (rad) */
  float alpha_omega;       /**< rate-RMS EMA alpha (~0.20, tau ~100 ms) */
  float alpha_accel_lf;    /**< sway-RMS EMA alpha (~0.10, tau ~200 ms) */
  float alpha_disp;        /**< drift-RMS EMA alpha (~0.10, tau ~200 ms) */
  float alpha_qref;        /**< qref low-pass alpha (~0.04, tau ~500 ms) */
  float smoothing_alpha;   /**< display EMA alpha (~0.25, tau ~80 ms) */
} stability_config_t;

/** @brief One frame of score output. */
typedef struct {
  float score;        /**< smoothed stability, 0..100 */
  float rms_omega;    /**< raw rate-term RMS (rad/s) */
  float rms_accel_lf; /**< raw sway-term RMS (m/s^2) */
  float rms_disp;     /**< raw drift-term RMS (rad) */
  float instability;  /**< composite instability index I */
} stability_output_t;

/** @brief Reset state and apply default config. Call once before ticking. */
void stability_init(void);

/** @brief Copy the current tunable config into @p cfg. */
void stability_get_config(stability_config_t *cfg);

/** @brief Replace the tunable config (e.g. from the tuning UI). */
void stability_set_config(const stability_config_t *cfg);

/**
 * @brief Feed one 50 Hz frame through the score.
 *
 * @param gyro_rad   Decimated calibrated gyro [x,y,z] in rad/s.
 * @param lin_accel  Decimated linear accel [x,y,z] in m/s^2.
 * @param quat       Game-rotation-vector quaternion [i,j,k,w].
 * @param out        Optional; filled with this frame's values if non-NULL.
 */
void stability_tick(const float gyro_rad[3], const float lin_accel[3],
                    const float quat[4], stability_output_t *out);

#ifdef __cplusplus
}
#endif
