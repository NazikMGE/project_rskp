#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file preshot.h
 * @brief Shot-window ring buffer — IMU history captured around a shot.
 *
 * A circular buffer of 200 Hz IMU samples lives in PSRAM. imu_task
 * appends every sample; when the shot detector reports a release the
 * write position is marked. The extraction window straddles that mark —
 * a few seconds of the aiming approach BEFORE the release plus the
 * release transient and follow-through AFTER it — so a later transfer
 * carries the whole shot, not just the lead-up.
 *
 * Single-producer (imu_task). The write index and the mark are
 * published with release stores, so a consumer running on another task
 * — the future BLE bulk-transfer — reads a consistent window. The ring
 * is sized well past the window so the producer cannot lap a frozen
 * region before it is read.
 */

/** @brief One recorded 200 Hz IMU sample. */
typedef struct {
  uint32_t t_ms;  /**< sample timestamp, firmware monotonic ms */
  float gyro[3];  /**< calibrated gyro [x,y,z], rad/s */
  float accel[3]; /**< linear accel [x,y,z], m/s^2 */
  float quat[4];  /**< game rotation vector [i,j,k,w], most-recent */
} preshot_sample_t;

/**
 * @brief Allocate the PSRAM ring. Call once before the first write.
 *
 * If PSRAM is unavailable the component logs a warning and disables
 * itself — write/mark/copy become no-ops; the firmware is unaffected.
 */
void preshot_init(void);

/** @brief Append one 200 Hz sample. Called from imu_task; O(1). */
void preshot_write(uint32_t t_ms, const float gyro[3], const float accel[3],
                   const float quat[4]);

/**
 * @brief Mark a detected release in the ring.
 *
 * Records the current write position. The extraction window is then the
 * span around it — the pre-release approach already buffered plus the
 * post-release samples imu_task keeps appending after the mark.
 */
void preshot_mark_shot(uint32_t onset_ms);

/**
 * @brief Whether the full post-release window has been captured yet.
 *
 * False until a shot is marked and imu_task has appended the whole
 * post-release span after it. The BLE-transfer consumer polls this,
 * then calls @ref preshot_copy to lift the complete window.
 */
bool preshot_window_ready(void);

/**
 * @brief Copy the shot window — pre-release approach, the release, and
 *        the follow-through — into @p dest.
 *
 * Best called once @ref preshot_window_ready is true; called earlier it
 * still returns a window, but with the post-release portion truncated to
 * whatever has been captured so far.
 *
 * @param dest         Destination; pass capacity for the whole window.
 * @param max_samples  Capacity of @p dest, in samples.
 * @param onset_index  Optional out — index within @p dest of the marked
 *                      release sample (the pre/post split point).
 * @return Samples copied — 0 if nothing is marked or PSRAM is disabled.
 */
uint32_t preshot_copy(preshot_sample_t *dest, uint32_t max_samples,
                      uint32_t *onset_index);

/**
 * @brief Onset timestamp recorded by the most recent @ref preshot_mark_shot.
 *
 * Stable across the post-release window-fill — the BLE transfer reads
 * it after @ref preshot_window_ready is true to stamp the transfer
 * header. 0 before any mark has been recorded.
 */
uint32_t preshot_marked_onset_ms(void);

#ifdef __cplusplus
}
#endif
