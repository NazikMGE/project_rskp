#include "preshot/preshot.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

/**
 * @file preshot.c
 * @brief Shot-window ring buffer — implementation.
 *
 * A circular buffer of recent 200 Hz IMU samples in PSRAM. imu_task
 * appends every sample; on a detected release the current write
 * position is marked. The extraction window straddles the mark —
 * PRESHOT_PRE_SECONDS of the aiming approach before it and
 * PRESHOT_POST_SECONDS of release transient + follow-through after it.
 *
 * Rate note: gyro/accel are sampled at the BNO085's full 200 Hz rate
 * — kept for release-transient post-processing (the loose impulse and
 * string oscillation carry energy above the 25 Hz Nyquist of a 50 Hz
 * stream). The quaternion field is only refreshed every 4th tick
 * (Game RV is 50 Hz native) and the shot-review aim trace consequently
 * sources its quaternion stream from the 50 Hz session-chunk store,
 * not from this blob — see `local_session_imu_chunk`.
 *
 * Concurrency: imu_task is the sole producer (preshot_write and
 * preshot_mark_shot both run there). preshot_mark_shot stores `marked`
 * last with a release, so a consumer that loads it then sees the mark's
 * index and fill. The post-release samples are appended after the mark;
 * preshot_window_ready acquires the write index, so a copy only
 * proceeds once those samples are all published. The ring
 * (PRESHOT_RING_SECONDS) is sized well past the window, so the producer
 * needs far longer than any transfer to lap a frozen region.
 */

static const char *TAG = "preshot";

#define PRESHOT_FS_HZ 200
#define PRESHOT_RING_SECONDS 12
#define PRESHOT_PRE_SECONDS 4  /**< aiming approach before release */
#define PRESHOT_POST_SECONDS 2 /**< release + follow-through */

#define PRESHOT_RING_CAP (PRESHOT_FS_HZ * PRESHOT_RING_SECONDS)
#define PRESHOT_PRE_SAMPLES (PRESHOT_FS_HZ * PRESHOT_PRE_SECONDS)
#define PRESHOT_POST_SAMPLES (PRESHOT_FS_HZ * PRESHOT_POST_SECONDS)
#define PRESHOT_WINDOW_SAMPLES (PRESHOT_PRE_SAMPLES + PRESHOT_POST_SAMPLES)

static struct {
  preshot_sample_t *ring;     /**< PSRAM-backed circular storage */
  _Atomic uint32_t write_idx; /**< next slot to write */
  uint32_t fill;              /**< samples written, saturates at CAP */
  _Atomic bool marked;
  _Atomic uint32_t marked_idx;      /**< write_idx captured at the release */
  _Atomic uint32_t marked_fill;     /**< fill captured at the release */
  _Atomic uint32_t marked_onset_ms; /**< the release onset timestamp */
  bool disabled;                    /**< true when the PSRAM ring is absent */
} ps;

void preshot_init(void) {
  memset(&ps, 0, sizeof(ps));

  size_t bytes = sizeof(preshot_sample_t) * (size_t)PRESHOT_RING_CAP;
  ps.ring = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ps.ring == NULL) {
    ps.disabled = true;
    ESP_LOGW(TAG,
             "PSRAM alloc of %u B failed — shot-window buffer DISABLED; "
             "enable CONFIG_SPIRAM for the board",
             (unsigned)bytes);
    return;
  }
  ESP_LOGI(TAG, "ready — %d s ring (%d samples, %u B PSRAM)",
           PRESHOT_RING_SECONDS, PRESHOT_RING_CAP, (unsigned)bytes);
}

void preshot_write(uint32_t t_ms, const float gyro[3], const float accel[3],
                   const float quat[4]) {
  if (ps.disabled) {
    return;
  }
  uint32_t w = atomic_load_explicit(&ps.write_idx, memory_order_relaxed);
  preshot_sample_t *slot = &ps.ring[w];
  slot->t_ms = t_ms;
  memcpy(slot->gyro, gyro, sizeof(slot->gyro));
  memcpy(slot->accel, accel, sizeof(slot->accel));
  memcpy(slot->quat, quat, sizeof(slot->quat));

  /* Publish the slot before advancing the index — a consumer that sees
   * the new index also sees the data. */
  atomic_store_explicit(&ps.write_idx, (w + 1) % PRESHOT_RING_CAP,
                        memory_order_release);
  if (ps.fill < PRESHOT_RING_CAP) {
    ps.fill++;
  }
}

void preshot_mark_shot(uint32_t onset_ms) {
  if (ps.disabled) {
    return;
  }
  uint32_t w = atomic_load_explicit(&ps.write_idx, memory_order_relaxed);
  atomic_store(&ps.marked_idx, w);
  atomic_store(&ps.marked_fill, ps.fill);
  atomic_store(&ps.marked_onset_ms, onset_ms);
  /* Stored last — publishes marked_idx / marked_fill together. */
  atomic_store(&ps.marked, true);

  uint32_t pre = ps.fill < PRESHOT_PRE_SAMPLES ? ps.fill : PRESHOT_PRE_SAMPLES;
  ESP_LOGI(TAG,
           "shot marked: onset=%lu ms, %lu pre-release samples; "
           "post-release window filling",
           (unsigned long)onset_ms, (unsigned long)pre);
}

bool preshot_window_ready(void) {
  if (ps.disabled || !atomic_load(&ps.marked)) {
    return false;
  }
  uint32_t w = atomic_load_explicit(&ps.write_idx, memory_order_acquire);
  uint32_t m = atomic_load(&ps.marked_idx);
  uint32_t since = (w + PRESHOT_RING_CAP - m) % PRESHOT_RING_CAP;
  return since >= PRESHOT_POST_SAMPLES;
}

uint32_t preshot_marked_onset_ms(void) {
  return atomic_load(&ps.marked_onset_ms);
}

uint32_t preshot_copy(preshot_sample_t *dest, uint32_t max_samples,
                      uint32_t *onset_index) {
  if (ps.disabled || dest == NULL || !atomic_load(&ps.marked)) {
    return 0;
  }
  uint32_t w = atomic_load_explicit(&ps.write_idx, memory_order_acquire);
  uint32_t m = atomic_load(&ps.marked_idx);
  uint32_t mfill = atomic_load(&ps.marked_fill);

  /* Pre part: the approach already buffered at the mark, capped by how
   * much history the ring actually held that early in the session. */
  uint32_t pre = mfill < PRESHOT_PRE_SAMPLES ? mfill : PRESHOT_PRE_SAMPLES;

  /* Post part: samples imu_task has appended since the mark, capped at
   * the configured post-release window. */
  uint32_t since = (w + PRESHOT_RING_CAP - m) % PRESHOT_RING_CAP;
  uint32_t post = since < PRESHOT_POST_SAMPLES ? since : PRESHOT_POST_SAMPLES;

  uint32_t n = pre + post;
  if (n > max_samples) {
    n = max_samples;
  }
  if (n == 0) {
    return 0;
  }
  if (onset_index != NULL) {
    *onset_index = pre;
  }

  /* The window is n samples starting `pre` samples before the mark. */
  uint32_t start = (m + PRESHOT_RING_CAP - pre) % PRESHOT_RING_CAP;
  uint32_t first = PRESHOT_RING_CAP - start;
  if (first >= n) {
    memcpy(dest, &ps.ring[start], (size_t)n * sizeof(preshot_sample_t));
  } else {
    memcpy(dest, &ps.ring[start], (size_t)first * sizeof(preshot_sample_t));
    memcpy(dest + first, &ps.ring[0],
           (size_t)(n - first) * sizeof(preshot_sample_t));
  }
  return n;
}
