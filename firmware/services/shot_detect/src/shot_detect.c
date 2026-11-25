#include "shot_detect/shot_detect.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

/**
 * @file shot_detect.c
 * @brief Variant-V4 archery release detector — implementation.
 *
 * Two parallel trigger channels run on each 200 Hz IMU sample:
 *
 *   accel:  |a|      -> highpass(10 Hz) -> square -> mean(25 ms) -> sqrt = E
 *   gyro:   |omega|                     -> square -> mean(25 ms) -> sqrt = G
 *
 * E is the high-frequency hand-shock envelope; G is the bow-rotation
 * envelope. Each has its own adaptive baseline (an EMA taken while the
 * bow is held), and a release is triggered when EITHER channel crosses
 * its baseline-scaled threshold — a heavily damped compound with a weak
 * hand-shock still snaps into a fast forward rotation and is caught on G.
 * A post-onset window then confirms the release by its ring-down (on
 * either channel). A small FSM — IDLE -> HOLD -> VALIDATING ->
 * REFRACTORY — sequences it.
 *
 * FSM durations (hold, settle, validation window, refractory) are
 * counted in SAMPLES, not wall-clock time: imu_task stamps each sample
 * at processing time, so a preempted task or batched IMU reports would
 * bunch the timestamps. One tick is one sample regardless, so sample
 * counts are immune to that. Wall-clock (`t_us`) is used only to form
 * the reported onset timestamp.
 *
 * Constants here are starting points; the floor values and the K
 * multipliers are meant to be tuned from real-bow envelope recordings —
 * the 1 Hz diagnostic log exists for exactly that.
 */

static const char *TAG = "shot_detect";

/* ---- signal chain --------------------------------------------------- */

/** Sample rate the detector and its filter coefficients assume. */
#define SD_FS_HZ 200

/*
 * Highpass: 2nd-order Butterworth, fc = 10 Hz at fs = 200 Hz, applied as
 * a Direct Form II biquad. Equivalent to
 *   scipy.signal.butter(2, 10, 'highpass', fs=200, output='sos')
 * fc/Nyquist = 0.1 sits well clear of the z=1 conditioning trap, so
 * float32 coefficients are safe here. SD_HP_A1/A2 are the normalised
 * denominator coefficients (the difference equation carries the sign).
 */
#define SD_HP_B0 0.80059240f
#define SD_HP_B1 -1.60118480f
#define SD_HP_B2 0.80059240f
#define SD_HP_A1 -1.56101808f
#define SD_HP_A2 0.64135154f

/** Running-mean window for both envelopes: 5 samples = 25 ms at 200 Hz. */
#define SD_RMS_WIN 5

/* ---- hold gate ------------------------------------------------------ */

/** EMA weight for the gyro hold metric (tau ~= 0.3 s at 200 Hz). */
#define SD_HOLD_GYRO_BETA 0.0166f

/** Default gyro mean-square hold-gate threshold, in (rad/s)^2 — about
 *  10 deg/s RMS. Generous so novices qualify; the trigger and
 *  ring-down checks do the real work. Used as the @ref
 *  shot_detect_config_t::hold_gyro_threshold_rad_sq default. */
#define SD_HOLD_GYRO_THRESH_SQ_DEFAULT 0.030f

/** Consecutive non-steady samples tolerated before the hold disarms.
 *  A brief aim wobble or sight micro-correction must not drop to IDLE
 *  and discard the hold's arm progress; a draw, far longer, still does.
 *  300 ms at 200 Hz. */
#define SD_HOLD_GRACE_SAMPLES (SD_FS_HZ * 300 / 1000)

/* ---- adaptive baselines + trigger ----------------------------------- */

/** EMA weight for the envelope baselines (tau ~= 1 s at 200 Hz). */
#define SD_BASELINE_ALPHA 0.005f

/** Default accel trigger ratio (threshold = max(floor, baseline * K)). */
#define SD_TRIG_K_DEFAULT 6.0f

/** Default accel envelope floor (m/s^2) — keeps a very steady archer
 *  from getting a hypersensitive trigger. */
#define SD_TRIG_FLOOR_DEFAULT 1.5f

/** Default gyro trigger ratio. Ratio-based against the held baseline,
 *  so it self-scales per bow and archer — no absolute dps figure. */
#define SD_GYRO_TRIG_K_DEFAULT 8.0f

/** Default gyro envelope floor (rad/s, ~29 deg/s) — backstop so a
 *  perfectly still device cannot get a near-zero gyro threshold. */
#define SD_GYRO_TRIG_FLOOR_DEFAULT 0.5f

/** Consecutive over-threshold samples needed to confirm a trigger. */
#define SD_TRIG_DEBOUNCE 3

/* ---- ring-down validation ------------------------------------------- */

/** Envelope above baseline * SD_CONF_K counts as ring-down activity. */
#define SD_CONF_K 3.0f

/** Fraction of the window that must be active for the sustained path. */
#define SD_SUSTAIN_FRAC 0.5f

/** Peak above baseline * SD_STRONG_K confirms even a short ring-down
 *  (heavily damped compounds) — the bimodal confirm's second path. */
#define SD_STRONG_K 25.0f

/* ---- timing — counted in samples at SD_FS_HZ, jitter-immune --------- */

/* Minimum hold time is runtime-tunable — see
 * @ref shot_detect_config_t::min_hold_ms. Default 500 ms (= SD_FS_HZ / 2
 * samples). Long enough to seed and settle the baselines past
 * SD_SETTLE_SAMPLES, short enough not to miss a fast recurve shooter
 * who releases soon after reaching anchor. */

/** Baselines frozen for the start of a hold — still-settling motion. */
#define SD_SETTLE_SAMPLES (SD_FS_HZ / 5) /* 200 ms */

/** Ring-down window, in samples after the onset crossing. */
#define SD_VALID_DELAY_SAMPLES (SD_FS_HZ * 30 / 1000) /* 30 ms */
#define SD_VALID_END_SAMPLES (SD_FS_HZ * 200 / 1000)  /* 200 ms */

/** Refractory after a confirmed shot / after a rejected trigger. */
#define SD_REFRACTORY_OK_SAMPLES (SD_FS_HZ * 3)           /* 3 s */
#define SD_REFRACTORY_REJ_SAMPLES (SD_FS_HZ * 300 / 1000) /* 300 ms */

/** RMS-window + highpass group delay, subtracted from the reported
 *  onset so it tracks the physical release. Tune against ground truth. */
#define SD_ENVELOPE_DELAY_US 12000ULL

/** Default minimum hold time before a release may be triggered, in
 *  milliseconds (matches SD_MIN_HOLD_SAMPLES at SD_FS_HZ). */
#define SD_MIN_HOLD_MS_DEFAULT 500u

/** Safe clamps for the BLE-pushed config — keeps a malformed write from
 *  stalling the detector (e.g. a near-zero hold time would make the
 *  detector fire mid-draw). Wide enough for any plausible tuning. */
#define SD_MIN_HOLD_MS_MIN 100u
#define SD_MIN_HOLD_MS_MAX 5000u

/** @brief Detector FSM state. */
typedef enum {
  SD_IDLE = 0,   /**< Bow not held steady. */
  SD_HOLD,       /**< Held steady; baselines tracking, trigger armed. */
  SD_VALIDATING, /**< Trigger fired; collecting the ring-down window. */
  SD_REFRACTORY, /**< Post-event cooldown. */
} sd_state_t;

/** @brief All detector state — single instance, one IMU. */
static struct {
  sd_state_t state;

  /* accel signal chain */
  float hp_w0, hp_w1;        /**< highpass biquad delay line */
  float rms_buf[SD_RMS_WIN]; /**< ring of squared highpass samples */
  float rms_sum;             /**< running sum of rms_buf */
  int rms_idx;
  int rms_count;
  float e_prev; /**< previous-tick accel envelope, for onset interpolation */

  /* gyro signal chain */
  float g_buf[SD_RMS_WIN]; /**< ring of gyro squared-magnitude samples */
  float g_sum;             /**< running sum of g_buf */
  int g_idx;
  int g_count;
  float g_env_prev; /**< previous-tick gyro envelope, for onset interpolation */

  uint64_t t_prev_us; /**< previous-tick timestamp */

  /* hold gate */
  float gyro_g2_ema;        /**< EMA of gyro squared-magnitude */
  uint32_t hold_start_tick; /**< tick_count when the steady stretch began */
  uint32_t unsteady_count;  /**< consecutive non-steady samples */

  /* adaptive baselines */
  float baseline;  /**< accel envelope baseline */
  float gyro_base; /**< gyro envelope baseline */
  bool baseline_seeded;

  /* trigger debounce */
  int trig_count;      /**< 0 = no candidate, >0 = debouncing */
  uint64_t onset_us;   /**< interpolated onset, wall clock — for the report */
  uint32_t onset_tick; /**< tick_count at the onset crossing — window timing */

  /* ring-down validation */
  int valid_total;
  int valid_active;
  float valid_peak;      /**< accel envelope peak — reported as peak_mps2 */
  float valid_gyro_peak; /**< gyro envelope peak — confirm strong-path only */

  /* refractory */
  uint32_t refractory_counter; /**< samples left in the cooldown */

  uint32_t tick_count;

  /* runtime-tunable parameters; defaults applied in shot_detect_init */
  shot_detect_config_t cfg;
  uint32_t min_hold_samples; /**< cached: cfg.min_hold_ms * SD_FS_HZ / 1000 */
} sd;

/* ---- Forward declarations ----------------------------------------- */

static const char *sd_state_name(sd_state_t s);
static uint32_t sd_onset_to_ms(uint64_t onset_us);
static void sd_apply_defaults(shot_detect_config_t *cfg);
static uint32_t sd_min_hold_samples(uint32_t min_hold_ms);

static const char *sd_state_name(sd_state_t s) {
  switch (s) {
  case SD_IDLE:
    return "IDLE";
  case SD_HOLD:
    return "HOLD";
  case SD_VALIDATING:
    return "VALIDATING";
  case SD_REFRACTORY:
    return "REFRACTORY";
  default:
    return "?";
  }
}

/**
 * @brief Onset microseconds to reported milliseconds, group-delay
 *        compensated so the timestamp tracks the physical release.
 */
static uint32_t sd_onset_to_ms(uint64_t onset_us) {
  uint64_t t =
      (onset_us > SD_ENVELOPE_DELAY_US) ? onset_us - SD_ENVELOPE_DELAY_US : 0;
  return (uint32_t)(t / 1000ULL);
}

static void sd_apply_defaults(shot_detect_config_t *cfg) {
  cfg->hold_gyro_threshold_rad_sq = SD_HOLD_GYRO_THRESH_SQ_DEFAULT;
  cfg->accel_trigger_ratio = SD_TRIG_K_DEFAULT;
  cfg->accel_trigger_floor_ms2 = SD_TRIG_FLOOR_DEFAULT;
  cfg->gyro_trigger_ratio = SD_GYRO_TRIG_K_DEFAULT;
  cfg->gyro_trigger_floor_rad_s = SD_GYRO_TRIG_FLOOR_DEFAULT;
  cfg->min_hold_ms = SD_MIN_HOLD_MS_DEFAULT;
}

/**
 * @brief Convert a hold time in ms to the equivalent sample count at
 *        the detector's fixed 200 Hz tick rate, clamped to a safe range.
 */
static uint32_t sd_min_hold_samples(uint32_t min_hold_ms) {
  if (min_hold_ms < SD_MIN_HOLD_MS_MIN) {
    min_hold_ms = SD_MIN_HOLD_MS_MIN;
  } else if (min_hold_ms > SD_MIN_HOLD_MS_MAX) {
    min_hold_ms = SD_MIN_HOLD_MS_MAX;
  }
  return min_hold_ms * SD_FS_HZ / 1000u;
}

void shot_detect_init(void) {
  memset(&sd, 0, sizeof(sd));
  sd.state = SD_IDLE;
  sd_apply_defaults(&sd.cfg);
  sd.min_hold_samples = sd_min_hold_samples(sd.cfg.min_hold_ms);
  ESP_LOGI(TAG, "ready — V4 dual-channel (accel + gyro) detector @ %d Hz",
           SD_FS_HZ);
}

void shot_detect_get_config(shot_detect_config_t *cfg) {
  if (cfg != NULL) {
    *cfg = sd.cfg;
  }
}

void shot_detect_set_config(const shot_detect_config_t *cfg) {
  if (cfg == NULL) {
    return;
  }
  /* Clamp the gates to a safe-bounded range — a 0 threshold would
   * disarm the hold gate (steady never returns true), a negative ratio
   * would invert the trigger logic. Mirrors the JSON-validated bands
   * the mobile UI uses, but cannot trust them. */
  shot_detect_config_t next = *cfg;
  if (!isfinite(next.hold_gyro_threshold_rad_sq) ||
      next.hold_gyro_threshold_rad_sq <= 0.0f) {
    next.hold_gyro_threshold_rad_sq = SD_HOLD_GYRO_THRESH_SQ_DEFAULT;
  }
  if (!isfinite(next.accel_trigger_ratio) || next.accel_trigger_ratio < 1.0f) {
    next.accel_trigger_ratio = SD_TRIG_K_DEFAULT;
  }
  if (!isfinite(next.accel_trigger_floor_ms2) ||
      next.accel_trigger_floor_ms2 < 0.0f) {
    next.accel_trigger_floor_ms2 = SD_TRIG_FLOOR_DEFAULT;
  }
  if (!isfinite(next.gyro_trigger_ratio) || next.gyro_trigger_ratio < 1.0f) {
    next.gyro_trigger_ratio = SD_GYRO_TRIG_K_DEFAULT;
  }
  if (!isfinite(next.gyro_trigger_floor_rad_s) ||
      next.gyro_trigger_floor_rad_s < 0.0f) {
    next.gyro_trigger_floor_rad_s = SD_GYRO_TRIG_FLOOR_DEFAULT;
  }
  sd.cfg = next;
  sd.min_hold_samples = sd_min_hold_samples(next.min_hold_ms);
  ESP_LOGI(TAG,
           "config: hold=%.4f rad^2/s^2 accelK=%.2f accelFloor=%.2f m/s^2 "
           "gyroK=%.2f gyroFloor=%.2f rad/s minHold=%lu ms",
           (double)sd.cfg.hold_gyro_threshold_rad_sq,
           (double)sd.cfg.accel_trigger_ratio,
           (double)sd.cfg.accel_trigger_floor_ms2,
           (double)sd.cfg.gyro_trigger_ratio,
           (double)sd.cfg.gyro_trigger_floor_rad_s,
           (unsigned long)sd.cfg.min_hold_ms);
}

bool shot_detect_tick(uint64_t t_us, const float gyro_rad[3],
                      const float lin_accel[3], shot_event_t *out_event) {
  sd.tick_count++;

  /* accel envelope: |a| -> HP(10Hz) -> square -> mean(25ms) -> sqrt */
  float mag = sqrtf(lin_accel[0] * lin_accel[0] + lin_accel[1] * lin_accel[1] +
                    lin_accel[2] * lin_accel[2]);

  float d0 = mag - SD_HP_A1 * sd.hp_w0 - SD_HP_A2 * sd.hp_w1;
  float hp = SD_HP_B0 * d0 + SD_HP_B1 * sd.hp_w0 + SD_HP_B2 * sd.hp_w1;
  sd.hp_w1 = sd.hp_w0;
  sd.hp_w0 = d0;

  /* Running-mean of the highpass energy — the running sum keeps it
   * O(1); rms_buf[idx] is the sample being evicted (0 until filled). */
  float sq = hp * hp;
  sd.rms_sum += sq - sd.rms_buf[sd.rms_idx];
  sd.rms_buf[sd.rms_idx] = sq;
  sd.rms_idx = (sd.rms_idx + 1) % SD_RMS_WIN;
  if (sd.rms_count < SD_RMS_WIN) {
    sd.rms_count++;
  }
  /* fmaxf: the running sum carries float drift — a window of near-zero
   * samples can leave the mean a hair below zero; clamp before sqrt. */
  float e = sqrtf(fmaxf(sd.rms_sum / (float)sd.rms_count, 0.0f));

  /* --- gyro: squared magnitude drives both the hold gate (a slow EMA)
   *     and a short-window-RMS release channel (the bow-rotation
   *     transient at release) --- */
  float g2 = gyro_rad[0] * gyro_rad[0] + gyro_rad[1] * gyro_rad[1] +
             gyro_rad[2] * gyro_rad[2];

  sd.g_sum += g2 - sd.g_buf[sd.g_idx];
  sd.g_buf[sd.g_idx] = g2;
  sd.g_idx = (sd.g_idx + 1) % SD_RMS_WIN;
  if (sd.g_count < SD_RMS_WIN) {
    sd.g_count++;
  }
  float g_env = sqrtf(fmaxf(sd.g_sum / (float)sd.g_count, 0.0f));

  /* hold gate: low-passed gyro squared-magnitude */
  sd.gyro_g2_ema += SD_HOLD_GYRO_BETA * (g2 - sd.gyro_g2_ema);
  bool steady = sd.gyro_g2_ema < sd.cfg.hold_gyro_threshold_rad_sq;

  bool produced = false;

  switch (sd.state) {
  case SD_IDLE:
    if (steady) {
      sd.state = SD_HOLD;
      sd.hold_start_tick = sd.tick_count;
      sd.trig_count = 0;
      sd.unsteady_count = 0;
    }
    break;

  case SD_HOLD: {
    /* Hold-gate hysteresis: a brief unsteady burst (aim wobble, a sight
     * micro-correction) is tolerated; only a sustained one — a draw —
     * disarms the detector and discards the hold's arm progress. */
    if (steady) {
      sd.unsteady_count = 0;
    } else if (++sd.unsteady_count > SD_HOLD_GRACE_SAMPLES) {
      sd.state = SD_IDLE;
      sd.trig_count = 0;
      break;
    }

    bool pending = sd.trig_count > 0;
    uint32_t held = sd.tick_count - sd.hold_start_tick;

    /* Track both baselines only while genuinely holding — settled, with
     * no trigger candidate in flight — so a release transient can never
     * pull a baseline (and thus a threshold) up after itself. The
     * baselines persist across holds; they are never reset. */
    if (!pending && held >= SD_SETTLE_SAMPLES) {
      if (!sd.baseline_seeded) {
        sd.baseline = e;
        sd.gyro_base = g_env;
        sd.baseline_seeded = true;
      } else {
        sd.baseline += SD_BASELINE_ALPHA * (e - sd.baseline);
        sd.gyro_base += SD_BASELINE_ALPHA * (g_env - sd.gyro_base);
      }
    }

    /* Two parallel trigger channels: accel hand-shock and gyro bow
     * rotation. A crossing on EITHER arms / advances a release. */
    float th = sd.baseline * sd.cfg.accel_trigger_ratio;
    if (th < sd.cfg.accel_trigger_floor_ms2) {
      th = sd.cfg.accel_trigger_floor_ms2;
    }
    float g_th = sd.gyro_base * sd.cfg.gyro_trigger_ratio;
    if (g_th < sd.cfg.gyro_trigger_floor_rad_s) {
      g_th = sd.cfg.gyro_trigger_floor_rad_s;
    }

    if (pending) {
      if (e > th || g_env > g_th) {
        if (++sd.trig_count >= SD_TRIG_DEBOUNCE) {
          sd.state = SD_VALIDATING;
          sd.valid_total = 0;
          sd.valid_active = 0;
          sd.valid_peak = 0.0f;
          sd.valid_gyro_peak = 0.0f;
          out_event->type = SHOT_EVT_PROVISIONAL;
          out_event->onset_ms = sd_onset_to_ms(sd.onset_us);
          out_event->peak_mps2 = 0.0f;
          produced = true;
          ESP_LOGI(TAG, "trigger: onset=%lu ms (E=%.2f/%.2f G=%.2f/%.2f)",
                   (unsigned long)out_event->onset_ms, e, th, g_env, g_th);
        }
      } else {
        sd.trig_count = 0; /* sub-debounce blip — not a release */
      }
    } else {
      bool armed = sd.baseline_seeded && held >= sd.min_hold_samples;
      bool accel_cross = sd.e_prev <= th && e > th;
      bool gyro_cross = sd.g_env_prev <= g_th && g_env > g_th;
      if (armed && (accel_cross || gyro_cross)) {
        /* Rising-edge crossing — record the tick for window timing and
         * interpolate the wall-clock onset on whichever channel crossed
         * (accel takes precedence when both crossed the same tick). */
        float prev = accel_cross ? sd.e_prev : sd.g_env_prev;
        float cur = accel_cross ? e : g_env;
        float lvl = accel_cross ? th : g_th;
        float span = cur - prev;
        float frac = span > 0.0f ? (lvl - prev) / span : 0.0f;
        if (frac < 0.0f) {
          frac = 0.0f;
        } else if (frac > 1.0f) {
          frac = 1.0f;
        }
        sd.onset_us =
            sd.t_prev_us + (uint64_t)(frac * (float)(t_us - sd.t_prev_us));
        sd.onset_tick = sd.tick_count;
        sd.trig_count = 1;
      }
    }
    break;
  }

  case SD_VALIDATING: {
    /* since = samples elapsed from the onset crossing — the debounce
     * offset is absorbed because it counts from onset_tick, not the
     * trigger-confirm tick. */
    uint32_t since = sd.tick_count - sd.onset_tick;
    if (since >= SD_VALID_DELAY_SAMPLES && since <= SD_VALID_END_SAMPLES) {
      sd.valid_total++;
      /* a sample is ring-down activity if EITHER channel is elevated —
       * a gyro-triggered release with a weak hand-shock still confirms. */
      if (e > sd.baseline * SD_CONF_K || g_env > sd.gyro_base * SD_CONF_K) {
        sd.valid_active++;
      }
      if (e > sd.valid_peak) {
        sd.valid_peak = e;
      }
      if (g_env > sd.valid_gyro_peak) {
        sd.valid_gyro_peak = g_env;
      }
    }
    if (since > SD_VALID_END_SAMPLES) {
      float frac = sd.valid_total > 0
                       ? (float)sd.valid_active / (float)sd.valid_total
                       : 0.0f;
      bool ring_down = frac > SD_SUSTAIN_FRAC;
      bool strong = sd.valid_peak > sd.baseline * SD_STRONG_K ||
                    sd.valid_gyro_peak > sd.gyro_base * SD_STRONG_K;
      bool confirmed = ring_down || strong;

      out_event->type = confirmed ? SHOT_EVT_CONFIRMED : SHOT_EVT_REJECTED;
      out_event->onset_ms = sd_onset_to_ms(sd.onset_us);
      out_event->peak_mps2 = sd.valid_peak;
      produced = true;

      sd.refractory_counter =
          confirmed ? SD_REFRACTORY_OK_SAMPLES : SD_REFRACTORY_REJ_SAMPLES;
      sd.state = SD_REFRACTORY;
      ESP_LOGI(TAG, "%s: onset=%lu ms peak=%.2f gpeak=%.2f sustain=%d/%d",
               confirmed ? "CONFIRMED" : "rejected",
               (unsigned long)out_event->onset_ms, sd.valid_peak,
               sd.valid_gyro_peak, sd.valid_active, sd.valid_total);
    }
    break;
  }

  case SD_REFRACTORY:
    if (sd.refractory_counter > 0) {
      sd.refractory_counter--;
    }
    if (sd.refractory_counter == 0) {
      sd.state = SD_IDLE;
      sd.trig_count = 0;
    }
    break;
  }

  sd.e_prev = e;
  sd.g_env_prev = g_env;
  sd.t_prev_us = t_us;

  /* 1 Hz diagnostic — the data you tune the floors / the K's against. */
  if (sd.tick_count % SD_FS_HZ == 0) {
    ESP_LOGI(TAG, "[%s] E=%.3f base=%.3f | G=%.3f gbase=%.3f",
             sd_state_name(sd.state), e, sd.baseline, g_env, sd.gyro_base);
  }

  return produced;
}
