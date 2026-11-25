/**
 * @file shot_detect_test.c
 * @brief Host unit tests for the V4 release detector (services/shot_detect).
 *
 * The unmodified firmware source is compiled with a desktop GCC against the
 * `stubs/esp_log.h` stand-in and driven sample by sample with deterministic
 * synthetic signals. Each test documents the situation on the shooting line
 * it models and the detector branch it is meant to exercise.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "host_log.h"
#include "shot_detect/shot_detect.h"
#include "sim_signal.h"
#include "test_util.h"

/* ---- test rig -------------------------------------------------------- */

#define RIG_MAX_EVENTS 16

/** Quiet-hold noise amplitudes (uniform, per axis). */
#define QUIET_GYRO_NOISE 0.03f  /* rad/s  */
#define QUIET_ACCEL_NOISE 0.15f /* m/s^2 */

/** Parameters of a synthetic release transient. */
typedef struct {
  double a_peak; /**< accel burst amplitude, m/s^2 */
  double a_tau;  /**< accel decay time constant, s */
  double g_peak; /**< gyro burst amplitude, rad/s */
  double g_tau;  /**< gyro decay time constant, s */
} release_t;

/** A recurve-like release: strong hand-shock plus a forward bow roll. */
static const release_t RECURVE = {75.0, 0.10, 2.5, 0.15};

static struct {
  uint64_t t_us;  /**< timestamp of the next sample */
  uint64_t dt_us; /**< timestamp step per sample */
  long index;     /**< samples fed so far */
  sim_rng_t rng;
  float gyro_noise;
  float accel_noise;
  int n_events;
  shot_event_t events[RIG_MAX_EVENTS];
  long event_index[RIG_MAX_EVENTS]; /**< index of the producing sample */
} rig;

static void rig_reset(void) {
  memset(&rig, 0, sizeof(rig));
  rig.t_us = SIM_T0_US;
  rig.dt_us = SIM_DT_US;
  rig.gyro_noise = QUIET_GYRO_NOISE;
  rig.accel_noise = QUIET_ACCEL_NOISE;
  sim_rng_seed(&rig.rng, 20261116u);
  host_log_reset();
  shot_detect_init();
}

/** Current rig time on the detector clock, milliseconds. */
static uint32_t rig_now_ms(void) { return (uint32_t)(rig.t_us / 1000ULL); }

/** Feed one sample: the given signal plus the rig's background noise. */
static void rig_step(const float gyro_sig[3], const float accel_sig[3]) {
  float g[3];
  float a[3];
  sim_noise3(&rig.rng, rig.gyro_noise, g);
  sim_noise3(&rig.rng, rig.accel_noise, a);
  sim_add3(g, gyro_sig);
  sim_add3(a, accel_sig);

  shot_event_t ev;
  if (shot_detect_tick(rig.t_us, g, a, &ev)) {
    if (rig.n_events < RIG_MAX_EVENTS) {
      rig.events[rig.n_events] = ev;
      rig.event_index[rig.n_events] = rig.index;
    }
    rig.n_events++;
  }
  rig.t_us += rig.dt_us;
  rig.index++;
}

/** @p n samples of a quiet hold (background noise only). */
static void rig_hold(int n) {
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < n; i++) {
    rig_step(zero, zero);
  }
}

/** @p n samples of a slow, large bow motion (the draw / lowering the bow). */
static void rig_motion(int n, double gyro_amp) {
  for (int i = 0; i < n; i++) {
    double t = (double)i / SIM_FS_HZ;
    float g[3] = {(float)(gyro_amp * sin(2.0 * SIM_PI * 1.3 * t + 0.5)),
                  (float)(gyro_amp * cos(2.0 * SIM_PI * 0.9 * t)),
                  (float)(0.5 * gyro_amp * sin(2.0 * SIM_PI * 2.1 * t))};
    float a[3] = {(float)(1.5 * sin(2.0 * SIM_PI * 1.1 * t)),
                  (float)(1.0 * cos(2.0 * SIM_PI * 0.7 * t)), 0.0f};
    rig_step(g, a);
  }
}

/**
 * @brief @p n samples of a release transient starting at the first sample.
 * @return Timestamp (ms) of that first sample - the "true" release moment.
 */
static uint32_t rig_release(int n, const release_t *r) {
  uint32_t t_release_ms = rig_now_ms();
  for (int i = 0; i < n; i++) {
    double t = (double)i / SIM_FS_HZ;
    float a[3];
    float g[3];
    sim_release_accel(t, r->a_peak, r->a_tau, a);
    sim_release_gyro(t, r->g_peak, r->g_tau, g);
    rig_step(g, a);
  }
  return t_release_ms;
}

/** One sample with a short knock of @p amp m/s^2 along x (a bumped bow). */
static void rig_bump(float amp) {
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  const float a[3] = {amp, 0.0f, 0.0f};
  rig_step(zero, a);
}

/** Read "sustain=a/b" from the detector's last validation log line. */
static bool log_sustain(int *active, int *total) {
  const char *line = host_log_find_last("sustain=");
  if (line == NULL) {
    return false;
  }
  const char *p = strstr(line, "sustain=");
  return sscanf(p, "sustain=%d/%d", active, total) == 2;
}

/** Read "gbase=x" (gyro baseline) from the last 1 Hz diagnostic line. */
static bool log_gyro_base(float *gbase) {
  const char *line = host_log_find_last("gbase=");
  if (line == NULL) {
    return false;
  }
  return sscanf(strstr(line, "gbase="), "gbase=%f", gbase) == 1;
}

/** True when the rig recorded exactly PROVISIONAL followed by @p final. */
static bool rig_is_pair(int first, shot_event_type_t final) {
  return rig.n_events >= first + 2 &&
         rig.events[first].type == SHOT_EVT_PROVISIONAL &&
         rig.events[first + 1].type == final &&
         rig.events[first].onset_ms == rig.events[first + 1].onset_ms;
}

/* ---- configuration --------------------------------------------------- */

static void test_defaults_and_null_safety(void) {
  rig_reset();
  shot_detect_config_t c;
  memset(&c, 0xA5, sizeof(c));
  shot_detect_get_config(&c);
  CHECK_NEAR(c.hold_gyro_threshold_rad_sq, 0.030, 1e-6);
  CHECK_NEAR(c.accel_trigger_ratio, 6.0, 1e-6);
  CHECK_NEAR(c.accel_trigger_floor_ms2, 1.5, 1e-6);
  CHECK_NEAR(c.gyro_trigger_ratio, 8.0, 1e-6);
  CHECK_NEAR(c.gyro_trigger_floor_rad_s, 0.5, 1e-6);
  CHECK_EQ_INT(c.min_hold_ms, 500);

  /* NULL arguments are documented no-ops. */
  shot_detect_get_config(NULL);
  shot_detect_set_config(NULL);
  shot_detect_config_t after;
  shot_detect_get_config(&after);
  CHECK(memcmp(&c, &after, sizeof(c)) == 0);
  CHECK(host_log_find_last("ready") != NULL);
}

static void test_config_rejects_invalid_values(void) {
  rig_reset();

  /* Non-finite values (a corrupted BLE write) fall back to the defaults. */
  shot_detect_config_t bad = {NAN, INFINITY, NAN, -INFINITY, NAN, 700u};
  shot_detect_set_config(&bad);
  shot_detect_config_t c;
  shot_detect_get_config(&c);
  CHECK_NEAR(c.hold_gyro_threshold_rad_sq, 0.030, 1e-6);
  CHECK_NEAR(c.accel_trigger_ratio, 6.0, 1e-6);
  CHECK_NEAR(c.accel_trigger_floor_ms2, 1.5, 1e-6);
  CHECK_NEAR(c.gyro_trigger_ratio, 8.0, 1e-6);
  CHECK_NEAR(c.gyro_trigger_floor_rad_s, 0.5, 1e-6);
  CHECK_EQ_INT(c.min_hold_ms, 700);

  /* Finite but out-of-range values are replaced as well. */
  shot_detect_config_t low = {0.0f, 0.99f, -0.1f, 0.5f, -1.0f, 700u};
  shot_detect_set_config(&low);
  shot_detect_get_config(&c);
  CHECK_NEAR(c.hold_gyro_threshold_rad_sq, 0.030, 1e-6);
  CHECK_NEAR(c.accel_trigger_ratio, 6.0, 1e-6);
  CHECK_NEAR(c.accel_trigger_floor_ms2, 1.5, 1e-6);
  CHECK_NEAR(c.gyro_trigger_ratio, 8.0, 1e-6);
  CHECK_NEAR(c.gyro_trigger_floor_rad_s, 0.5, 1e-6);
  CHECK(host_log_find_last("config:") != NULL);
}

static void test_config_keeps_valid_values(void) {
  rig_reset();
  shot_detect_config_t good = {0.050f, 4.0f, 2.0f, 10.0f, 0.8f, 800u};
  shot_detect_set_config(&good);
  shot_detect_config_t c;
  shot_detect_get_config(&c);
  CHECK(memcmp(&good, &c, sizeof(c)) == 0);

  /* The lowest values the setter still accepts: ratio 1.0 and floor 0.0. */
  shot_detect_config_t edge = {0.001f, 1.0f, 0.0f, 1.0f, 0.0f, 100u};
  shot_detect_set_config(&edge);
  shot_detect_get_config(&c);
  CHECK(memcmp(&edge, &c, sizeof(c)) == 0);
}

/* ---- detection ------------------------------------------------------- */

static void test_quiet_hold_produces_no_events(void) {
  rig_reset();
  /* out_event must stay untouched while no event is produced. */
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  shot_event_t ev;
  memset(&ev, 0x5A, sizeof(ev));
  shot_event_t untouched = ev;
  bool produced = shot_detect_tick(rig.t_us, zero, zero, &ev);
  CHECK(!produced);
  CHECK(memcmp(&ev, &untouched, sizeof(ev)) == 0);

  rig_hold(10 * SIM_FS_HZ); /* 10 s of a steady aim */
  CHECK_EQ_INT(rig.n_events, 0);
  CHECK(host_log_find_last("[HOLD]") != NULL);
}

static void test_clear_release_two_stage_event(void) {
  rig_reset();
  rig_hold(2 * SIM_FS_HZ);
  long release_index = rig.index;
  uint32_t t_release = rig_release(2 * SIM_FS_HZ, &RECURVE);

  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));

  /* Stage 1: the provisional marker arrives within 15 ms of the release. */
  long latency = rig.event_index[0] - release_index;
  CHECK(latency >= 2 && latency <= 3);
  CHECK_NEAR(rig.events[0].peak_mps2, 0.0, 1e-9);

  /* Stage 2: the verdict comes once the 200 ms ring-down window closes. */
  long verdict = rig.event_index[1] - release_index;
  CHECK(verdict >= 40 && verdict <= 43);
  CHECK(rig.events[1].peak_mps2 > 5.0f);

  /* Reported onset: within one envelope window (25 ms) of the release. */
  double onset_err_ms = (double)rig.events[1].onset_ms - (double)t_release;
  CHECK(onset_err_ms >= -25.0 && onset_err_ms <= 5.0);

  /* Both confirmation paths agree on a clear shot. */
  int active = 0;
  int total = 0;
  CHECK(log_sustain(&active, &total));
  CHECK_EQ_INT(total, 35); /* samples 6..40 after the onset = 30..200 ms */
  CHECK(2 * active > total);
  CHECK(host_log_find_last("CONFIRMED") != NULL);

  printf("    clear release: provisional +%ld ms, verdict +%ld ms, "
         "onset error %+.0f ms, peak %.1f m/s^2, sustain %d/%d\n",
         latency * 5, verdict * 5, onset_err_ms,
         (double)rig.events[1].peak_mps2, active, total);
}

static void test_gyro_only_release(void) {
  rig_reset();
  rig_hold(2 * SIM_FS_HZ);
  /* A heavily damped bow: almost no hand-shock, but it still rolls forward. */
  const release_t damped = {0.0, 0.10, 2.5, 0.15};
  rig_release(2 * SIM_FS_HZ, &damped);

  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));
  /* The accel channel never left its noise floor. */
  CHECK(rig.events[1].peak_mps2 < 0.5f);
  const char *trigger = host_log_find_last("trigger:");
  CHECK(trigger != NULL);
}

static void test_bump_is_rejected(void) {
  rig_reset();
  rig_hold(2 * SIM_FS_HZ);
  rig_bump(4.5f); /* someone touched the bow: one 5 ms knock */
  rig_hold(SIM_FS_HZ);

  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_REJECTED));
  int active = 0;
  int total = 0;
  CHECK(log_sustain(&active, &total));
  CHECK(2 * active <= total); /* no ring-down */
  CHECK(host_log_find_last("rejected") != NULL);
  printf("    bump 4.5 m/s^2: peak %.2f m/s^2, sustain %d/%d -> rejected\n",
         (double)rig.events[1].peak_mps2, active, total);
}

static void test_short_strong_impulse_confirmed_by_peak(void) {
  rig_reset();
  rig_hold(2 * SIM_FS_HZ);
  /* Very short but intense transient: dies out within ~20 ms. */
  const release_t snap = {60.0, 0.005, 0.0, 0.10};
  rig_release(SIM_FS_HZ, &snap);

  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));
  int active = 0;
  int total = 0;
  CHECK(log_sustain(&active, &total));
  CHECK(2 * active <= total); /* the ring-down path alone would reject it */
  printf("    short impulse: peak %.2f m/s^2, sustain %d/%d -> confirmed "
         "by peak\n",
         (double)rig.events[1].peak_mps2, active, total);
}

static void test_slow_lowering_confirmed_by_ringdown(void) {
  rig_reset();
  rig.gyro_noise = 0.04f;
  rig_hold(2 * SIM_FS_HZ);
  /* The archer lets the bow down without shooting: a slow rotation that
   * ramps up in 50 ms and lasts 0.4 s, with no hand-shock at all. */
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < SIM_FS_HZ; i++) {
    double t = (double)i / SIM_FS_HZ;
    double w = t < 0.05 ? 0.65 * t / 0.05 : (t < 0.45 ? 0.65 : 0.0);
    float g[3] = {(float)w, 0.0f, 0.0f};
    rig_step(g, zero);
  }

  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));
  /* Sustained activity confirms it, yet the release peak stays tiny - the
   * signature by which such weak events are told apart afterwards. */
  int active = 0;
  int total = 0;
  CHECK(log_sustain(&active, &total));
  CHECK(2 * active > total);
  CHECK(rig.events[1].peak_mps2 < 1.0f);
  printf("    bow lowering: peak %.2f m/s^2, sustain %d/%d -> weak event\n",
         (double)rig.events[1].peak_mps2, active, total);
}

static void test_sub_debounce_blip_ignored(void) {
  rig_reset();
  rig.gyro_noise = 0.0f; /* exact arithmetic on the gyro channel */
  rig_hold(2 * SIM_FS_HZ);

  /* Two gyro samples sized so the 5-sample mean square exceeds the floor
   * (0.5 rad/s)^2 for exactly one tick: 1.2 alone is below 1.25, 1.2 + 0.1
   * is above, and one tick later the 1.2 sample leaves the window. */
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  const float first[3] = {sqrtf(1.2f), 0.0f, 0.0f};
  const float second[3] = {sqrtf(0.1f), 0.0f, 0.0f};
  rig_step(first, zero);
  rig_hold(3);
  rig_step(second, zero);
  rig_hold(SIM_FS_HZ);
  CHECK_EQ_INT(rig.n_events, 0);

  /* The detector is still armed: a real release right after is caught. */
  rig.gyro_noise = QUIET_GYRO_NOISE;
  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));
}

/* ---- hold gate ------------------------------------------------------- */

static void test_not_armed_before_min_hold(void) {
  rig_reset();
  rig_hold(60); /* only 0.3 s of hold - below the 0.5 s minimum */
  long release_index = rig.index;
  rig_release(40, &RECURVE);
  /* No trigger may be reported for this release onset. */
  for (int i = 0; i < rig.n_events && i < RIG_MAX_EVENTS; i++) {
    CHECK(rig.event_index[i] - release_index > 5);
  }
  CHECK_EQ_INT(rig.n_events, 0);
}

static void test_min_hold_is_clamped(void) {
  /* A 10 ms request is clamped to 100 ms, so the detector arms as soon as
   * the baselines are seeded (0.2 s) and catches a release at 0.3 s. */
  rig_reset();
  shot_detect_config_t c;
  shot_detect_get_config(&c);
  c.min_hold_ms = 10u;
  shot_detect_set_config(&c);
  rig_hold(60);
  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));

  /* A 60 s request is clamped to 5 s: a release after 3 s of hold is not
   * armed yet, a release after 5.5 s is. */
  rig_reset();
  shot_detect_get_config(&c);
  c.min_hold_ms = 60000u;
  shot_detect_set_config(&c);
  rig_hold(3 * SIM_FS_HZ);
  rig_bump(4.5f);
  rig_hold(SIM_FS_HZ / 2);
  CHECK_EQ_INT(rig.n_events, 0);

  rig_hold(2 * SIM_FS_HZ);
  rig_bump(4.5f);
  rig_hold(SIM_FS_HZ / 2);
  CHECK_EQ_INT(rig.n_events, 2);
}

static void test_draw_motion_keeps_detector_idle(void) {
  rig_reset();
  rig_motion(2 * SIM_FS_HZ, 0.8); /* drawing the bow */
  CHECK(host_log_find_last("[IDLE]") != NULL);
  rig_bump(30.0f); /* a knock while the bow is still moving */
  rig_motion(SIM_FS_HZ / 2, 0.8);
  CHECK_EQ_INT(rig.n_events, 0);
}

static void test_start_in_motion_baseline_relaxes(void) {
  /* The session is opened while the bow is already being drawn. The hold
   * metric starts from zero, so the first samples read as "steady" and the
   * baselines are seeded from the motion. During the real hold that follows
   * they must relax, so that a release is still detected. */
  rig_reset();
  rig_motion(2 * SIM_FS_HZ, 0.8);
  float gbase_motion = 0.0f;
  CHECK(log_gyro_base(&gbase_motion));
  CHECK(gbase_motion > 0.5f); /* seeded from the draw, not from a hold */
  CHECK_EQ_INT(rig.n_events, 0);

  rig_hold(4 * SIM_FS_HZ);
  float gbase_hold = 0.0f;
  CHECK(log_gyro_base(&gbase_hold));
  CHECK(gbase_hold < 0.25f * gbase_motion);

  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));
  printf("    start in motion: gyro baseline %.3f rad/s after the draw, "
         "%.3f rad/s after 4 s of hold\n",
         (double)gbase_motion, (double)gbase_hold);
}

static void test_brief_wobble_keeps_hold(void) {
  rig_reset();
  rig_hold(100);
  /* A 0.1 s sight micro-correction: 0.4 rad/s, below the gyro trigger. */
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  const float wobble[3] = {0.4f, 0.0f, 0.0f};
  for (int i = 0; i < 20; i++) {
    rig_step(wobble, zero);
  }
  rig_hold(60);
  CHECK_EQ_INT(rig.n_events, 0);

  /* 0.4 s after the wobble began. Had the hold been dropped, the 0.5 s
   * minimum would have restarted and this release would be missed. */
  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));
}

static void test_long_unsteady_disarms_hold(void) {
  rig_reset();
  rig_hold(SIM_FS_HZ);
  /* 0.5 s of the same 0.4 rad/s motion is no wobble - it is a re-draw. */
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  const float redraw[3] = {0.4f, 0.0f, 0.0f};
  for (int i = 0; i < 100; i++) {
    rig_step(redraw, zero);
  }
  rig_hold(60); /* 0.3 s later the hold is not re-established yet */
  rig_bump(4.5f);
  rig_hold(SIM_FS_HZ / 2);
  CHECK_EQ_INT(rig.n_events, 0);
}

/* ---- refractory ------------------------------------------------------ */

static void test_refractory_after_confirmed(void) {
  rig_reset();
  rig_hold(2 * SIM_FS_HZ);
  rig_release(SIM_FS_HZ, &RECURVE); /* shot 1 */
  CHECK_EQ_INT(rig.n_events, 2);

  /* 1 s later: inside the 3 s refractory, a second transient is ignored. */
  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK_EQ_INT(rig.n_events, 2);

  /* 5.5 s after shot 1: refractory over and a new 0.5 s hold completed. */
  rig_hold(7 * SIM_FS_HZ / 2);
  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK_EQ_INT(rig.n_events, 4);
  CHECK(rig_is_pair(2, SHOT_EVT_CONFIRMED));
  CHECK(rig.events[2].onset_ms > rig.events[0].onset_ms + 3000u);
}

static void test_refractory_after_rejected(void) {
  rig_reset();
  rig_hold(2 * SIM_FS_HZ);
  rig_bump(4.5f);
  rig_hold(50); /* verdict (rejected) arrives ~0.2 s after the bump */
  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_REJECTED));

  /* Inside the 0.3 s refractory a second bump is ignored... */
  rig_bump(4.5f);
  rig_hold(60);
  CHECK_EQ_INT(rig.n_events, 2);

  /* ...and 1.5 s later a real release is caught again. */
  rig_hold(240);
  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK_EQ_INT(rig.n_events, 4);
  CHECK(rig_is_pair(2, SHOT_EVT_CONFIRMED));
}

/* ---- adaptive thresholds --------------------------------------------- */

static void test_adaptive_accel_threshold(void) {
  /* Quiet hold: the 1.5 m/s^2 floor applies and a 4.5 m/s^2 knock fires. */
  rig_reset();
  rig_hold(2 * SIM_FS_HZ);
  rig_bump(4.5f);
  rig_hold(SIM_FS_HZ / 2);
  CHECK(rig.n_events >= 1);

  /* Vibrating hold (e.g. wind on the stabiliser): the baseline rises, the
   * threshold follows it above the floor and the same knock is ignored. */
  rig_reset();
  rig.accel_noise = 2.0f;
  rig_hold(4 * SIM_FS_HZ);
  rig_bump(4.5f);
  rig_hold(SIM_FS_HZ / 2);
  CHECK_EQ_INT(rig.n_events, 0);
}

static void test_adaptive_gyro_threshold(void) {
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  const float turn[3] = {0.62f, 0.0f, 0.0f};

  /* Quiet hold: gyro floor 0.5 rad/s, a 0.62 rad/s turn fires. */
  rig_reset();
  rig_hold(2 * SIM_FS_HZ);
  for (int i = 0; i < 10; i++) {
    rig_step(turn, zero);
  }
  rig_hold(SIM_FS_HZ / 2);
  CHECK(rig.n_events >= 1);

  /* Trembling hold: gyro baseline ~0.1 rad/s lifts the threshold to
   * ~0.8 rad/s, so the same turn no longer fires. */
  rig_reset();
  rig.gyro_noise = 0.10f;
  rig_hold(4 * SIM_FS_HZ);
  for (int i = 0; i < 10; i++) {
    rig_step(turn, zero);
  }
  rig_hold(SIM_FS_HZ / 2);
  CHECK_EQ_INT(rig.n_events, 0);
}

static void test_raised_floors_suppress_trigger(void) {
  rig_reset();
  shot_detect_config_t c;
  shot_detect_get_config(&c);
  c.accel_trigger_floor_ms2 = 500.0f;
  c.gyro_trigger_floor_rad_s = 100.0f;
  shot_detect_set_config(&c);
  rig_hold(2 * SIM_FS_HZ);
  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK_EQ_INT(rig.n_events, 0);
}

/* ---- timing robustness ----------------------------------------------- */

static void test_batched_timestamps_do_not_break_fsm(void) {
  rig_reset();
  /* The sensor hub delivers reports in bursts of four: timestamps advance
   * 20 ms once per four samples. Windows are counted in samples, so the
   * verdict must still come 41 samples after the onset crossing. */
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  uint64_t base = rig.t_us;
  for (int i = 0; i < 2 * SIM_FS_HZ; i++) {
    rig.t_us = base + (uint64_t)(i / 4) * 20000ULL;
    rig_step(zero, zero);
  }
  long release_index = rig.index;
  uint32_t t_release = (uint32_t)((base + (uint64_t)(rig.index / 4) * 20000ULL) /
                                  1000ULL);
  for (int i = 0; i < SIM_FS_HZ; i++) {
    double t = (double)i / SIM_FS_HZ;
    float a[3];
    float g[3];
    sim_release_accel(t, RECURVE.a_peak, RECURVE.a_tau, a);
    sim_release_gyro(t, RECURVE.g_peak, RECURVE.g_tau, g);
    rig.t_us = base + (uint64_t)(rig.index / 4) * 20000ULL;
    rig_step(g, a);
  }
  CHECK_EQ_INT(rig.n_events, 2);
  CHECK(rig_is_pair(0, SHOT_EVT_CONFIRMED));
  long verdict = rig.event_index[1] - release_index;
  CHECK(verdict >= 40 && verdict <= 43);
  double onset_err_ms = (double)rig.events[1].onset_ms - (double)t_release;
  CHECK(onset_err_ms >= -40.0 && onset_err_ms <= 25.0);
}

static void test_onset_saturates_at_zero(void) {
  rig_reset();
  /* A compressed clock (10 us per sample, starting at 0) puts the onset
   * earlier than the 12 ms group-delay compensation: it must clamp to 0
   * instead of wrapping around. */
  rig.t_us = 0ULL;
  rig.dt_us = 10ULL;
  rig_hold(2 * SIM_FS_HZ);
  rig_release(SIM_FS_HZ, &RECURVE);
  CHECK_EQ_INT(rig.n_events, 2);
  CHECK_EQ_INT(rig.events[0].onset_ms, 0);
  CHECK_EQ_INT(rig.events[1].onset_ms, 0);
}

static void test_diagnostic_log_names_every_state(void) {
  rig_reset();
  bool seen_idle = false;
  bool seen_hold = false;
  bool seen_validating = false;
  bool seen_refractory = false;
  unsigned lines_before = host_log_count();

  /* 2 s draw, 3.95 s hold, release placed so that the 1 Hz diagnostic
   * (every 200th sample) lands inside the 0.2 s validation window. */
  for (int phase = 0; phase < 3; phase++) {
    int n = phase == 0 ? 400 : (phase == 1 ? 790 : 400);
    for (int i = 0; i < n; i++) {
      if (phase == 0) {
        rig_motion(1, 0.8);
      } else if (phase == 1) {
        rig_hold(1);
      } else {
        double t = (double)i / SIM_FS_HZ;
        float a[3];
        float g[3];
        sim_release_accel(t, RECURVE.a_peak, RECURVE.a_tau, a);
        sim_release_gyro(t, RECURVE.g_peak, RECURVE.g_tau, g);
        rig_step(g, a);
      }
      if (rig.index % SIM_FS_HZ == 0) {
        seen_idle |= host_log_find_last("[IDLE]") != NULL;
        seen_hold |= host_log_find_last("[HOLD]") != NULL;
        seen_validating |= host_log_find_last("[VALIDATING]") != NULL;
        seen_refractory |= host_log_find_last("[REFRACTORY]") != NULL;
      }
    }
  }
  CHECK(seen_idle);
  CHECK(seen_hold);
  CHECK(seen_validating);
  CHECK(seen_refractory);
  /* One diagnostic line per second plus the trigger and verdict lines. */
  CHECK(host_log_count() - lines_before >= (unsigned)(rig.index / SIM_FS_HZ));
}

int main(void) {
  printf("shot_detect host tests\n");
  RUN(test_defaults_and_null_safety);
  RUN(test_config_rejects_invalid_values);
  RUN(test_config_keeps_valid_values);
  RUN(test_quiet_hold_produces_no_events);
  RUN(test_clear_release_two_stage_event);
  RUN(test_gyro_only_release);
  RUN(test_bump_is_rejected);
  RUN(test_short_strong_impulse_confirmed_by_peak);
  RUN(test_slow_lowering_confirmed_by_ringdown);
  RUN(test_sub_debounce_blip_ignored);
  RUN(test_not_armed_before_min_hold);
  RUN(test_min_hold_is_clamped);
  RUN(test_draw_motion_keeps_detector_idle);
  RUN(test_start_in_motion_baseline_relaxes);
  RUN(test_brief_wobble_keeps_hold);
  RUN(test_long_unsteady_disarms_hold);
  RUN(test_refractory_after_confirmed);
  RUN(test_refractory_after_rejected);
  RUN(test_adaptive_accel_threshold);
  RUN(test_adaptive_gyro_threshold);
  RUN(test_raised_floors_suppress_trigger);
  RUN(test_batched_timestamps_do_not_break_fsm);
  RUN(test_onset_saturates_at_zero);
  RUN(test_diagnostic_log_names_every_state);
  return tu_summary("shot_detect");
}
