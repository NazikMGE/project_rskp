/**
 * @file control_example.c
 * @brief Control example: a modelled 24 s training fragment run through the
 *        unmodified shot detector and hold-stability score.
 *
 * The program reproduces the data path of imu_task (main/main.cpp) on the
 * host: every 200 Hz sample goes to shot_detect_tick() and to the moving
 * average; every fourth sample the decimated frame and the orientation
 * quaternion go to stability_tick(). The input is a deterministic model of
 * what the sensor on the bow sees:
 *
 *   draw -> hold -> release -> draw -> hold (+ a knock on the bow) ->
 *   release -> draw -> unsteady hold -> letting the bow down without a shot
 *
 * The moments of the four stimuli are known in advance, so the detector
 * output can be compared with the expected result. Files written to the
 * output directory (argv[1], default "."):
 *
 *   control_example_signal.csv     200 Hz input, gyro and linear accel
 *   control_example_stability.csv  50 Hz score and its three components
 *   control_example_events.csv     detector events
 *   control_example_summary.csv    expected vs. obtained, one row per stimulus
 *   control_example_log.txt        diagnostic journal of both modules
 *
 * Exit code 0 means every expectation of the control example was met.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "host_log.h"
#include "shot_detect/shot_detect.h"
#include "sim_signal.h"
#include "stability/stability.h"

/* ---- scenario -------------------------------------------------------- */

#define DURATION_S 24
#define N_SAMPLES (DURATION_S * SIM_FS_HZ)
#define DECIMATION 4        /* 200 Hz -> 50 Hz, as MA_WINDOW in main.cpp */
#define BOOT_US 60000000ULL /* the session starts 60 s after power-on */

/** Start of the unsteady (tired) hold, seconds. */
#define T_UNSTEADY 18.0

/** One stimulus of the scenario and what the detector must answer. */
typedef struct {
  const char *name;
  double t;                  /**< moment of the stimulus, s */
  shot_event_type_t verdict; /**< expected second-stage event */
  double onset_tol_ms;       /**< allowed |onset error|; < 0 = not checked */
  double latency_max_ms;     /**< allowed delay of the provisional event */
  bool weak;                 /**< expected peak below 5 m/s^2 */
} stimulus_t;

/*
 * The releases and the knock are impulsive, so the reported onset must fall
 * within one envelope window (25 ms) of the stimulus. Letting the bow down is
 * a gradual rotation without a defined onset: only the fact and the delay of
 * the event are checked.
 */
static const stimulus_t STIMULI[] = {
    {"release 1", 5.0, SHOT_EVT_CONFIRMED, 25.0, 20.0, false},
    {"knock", 12.0, SHOT_EVT_REJECTED, 25.0, 20.0, true},
    {"release 2", 14.0, SHOT_EVT_CONFIRMED, 25.0, 20.0, false},
    {"let-down", 21.0, SHOT_EVT_CONFIRMED, -1.0, 100.0, true},
};
#define N_STIMULI ((int)(sizeof(STIMULI) / sizeof(STIMULI[0])))

/** Intervals of large bow motion (the draw / carrying the bow). */
static const double MOTION[][2] = {
    {0.0, 2.0}, {7.0, 9.0}, {16.0, 18.0}, {21.5, 23.0}};

/** Raised-cosine gain: fades a motion interval in and out over 0.25 s. */
static double motion_gain(double t) {
  const double fade = 0.25;
  for (unsigned i = 0; i < sizeof(MOTION) / sizeof(MOTION[0]); i++) {
    double a = MOTION[i][0];
    double b = MOTION[i][1];
    if (t < a || t >= b) {
      continue;
    }
    double g = 1.0;
    if (t - a < fade && a > 0.0) { /* the fragment starts mid-draw */
      g = 0.5 - 0.5 * cos(SIM_PI * (t - a) / fade);
    }
    if (b - t < fade) {
      g = fmin(g, 0.5 - 0.5 * cos(SIM_PI * (b - t) / fade));
    }
    return g;
  }
  return 0.0;
}

/** Model of the sensor signals at time @p t (without noise). */
static void scenario(double t, float gyro[3], float accel[3]) {
  /* Aiming: postural sway (below 2 Hz) plus a 9 Hz physiological tremor;
   * the last hold is 3.5 times less steady than the first two. */
  double k = t >= T_UNSTEADY ? 3.5 : 1.0;
  gyro[0] = (float)(k * 0.012 * sin(2.0 * SIM_PI * 0.7 * t));
  gyro[1] = (float)(k * 0.008 * sin(2.0 * SIM_PI * 1.3 * t + 1.0));
  gyro[2] = (float)(k * 0.015 * sin(2.0 * SIM_PI * 9.0 * t));
  accel[0] = (float)(k * 0.10 * sin(2.0 * SIM_PI * 1.1 * t));
  accel[1] = (float)(k * 0.06 * cos(2.0 * SIM_PI * 0.6 * t));
  accel[2] = (float)(k * 0.08 * sin(2.0 * SIM_PI * 9.0 * t + 0.4));

  /* Large motion: drawing the bow, carrying it between shots. */
  double m = motion_gain(t);
  gyro[0] += (float)(m * 0.8 * sin(2.0 * SIM_PI * 1.3 * t + 0.5));
  gyro[1] += (float)(m * 0.8 * cos(2.0 * SIM_PI * 0.9 * t));
  gyro[2] += (float)(m * 0.4 * sin(2.0 * SIM_PI * 2.1 * t));
  accel[0] += (float)(m * 1.5 * sin(2.0 * SIM_PI * 1.1 * t));
  accel[1] += (float)(m * 1.0 * cos(2.0 * SIM_PI * 0.7 * t));

  /* Two releases: hand-shock ring-down and a forward roll of the bow. */
  float v[3];
  sim_release_accel(t - STIMULI[0].t, 75.0, 0.10, v);
  sim_add3(accel, v);
  sim_release_gyro(t - STIMULI[0].t, 2.5, 0.15, v);
  sim_add3(gyro, v);
  sim_release_accel(t - STIMULI[2].t, 70.0, 0.10, v);
  sim_add3(accel, v);
  sim_release_gyro(t - STIMULI[2].t, 2.3, 0.15, v);
  sim_add3(gyro, v);

  /* A knock on the bow: one 5 ms sample of 4.5 m/s^2. */
  if (fabs(t - STIMULI[1].t) < 0.5 / SIM_FS_HZ) {
    accel[0] += 4.5f;
  }

  /* Letting the bow down without shooting: a 0.65 rad/s rotation that
   * builds up in 50 ms and lasts until the carrying motion takes over. */
  double d = t - STIMULI[3].t;
  if (d >= 0.0 && d < 0.5) {
    gyro[0] += (float)(d < 0.05 ? 0.65 * d / 0.05 : 0.65);
  }
}

/* ---- data path of imu_task ------------------------------------------- */

/** Fixed-window moving average, as moving_avg_t / ma_push in main.cpp. */
typedef struct {
  float buf[DECIMATION];
  float sum;
  int idx;
  int count;
} moving_avg_t;

static void ma_push(moving_avg_t *m, float v) {
  m->sum += v - m->buf[m->idx];
  m->buf[m->idx] = v;
  m->idx = (m->idx + 1) % DECIMATION;
  if (m->count < DECIMATION) {
    m->count++;
  }
}

static float ma_value(const moving_avg_t *m) {
  return m->count > 0 ? m->sum / (float)m->count : 0.0f;
}

/**
 * @brief Advance the orientation quaternion [i,j,k,w] by the body rate
 *        @p w over @p dt - what the sensor's fusion does internally.
 */
static void quat_integrate(double q[4], const float w[3], double dt) {
  double hx = 0.5 * dt * w[0];
  double hy = 0.5 * dt * w[1];
  double hz = 0.5 * dt * w[2];
  double qi = q[0] + (q[3] * hx + q[1] * hz - q[2] * hy);
  double qj = q[1] + (q[3] * hy + q[2] * hx - q[0] * hz);
  double qk = q[2] + (q[3] * hz + q[0] * hy - q[1] * hx);
  double qw = q[3] - (q[0] * hx + q[1] * hy + q[2] * hz);
  double n = sqrt(qi * qi + qj * qj + qk * qk + qw * qw);
  q[0] = qi / n;
  q[1] = qj / n;
  q[2] = qk / n;
  q[3] = qw / n;
}

/* ---- recording and checking ------------------------------------------ */

#define MAX_EVENTS 32

typedef struct {
  shot_event_t ev;
  double t_tick; /**< time of the sample that produced the event, s */
} recorded_t;

static recorded_t events[MAX_EVENTS];
static int n_events;
static float score[N_SAMPLES / DECIMATION];
static int n_frames;
static int failures;

static const char *type_name(shot_event_type_t t) {
  switch (t) {
  case SHOT_EVT_PROVISIONAL:
    return "PROVISIONAL";
  case SHOT_EVT_CONFIRMED:
    return "CONFIRMED";
  case SHOT_EVT_REJECTED:
    return "REJECTED";
  default:
    return "?";
  }
}

/** Reported onset as seconds from the start of the fragment. */
static double onset_s(const shot_event_t *ev) {
  return ((double)ev->onset_ms - (double)(BOOT_US / 1000ULL)) / 1000.0;
}

static void expect(bool ok, const char *what) {
  printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
  if (!ok) {
    failures++;
  }
}

/** Mean score over the frames with t in [a, b). */
static double mean_score(double a, double b) {
  double acc = 0.0;
  int n = 0;
  for (int i = 0; i < n_frames; i++) {
    double t = (double)(i + 1) * DECIMATION / SIM_FS_HZ;
    if (t >= a && t < b) {
      acc += (double)score[i];
      n++;
    }
  }
  return n > 0 ? acc / n : 0.0;
}

/**
 * @brief Compare the event pair starting at @p first with stimulus @p s and
 *        append one row to the summary file.
 */
static void check_stimulus(const stimulus_t *s, int first, FILE *f_sum) {
  char msg[200];
  bool present = first + 1 < n_events &&
                 events[first].ev.type == SHOT_EVT_PROVISIONAL &&
                 events[first + 1].ev.type == s->verdict &&
                 events[first].ev.onset_ms == events[first + 1].ev.onset_ms;
  snprintf(msg, sizeof(msg), "%s: PROVISIONAL + %s with a common onset",
           s->name, type_name(s->verdict));
  expect(present, msg);
  double hold = mean_score(s->t - 1.0, s->t);
  if (!present) {
    fprintf(f_sum, "%s,%.3f,%s,missing,,,,,%.1f\n", s->name, s->t,
            type_name(s->verdict), hold);
    return;
  }
  const recorded_t *prov = &events[first];
  const recorded_t *verd = &events[first + 1];
  double err_ms = (onset_s(&prov->ev) - s->t) * 1000.0;
  double lat_ms = (prov->t_tick - s->t) * 1000.0;
  double verd_ms = (verd->t_tick - s->t) * 1000.0;

  if (s->onset_tol_ms >= 0.0) {
    snprintf(msg, sizeof(msg), "%s: onset error %+.0f ms within +/-%.0f ms",
             s->name, err_ms, s->onset_tol_ms);
    expect(fabs(err_ms) <= s->onset_tol_ms, msg);
  }
  snprintf(msg, sizeof(msg), "%s: provisional event after %.0f ms (<= %.0f ms)",
           s->name, lat_ms, s->latency_max_ms);
  expect(lat_ms >= 0.0 && lat_ms <= s->latency_max_ms, msg);
  snprintf(msg, sizeof(msg), "%s: peak %.2f m/s^2 is %s 5 m/s^2", s->name,
           (double)verd->ev.peak_mps2, s->weak ? "below" : "above");
  expect(s->weak ? verd->ev.peak_mps2 < 5.0f : verd->ev.peak_mps2 > 5.0f, msg);

  fprintf(f_sum, "%s,%.3f,%s,%s,%.0f,%.0f,%.0f,%.2f,%.1f\n", s->name, s->t,
          type_name(s->verdict), type_name(verd->ev.type), err_ms, lat_ms,
          verd_ms, (double)verd->ev.peak_mps2, hold);
}

static FILE *open_out(const char *dir, const char *name) {
  char path[1024];
  snprintf(path, sizeof(path), "%s/%s", dir, name);
  FILE *f = fopen(path, "w");
  if (f == NULL) {
    fprintf(stderr, "cannot write %s\n", path);
  }
  return f;
}

int main(int argc, char **argv) {
  const char *dir = argc > 1 ? argv[1] : ".";
  FILE *f_sig = open_out(dir, "control_example_signal.csv");
  FILE *f_stab = open_out(dir, "control_example_stability.csv");
  FILE *f_evt = open_out(dir, "control_example_events.csv");
  FILE *f_sum = open_out(dir, "control_example_summary.csv");
  FILE *f_log = open_out(dir, "control_example_log.txt");
  if (f_sig == NULL || f_stab == NULL || f_evt == NULL || f_sum == NULL ||
      f_log == NULL) {
    return 2;
  }

  host_log_reset();
  host_log_mirror(f_log);
  shot_detect_init();
  stability_init();

  moving_avg_t gyro_ma[3];
  moving_avg_t accel_ma[3];
  memset(gyro_ma, 0, sizeof(gyro_ma));
  memset(accel_ma, 0, sizeof(accel_ma));
  double q[4] = {0.0, 0.0, 0.0, 1.0};
  sim_rng_t rng;
  sim_rng_seed(&rng, 2026u);

  fprintf(f_sig, "t_s,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n");
  fprintf(f_stab, "t_s,score,instability,rms_omega,rms_accel_lf,rms_disp\n");

  for (int i = 0; i < N_SAMPLES; i++) {
    double t = (double)i / SIM_FS_HZ;
    uint64_t t_us = BOOT_US + (uint64_t)i * SIM_DT_US;

    /* Sensor model: the scenario plus measurement noise. */
    float gyro[3];
    float accel[3];
    float noise[3];
    scenario(t, gyro, accel);
    sim_noise3(&rng, 0.02f, noise);
    sim_add3(gyro, noise);
    sim_noise3(&rng, 0.12f, noise);
    sim_add3(accel, noise);
    quat_integrate(q, gyro, 1.0 / SIM_FS_HZ);
    fprintf(f_sig, "%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", t, gyro[0], gyro[1],
            gyro[2], accel[0], accel[1], accel[2]);

    /* 200 Hz path: decimation filter and the shot detector. */
    for (int a = 0; a < 3; a++) {
      ma_push(&gyro_ma[a], gyro[a]);
      ma_push(&accel_ma[a], accel[a]);
    }
    shot_event_t ev;
    if (shot_detect_tick(t_us, gyro, accel, &ev) && n_events < MAX_EVENTS) {
      events[n_events].ev = ev;
      events[n_events].t_tick = t;
      n_events++;
    }

    /* 50 Hz path: one decimated frame per four samples. */
    if ((i + 1) % DECIMATION == 0) {
      const float st_gyro[3] = {ma_value(&gyro_ma[0]), ma_value(&gyro_ma[1]),
                                ma_value(&gyro_ma[2])};
      const float st_accel[3] = {ma_value(&accel_ma[0]), ma_value(&accel_ma[1]),
                                 ma_value(&accel_ma[2])};
      const float st_quat[4] = {(float)q[0], (float)q[1], (float)q[2],
                                (float)q[3]};
      stability_output_t st;
      stability_tick(st_gyro, st_accel, st_quat, &st);
      score[n_frames++] = st.score;
      fprintf(f_stab, "%.3f,%.3f,%.5f,%.5f,%.5f,%.5f\n",
              (double)(i + 1) / SIM_FS_HZ, st.score, st.instability,
              st.rms_omega, st.rms_accel_lf, st.rms_disp);
    }
  }
  host_log_mirror(NULL);

  /* ---- detector events ---- */
  printf("control example: %d samples at %d Hz, %d frames at %d Hz\n",
         N_SAMPLES, SIM_FS_HZ, n_frames, SIM_FS_HZ / DECIMATION);
  printf("detector events:\n");
  fprintf(f_evt, "n,t_tick_s,type,onset_ms,onset_s,peak_mps2\n");
  for (int i = 0; i < n_events; i++) {
    const recorded_t *r = &events[i];
    printf("  %d  t=%7.3f s  %-11s  onset=%u ms (%.3f s)  peak=%.2f m/s^2\n",
           i + 1, r->t_tick, type_name(r->ev.type), (unsigned)r->ev.onset_ms,
           onset_s(&r->ev), (double)r->ev.peak_mps2);
    fprintf(f_evt, "%d,%.3f,%s,%u,%.3f,%.3f\n", i + 1, r->t_tick,
            type_name(r->ev.type), (unsigned)r->ev.onset_ms, onset_s(&r->ev),
            (double)r->ev.peak_mps2);
  }

  /* ---- expectations of the control example ---- */
  printf("checks:\n");
  expect(n_events == 2 * N_STIMULI,
         "exactly 8 events: four stimuli, two events each");
  fprintf(f_sum, "stimulus,t_s,expected,obtained,onset_error_ms,"
                 "provisional_delay_ms,verdict_delay_ms,peak_mps2,"
                 "hold_score\n");
  for (int i = 0; i < N_STIMULI; i++) {
    check_stimulus(&STIMULI[i], 2 * i, f_sum);
  }

  double s_good = mean_score(STIMULI[0].t - 1.0, STIMULI[0].t);
  double s_tired = mean_score(STIMULI[3].t - 1.0, STIMULI[3].t);
  double s_draw = mean_score(7.5, 8.5);
  printf("stability score: steady hold %.1f, unsteady hold %.1f, draw %.1f\n",
         s_good, s_tired, s_draw);
  expect(s_good > s_tired + 10.0,
         "score of the steady hold exceeds the unsteady one by over 10");
  expect(s_draw < 20.0, "score during the draw is below 20");

  fclose(f_sig);
  fclose(f_stab);
  fclose(f_evt);
  fclose(f_sum);
  fclose(f_log);
  printf("control example: %s\n", failures == 0 ? "PASSED" : "FAILED");
  return failures == 0 ? 0 : 1;
}
