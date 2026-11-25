#pragma once

#include <math.h>
#include <stdint.h>

/**
 * @file sim_signal.h
 * @brief Deterministic synthetic IMU signals for the host tests.
 *
 * Everything is reproducible: the "noise" is a fixed-seed linear congruential
 * generator, so a test run gives bit-identical results on every machine.
 * The release amplitude follows the field recordings of the prototype: a
 * clear recurve release peaks at 69-83 m/s^2, a weak event stays below
 * 5 m/s^2. The hold amplitudes are assumed - a few hundredths of rad/s and
 * tenths of m/s^2, the order of the default baselines of the stability
 * module.
 */

#define SIM_PI 3.14159265358979323846
#define SIM_FS_HZ 200         /**< detector sample rate */
#define SIM_DT_US 5000ULL     /**< one 200 Hz sample period */
#define SIM_T0_US 1000000ULL  /**< timestamp of the first sample */

/** @brief Fixed-seed pseudo-random source. */
typedef struct {
  uint32_t state;
} sim_rng_t;

static inline void sim_rng_seed(sim_rng_t *r, uint32_t seed) {
  r->state = seed;
}

/** @brief Next pseudo-random value, uniform in [-1, 1). */
static inline float sim_rng_uniform(sim_rng_t *r) {
  r->state = r->state * 1664525u + 1013904223u; /* Numerical Recipes LCG */
  return (float)(r->state >> 8) / 8388608.0f - 1.0f;
}

/** @brief Fill @p v with independent uniform noise of amplitude @p amp. */
static inline void sim_noise3(sim_rng_t *r, float amp, float v[3]) {
  v[0] = amp * sim_rng_uniform(r);
  v[1] = amp * sim_rng_uniform(r);
  v[2] = amp * sim_rng_uniform(r);
}

/**
 * @brief Release transient of a bow on the linear-accel channel.
 *
 * Three damped sinusoids of different frequencies - the ring-down of the
 * limbs and the riser after the string leaves the fingers.
 *
 * @param t_s   time since the release, seconds (negative -> zero output)
 * @param peak  amplitude scale, m/s^2
 * @param tau_s decay time constant, seconds
 */
static inline void sim_release_accel(double t_s, double peak, double tau_s,
                                     float v[3]) {
  if (t_s < 0.0) {
    v[0] = v[1] = v[2] = 0.0f;
    return;
  }
  double env = peak * exp(-t_s / tau_s);
  v[0] = (float)(env * sin(2.0 * SIM_PI * 38.0 * t_s));
  v[1] = (float)(0.6 * env * sin(2.0 * SIM_PI * 27.0 * t_s + 0.7));
  v[2] = (float)(0.4 * env * sin(2.0 * SIM_PI * 55.0 * t_s + 1.9));
}

/**
 * @brief Bow-rotation transient on the gyro channel (forward roll of the bow
 *        after the release), a damped 12 Hz oscillation.
 */
static inline void sim_release_gyro(double t_s, double peak, double tau_s,
                                    float v[3]) {
  if (t_s < 0.0) {
    v[0] = v[1] = v[2] = 0.0f;
    return;
  }
  double env = peak * exp(-t_s / tau_s);
  v[0] = (float)(env * cos(2.0 * SIM_PI * 12.0 * t_s));
  v[1] = (float)(0.3 * env * sin(2.0 * SIM_PI * 12.0 * t_s));
  v[2] = 0.0f;
}

static inline float sim_mag3(const float v[3]) {
  return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static inline void sim_add3(float acc[3], const float v[3]) {
  acc[0] += v[0];
  acc[1] += v[1];
  acc[2] += v[2];
}
