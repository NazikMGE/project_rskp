/* Host unit test for imu_profile.h — no ESP-IDF. Compile and run with gcc:
 *   gcc -I firmware/main firmware/test/host/imu_profile_test.c -o
 * /tmp/imu_profile_test && /tmp/imu_profile_test (run from the worktree root)
 */
#include <assert.h>
#include <stdio.h>

#include "imu_profile.h"

int main(void) {
  /* Session profile = current full streaming set. */
  imu_report_profile_t s = imu_profile_for(true);
  assert(s.rv_game_us == 20000);     /* 50 Hz fusion (decimation clock) */
  assert(s.cal_gyro_us == 5000);     /* 200 Hz */
  assert(s.linear_accel_us == 5000); /* 200 Hz */
  assert(s.accel_us == 0);           /* calibrated accel off in a session */
  assert(s.stability_us == 1000000); /* 1 Hz */

  /* Idle profile = lean keep-alive (fusion runs lean for accuracy + DCD). */
  imu_report_profile_t i = imu_profile_for(false);
  assert(i.rv_game_us == 0);           /* 50 Hz Game-RV frame OFF in idle */
  assert(i.cal_gyro_us == 100000);     /* 10 Hz gyro keep-alive (DCD) */
  assert(i.linear_accel_us == 100000); /* 10 Hz fused accel: accuracy + DCD */
  assert(i.accel_us == 0);             /* raw accel off in idle */
  assert(i.stability_us == 1000000);   /* 1 Hz stability -> auto-sleep */

  /* Invariants the power profile depends on: no 50 Hz frame and the 200 Hz DSP
   * rates are off, but fusion still runs lean (10 Hz linear accel) so the
   * health card reads the fused accuracy and the DCD-save gate is reachable. */
  assert(i.rv_game_us == 0);                     /* no 50 Hz fusion frame */
  assert(i.cal_gyro_us > s.cal_gyro_us);         /* idle gyro slower */
  assert(i.linear_accel_us > s.linear_accel_us); /* idle linAccel slower */
  assert(i.stability_us == s.stability_us);      /* motion source in both */

  printf("imu_profile_test: PASS\n");
  return 0;
}
