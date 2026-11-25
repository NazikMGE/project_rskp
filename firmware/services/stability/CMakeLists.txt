# StabilityTracker hold-stability service.
#
# Frequency-decomposed hold-stability score: RMS angular rate (tremor),
# RMS low-frequency linear accel (sway), and RMS angular displacement
# (aim drift), normalised per archer and mapped to 0..100. Pure DSP,
# O(1) per frame — imu_task ticks it from the 50 Hz path.
idf_component_register(
    SRCS "src/stability.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES log
)
