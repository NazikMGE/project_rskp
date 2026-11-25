# StabilityTracker shot-detection service.
#
# Variant-V4 archery release detector: a highpass-envelope detector on
# the 200 Hz linear-accel stream, an adaptive baseline, and a post-onset
# ring-down validation. Pure DSP + FSM, O(1) per sample — imu_task ticks
# it synchronously, no task or queue of its own.
idf_component_register(
    SRCS "src/shot_detect.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES log
)
