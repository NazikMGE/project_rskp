# StabilityTracker pre-shot ring buffer service.
#
# A circular buffer of recent 200 Hz IMU samples in PSRAM; imu_task
# appends every sample and marks the position on a detected release,
# freezing the aiming window for later post-shot analysis / transfer.
idf_component_register(
    SRCS "src/preshot.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES heap log
)
