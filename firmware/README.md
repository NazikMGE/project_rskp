# StabilityTracker Firmware

ESP-IDF firmware for the StabilityTracker v1 device — a XIAO ESP32-S3
Plus reading a BNO085 IMU over SPI and streaming fused orientation,
gyro, and linear-acceleration samples to the mobile app over BLE at
50 Hz.

## Hardware

- **MCU board**: Seeed XIAO ESP32-S3 Plus (16 MB flash, BLE-only radio)
- **IMU**: CEVA BNO085, SPI mode, SCLK 3 MHz
- **Power**: 1S LiPo, voltage sensed through an external 220k/220k
  divider on D5
- **Input**: one tactile button on D0 (long-press → deep sleep)

Pin assignments are the single source of truth in
[`boards/board/include/board/pinmap.h`](boards/board/include/board/pinmap.h).

## Layout

| Path                   | Role                                                |
| ---------------------- | --------------------------------------------------- |
| `main/`                | App entry, task setup, IMU → BLE pump               |
| `boards/board/`        | XIAO ESP32-S3 Plus pin map (header-only)            |
| `drivers/battery/`     | 1S LiPo monitor (ADC + OCV curve)                   |
| `drivers/button/`      | Tactile button input                                |
| `drivers/calibration/` | NVS-persisted BNO085 DCD-save policy                |
| `drivers/power/`       | Deep-sleep entry / wake-cause detection             |
| `drivers/supervisor/`  | Fault watchdog — bounded reboot, degraded fallback  |
| `services/ble/`        | NimBLE host wrapper — advertising, GATT, IMU notify |
| `vendor/esp32_BNO08x/` | BNO08x driver (git submodule)                       |

Components outside `main/` are surfaced via `EXTRA_COMPONENT_DIRS` in
the top-level `CMakeLists.txt`.

## Build

Requires ESP-IDF v5.x. Clone the BNO08x submodule first:

```sh
git submodule update --init --recursive
```

Then, from this directory:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

### sdkconfig

`sdkconfig` is generated and git-ignored; the checked-in source of
truth is `sdkconfig.defaults`. ESP-IDF applies `sdkconfig.defaults`
only for keys absent from an existing `sdkconfig`, so a stale
`sdkconfig` can silently override a later default. If a setting (e.g.
flash size) does not match `sdkconfig.defaults`, delete `sdkconfig`
and `sdkconfig.old`, then re-run `idf.py build`.

## Reference

- BNO085 / SH-2 reference manual: `docs/datasheets/bno085/`
- XIAO ESP32-S3 Plus pinout & schematic: `docs/datasheets/xiao-esp32s3-plus/`
- BLE wire protocol: `docs/ble/sensor-protocol.md`
