# StabilityTracker -- BLE sensor protocol (v3)

**Status:** draft, phase 1 + linear-accel + calibration-status + shot-event + metrics + preshot-transfer + hardware-tuning extensions.
**Last updated:** 2026-05-26.

This document defines the GATT contract between the StabilityTracker
sensor (ESP32-S3 firmware + BNO085 IMU) and the mobile app. Fixed-binary
frames, explicit byte layouts -- no protobuf, no JSON. The host has hard
real-time constraints (50 Hz @ 20 ms inter-sample budget, on-chip DSP)
that fixed structs satisfy with minimal CPU and zero allocator pressure.

The protocol is mirrored on both sides:

- **Firmware** -- `firmware/services/ble/`: `ble_gatt.c` encodes the
  frames and serves the GATT characteristics, `ble.c` runs the NimBLE
  host.
- **Mobile** -- `mobile/app/lib/features/device/data/sensor_protocol.dart`
  (parses frames into `SensorSample`).

If the two diverge, the parser rejects the frame with a `SensorFrameException`
and Sentry receives the diagnostic event. **Versioning** is by characteristic
UUID -- incompatible changes get a new UUID, not a new layout under the same
one.

## Service & characteristics

### Custom service: `5d3ace00-7e9f-4b1a-8d2c-f6a8b3e9c7f1`

Advertised on connectable, undirected advertisements at 100 ms interval
with the local-name field set to `ST-XXXX` where `XXXX` is the last two
bytes of the device's BLE MAC in uppercase hex (e.g. `ST-7C2A`). The
mobile app's scanner filters by this service UUID and the local-name
prefix `ST-` so it ignores unrelated devices in the same room.

| UUID                                   | Properties      | Purpose                          |
|----------------------------------------|-----------------|----------------------------------|
| `5d3ace04-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Notify          | IMU stream -- 25 B sample @ 50 Hz |
| `5d3ace02-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Read            | Device info (firmware ver, hw id, serial) |
| `5d3ace05-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Read            | Sensor health -- 8 B live accuracy + persisted DCD state |
| `5d3ace03-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Write           | Session control -- SESSION_START / SESSION_STOP / SHOT_TRANSFER_START |
| `5d3ace06-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Notify          | Shot events -- 9 B release-detection event |
| `5d3ace07-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Notify          | Stability metrics -- 26 B hold-quality frame @ 50 Hz |
| `5d3ace08-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Notify          | Preshot bulk transfer -- 8 / 16 / 28 B frames around a detected shot |
| `5d3ace09-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Read + Write    | Shot-detect tuning -- 24 B runtime config for the V4 release detector |
| `5d3ace0a-7e9f-4b1a-8d2c-f6a8b3e9c7f1` | Read + Write    | Stability-score tuning -- 50 B runtime config for the hold-quality DSP |

**Versioning.** The rule from v1.0: an incompatible wire change gets a
new characteristic UUID, not a changed layout under the same one -- an
outdated mobile build then fails service discovery and disconnects
cleanly instead of misparsing. Pre-v1, with no installed base and the
firmware and app moving together, incompatible changes are made in
place under the existing UUID. (The retired v1 IMU characteristic
`5d3ace01-...` predates this and kept its own UUID.)

The characteristic UUIDs differ from the service UUID
`5d3ace00-7e9f-4b1a-8d2c-f6a8b3e9c7f1` in only the 4th byte -- `02`-`07`
versus the service's `00`. A new characteristic takes the next free byte.

### Standard SIG service: `0000180f-0000-1000-8000-00805f9b34fb` (Battery Service)

Exposes a single characteristic:

| UUID                                   | Properties     | Purpose                         |
|----------------------------------------|----------------|---------------------------------|
| `00002a19-0000-1000-8000-00805f9b34fb` | Read + Notify  | Battery level -- uint8 percent   |

Standard SIG-assigned UUIDs (Battery Service `0x180F`, Battery Level
`0x2A19`) so any generic BLE inspector / Android system battery
indicator can read it.

## Connection lifecycle

| Step | Direction      | Action                                                |
|------|----------------|-------------------------------------------------------|
| 1    | firmware -> air | Advertises (`ST-XXXX`, service UUID, 100 ms interval).|
| 2    | mobile -> fw    | Connects; default ATT MTU = 23 (payload 20 B).        |
| 3    | mobile -> fw    | Discovers services + characteristics.                 |
| 4    | mobile -> fw    | Reads `device_info` (firmware version + hardware id). |
| 5    | mobile -> fw    | Subscribes to `imu_stream`, `shot_events`, `stability_metrics`, `preshot_transfer` (CCCD = 0x0001 each). |
| 6    | mobile -> fw    | Subscribes to `battery_level`.                        |
| 7    | mobile -> fw    | Writes `SESSION_START` to `control` to open a session. |
| 8    | firmware -> mob | Notifies `imu_stream` + `stability_metrics` @ 50 Hz while the session is open; `shot_events` on each detected release. |
| 9    | firmware -> mob | Notifies `battery_level` on every ≥1 % change, throttled to once / 5 s. |

**MTU negotiation is required.** The 25-byte IMU frame exceeds the
20-byte payload of the default ATT MTU (23). Both ends now drive the
exchange: the firmware advertises a preferred ATT MTU of 247 and
**proactively initiates** an MTU exchange on connect (relying on the
central to ask first is fragile -- a peer that forgets leaves us
truncating frames); the mobile client also requests 247 on its side
during link bring-up. Whichever side wins the race, the negotiated MTU
is the minimum of the two preferred values, which clears the 28-byte
floor with margin. Frames travel in a single ATT_HANDLE_VALUE_NTF, no
fragmentation.

**Reconnect policy:** the firmware advertises continuously until a
connection is established and resumes advertising on disconnect.

Phase 1 mobile side handles **in-session, foreground-only** reconnect:
when the platform-side link drops (`SensorConnection.isConnected`
emits `false`) the repository transitions `Connected -> Reconnecting`
and re-attempts up to three times with 2 / 4 / 8 s backoff. On
exhaustion the screen falls back to `Disconnected` and the user can
restart the scan. The user can cancel the cycle at any backoff or
mid-handshake via the Cancel button on the reconnect view.

Phase 2 future:

- **Cross-session persistence** -- cache the last-seen `bleId` in
  the local SQLite store so a cold-start with no scan still tries
  the previously paired sensor first.
- **Android `autoConnect=true`** -- bonded sensors reconnect in the
  background when the user re-enters proximity, even if the app is
  not foregrounded.
- **iOS background-BLE** -- the app would need to declare the
  `bluetooth-central` background mode (currently not declared) and
  handle CoreBluetooth state restoration. Out of scope for v1.

## IMU sample frame -- 25 bytes, little-endian

The `imu_stream` characteristic notifies one frame per sample at 50 Hz.

```
offset  size  type      field             encoding / scale
------  ----  --------  ----------------  -----------------------------------
0       4     uint32    timestamp_ms      monotonic ms since firmware boot;
                                          wraps every ~49.7 days (sessions
                                          are far shorter, no anti-wrap logic)
4       2     int16     quat_x            Q1.14 fixed-point: divide by 16384
6       2     int16     quat_y            (i.e. /2^14) -> component in [-1, +1]
8       2     int16     quat_z            game rotation vector (no magnetometer)
10      2     int16     quat_w
12      2     int16     gyro_x            Q12.4 fixed-point: divide by 16
14      2     int16     gyro_y            -> angular velocity in deg/s, range
16      2     int16     gyro_z            ±2048 dps (BNO085 max ±2000)
18      2     int16     accel_x           Q8.8 fixed-point: divide by 256
20      2     int16     accel_y           -> linear acceleration in m/s²
22      2     int16     accel_z           (gravity removed on-chip), ±128 m/s²
24      1     uint8     seq               wraparound 0..255, see Sequence counter
```

Total: 25 bytes per frame. Requires ATT MTU ≥ 28 (3-byte L2CAP/ATT
header + 25-byte payload); the mobile client negotiates up to 247
on connect, which all modern phones honor.

Per-sample sensor accuracy is **not** carried in the frame -- it is read
on demand from the sensor-health characteristic instead; streaming it at
50 Hz served no purpose for the DSP.

### Quaternion (Game Rotation Vector)

BNO085 fuses the accelerometer + gyroscope on-chip and outputs a
unit quaternion describing the sensor's orientation. We use the **game
rotation vector** report (SH-2 sensor ID 0x08), which is mag-free --
absolute heading is irrelevant for archery and the mag would only add
drift / disturbance noise.

Component order matches the SH-2 protocol:

- `quat_x` = i (vector x)
- `quat_y` = j (vector y)
- `quat_z` = k (vector z)
- `quat_w` = real (scalar)

Decoded value = `int16_le / 16384.0`. The unit-quaternion invariant
`x² + y² + z² + w² ≈ 1` holds within ~1e-3 of rounding noise; the
parser does **not** enforce it (`BNO085` guarantees normalisation in
firmware), but the DSP layer can use it as a sanity check.

### Gyro (calibrated angular velocity)

BNO085 calibrated gyroscope report (SH-2 sensor ID 0x02). Components
in the sensor's body frame:

- `gyro_x` = ω about x-axis (deg/s)
- `gyro_y` = ω about y-axis
- `gyro_z` = ω about z-axis

Decoded value = `int16_le / 16.0`. The Q12.4 representation gives
0.0625 dps resolution and ±2048 dps full scale -- physiological tremor
is in the fractions-of-dps to single-dps range, so the headroom serves
sharp release-rotation transients (~50-150 dps spikes).

Why both quaternion and gyro: the quaternion gives orientation /
"where the bow is pointing" which the DSP layer projects onto the
target plane; the gyro gives the direct rotation-rate signal that
captures tremor frequency content (4-12 Hz physiological band)
without the noise of numerically differentiating the quaternion.

### Linear acceleration

BNO085 linear-acceleration report (SH-2 sensor ID 0x04). Components
are in the sensor's body frame, **with gravity removed by on-chip
fusion** -- at rest the values cluster near zero, not near 1 g.

- `accel_x` = a along x-axis (m/s²)
- `accel_y` = a along y-axis
- `accel_z` = a along z-axis

Decoded value = `int16_le / 256.0`. The Q8.8 representation gives
~3.9 mg resolution and ±128 m/s² (≈ ±13 g) full scale. BNO085's own
linear-accel range is ±156 m/s²; we clamp at ±128 so a release-shock
spike saturates the int16 rather than wrapping. For archery use the
typical signal sits sub-1 m/s² with the release transient touching
3-6 m/s² -- well inside the encoded range.

Why include linear accel alongside the gyro: tremor analysis cares
about both rotational (gyro) and translational (linear-accel) drift
of the bow. The gyro tells us how the bow is *rotating* under hand
tremor; the linear-accel tells us how the bow is *translating* --
arm sway, draw-hand pull-through, post-release recoil. Combined,
they let the DSP layer separate "wrist tremble" from "whole-bow
sway", which discriminate different coaching cues.

### Sequence counter (byte 24)

Wraparound counter incremented on every emitted frame, regardless of
content. The mobile parser tracks the previous value and reports a
gap when `(current - previous) mod 256 != 1`.

Why: BLE notifications can be silently dropped -- radio interference,
GATT queue overflow under host-OS scheduling, etc. Without an
in-frame counter, lost samples are invisible to the DSP layer (the
`timestamp_ms` field has the same problem since gaps in time look
identical to the host-clock skew). The 8-bit width wraps every 256
samples = 5.12 s at 50 Hz, plenty for live gap detection; longer
outages get reported as multi-second gaps which are also visible in
the timestamp delta.

## Device info characteristic -- variable length, read-once

Read by the mobile app immediately after connect, before subscribing
to the IMU stream. Encodes:

```
offset  size  type      field
------  ----  --------  -----------------------------------------
0       4     uint32    firmware_version_packed  // major.minor.patch.build
                                                 // 8 bits each, big-endian
                                                 // (i.e. 0x01020304 = v1.2.3.4)
4       6     uint8[6]  hardware_id              // 48-bit MAC in BE
10      ?     utf8      serial_number            // null-terminated, max 32 B
```

Why packed-big-endian for `firmware_version`: easier to read in
logcat / Sentry tags as a hex dump without endian gymnastics. The
rest is little-endian for consistency with the IMU stream -- the
exception is intentional and documented.

## Sensor health characteristic -- 8 bytes, read

Read by the mobile app on connect and on the device screen to learn the
sensor's current condition without subscribing to the IMU stream -- two
kinds of data in one read: the **live** IMU accuracy right now, and the
**persisted** DCD-save bookkeeping.

```
offset  size  type    field           encoding
------  ----  ------  --------------  ----------------------------------------
0       1     uint8   schema_version  currently 0x02
1       1     uint8   flags           bit 0: imu_online (0 = IMU disabled,
                                      e.g. supervisor degraded mode);
                                      bit 1: dcd_ever_saved; bits 2-7 reserved
2       1     uint8   accel_accuracy  live accel accuracy 0-3; 0xFF = warming up
3       1     uint8   gyro_accuracy   live gyro accuracy 0-3; 0xFF = warming up
4       4     uint32  dcd_save_count  little-endian; count of successful saves
```

Total: 8 bytes, fits inside the default ATT MTU.

The live `accel_accuracy` / `gyro_accuracy` answer "are the IMU's reports
trustworthy right now" -- the device screen surfaces this as the sensor
warms up after a cold start. `imu_online` is 0 when the IMU has been shut
down (supervisor degraded mode), which the accuracy fields alone cannot
distinguish from "still warming up".

**`dcd_save_count` as a freshness key.** The sensor has no real-time
clock, so it cannot timestamp a calibration. The mobile app reads
`dcd_save_count` on connect; when it has increased versus the previously
stored value, the device re-saved its calibration since last seen, and
the app stamps its own "last calibrated" record with wall-clock time.
The count is monotonic and only advances on a successful DCD save.

The DCD (Dynamic Calibration Data) itself lives in the BNO085's own flash
and is opaque to the host -- this characteristic exposes only the
firmware's bookkeeping about it. The firmware keeps the characteristic
current -- live accuracy and the save count are pushed as they change --
so any read returns the present state.

## Battery level characteristic -- uint8 percent

Standard BLE SIG. Single-byte value 0-100 representing remaining
battery percentage. Notified on every ≥1 % change, throttled by the
firmware to at most one notify per 5 s -- battery % moves slowly so
the throttle rarely engages; it mainly bounds notify traffic if the
reading flaps across a 1 % boundary, and 5 s keeps the device-screen
battery indicator responsive.

When the battery is critically low (<5 %), the firmware notifies once
on transition; the mobile app surfaces a banner and the user is
expected to charge. The firmware does **not** disconnect on critical
battery -- the user may still want a brief session.

## Control characteristic -- session control

`5d3ace03-...`, **Write With Response**, single-byte opcode payload. A
session command is infrequent and must not be silently lost, so the
acknowledged write confirms to the mobile app that the device received
it.

The IMU stream is **session-gated**: after connecting and subscribing,
no `imu_stream` notifications flow until the mobile app writes
`SESSION_START`. An idle connection -- e.g. the device screen open with
no training under way -- therefore costs no 50 Hz traffic or radio
airtime.

| Opcode    | Name                  | Action                                                                                                                                                  |
|-----------|-----------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0x00      | `SESSION_STOP`        | End the session; the IMU and stability-metrics streams stop.                                                                                            |
| 0x01      | `SESSION_START`       | Open a session; the IMU and stability-metrics streams start. The firmware also freezes BNO085 dynamic calibration for the session so a mid-session recalibration step cannot perturb the analysed signal. |
| 0x02      | `CALIBRATE`           | Reserved -- BNO085 calibration trigger.                                                                                                                 |
| 0x03      | `RESET`               | Reserved -- soft firmware reset.                                                                                                                        |
| 0x10      | `SHOT_TRANSFER_START` | Request a preshot bulk transfer for the most recently marked shot; see the preshot characteristic.                                                       |
| other     | reserved              | Unknown opcodes are acknowledged at the ATT layer but ignored, with a UART log line.                                                                     |

The session also ends on disconnect; BNO085 dynamic calibration is
re-enabled on `SESSION_STOP` or disconnect. Unknown opcodes are
acknowledged at the ATT layer but ignored, and logged over UART.

## Shot event characteristic -- 9 bytes, notify

`5d3ace06-...`, **Notify**. The firmware runs an on-device archery
release detector (variant V4: a highpass-envelope detector on the
200 Hz linear-accel stream, gated by a sustained aiming hold and
confirmed by a post-onset ring-down check). Each detected event is
notified as one 9-byte frame.

```
offset  size  type    field           encoding
------  ----  ------  --------------  ----------------------------------------
0       1     uint8   schema_version  currently 0x01
1       1     uint8   event_type      0 provisional, 1 confirmed, 2 rejected
2       4     uint32  onset_ms        release-onset timestamp, little-endian;
                                      same monotonic ms clock as the IMU
                                      frame's timestamp_ms
6       2     uint16  peak_centi      peak release envelope, little-endian, in
                                      centi-m/s² (m/s² × 100); 0 for a
                                      provisional event
8       1     uint8   seq             wraparound 0..255
```

Total: 9 bytes, fits inside the default ATT MTU.

**Two-stage events.** A release is reported twice: a `provisional`
event within ~20 ms of the onset -- low latency, for an instant UI
marker -- then, ~230 ms later once the ring-down window has been
analysed, a `confirmed` or `rejected` event for the same shot. The
provisional event and its follow-up carry the **same `onset_ms`**; the
app pairs them on that field and either keeps or retracts the marker.

The `seq` counter is monotonic across all emitted events for gap
detection (same contract as the IMU stream's byte 24); it is **not**
used to pair provisional with confirmed -- `onset_ms` is.

**Not session-gated.** Unlike the IMU stream, shot events flow whenever
a central is subscribed, independent of `SESSION_START` / `SESSION_STOP`
-- the detector runs continuously and a shot is rare enough to always
be worth delivering.

## Stability metrics characteristic -- 26 bytes, notify

`5d3ace07-...`, **Notify**. The firmware's hold-stability score and the
raw components it is built from, notified at 50 Hz — one frame per IMU
sample, so the app can align the metrics with the IMU stream 1:1.

```
offset  size  type     field           encoding
------  ----  -------  --------------  ---------------------------------------
0       1     uint8    schema_version  currently 0x01
1       1     uint8    seq             wraparound 0..255, gap detection
2       4     uint32   timestamp_ms    firmware monotonic ms, IMU-frame clock
6       4     float32  score           smoothed hold-stability score, 0..100
10      4     float32  instability     composite instability index I
14      4     float32  rms_omega       rate-term RMS, rad/s
18      4     float32  rms_accel_lf    sway-term RMS, m/s^2
22      4     float32  rms_disp        drift-term RMS, rad
```

Total: 26 bytes. All multi-byte fields are little-endian; the five
metric values are IEEE-754 `float32` (sent raw, not Q-format -- the
tuning UI wants the exact values; 26 B at 50 Hz is ~1.3 KB/s, light).

`score` is the figure on the live stability gauge. `instability` and
the three `rms_*` terms are what the score is composed of; they are
carried so the app's experiment / tuning UI can show and re-weight the
raw inputs without a firmware change.

**Live-feedback semantic.** The three `rms_*` values are
single-pole **EMA-weighted** RMS estimates of the squared input, not
rectangular-window RMS -- the choice gives a 100-200 ms step response
versus the 1-2 s a window-RMS the size of the previous implementation
would impose, which broke the live biofeedback loop. The trade-off is
a noisier per-frame number; mobile-side post-hoc analysis recomputes
strict windowed RMS from the recorded raw 50 Hz IMU stream when
research-grade statistics are needed. See
`firmware/services/stability/include/stability/stability.h` for the
filter constants.

**Session-gated.** Like the IMU stream, metrics flow only between
`SESSION_START` and `SESSION_STOP` -- an idle connection costs no
metrics traffic.

## Preshot transfer characteristic -- 8 / 16 / 28 bytes, notify

`5d3ace08-...`, **Notify**. The firmware keeps a rolling 12-second
ring of 200 Hz IMU samples in PSRAM; when the shot detector flags a
release, the current write position is marked. The window straddling
that mark -- 4 s of the aiming approach before the release plus 2 s
of release transient and follow-through -- is streamed on this
characteristic on demand, one shot per request.

### Flow

1. Mobile receives a `provisional` or `confirmed` event on the shot
   characteristic (`5d3ace06-...`).
2. Mobile decides to fetch the window (typically on `confirmed`).
3. Mobile writes `SHOT_TRANSFER_START` (opcode `0x10`) to the control
   characteristic.
4. The firmware's transfer task wakes, waits for the post-release
   portion of the window to finish filling (~2 s typical, 10 s
   ceiling), then streams the window on `5d3ace08-...`:
   - one **header** frame (16 B) announcing the dimensions;
   - N **sample** frames (28 B each) with `chunk_idx = 0..N-1`;
   - one **trailer** frame (8 B) with the final status.

Every frame in the same transfer carries the same `transfer_seq` so a
mobile receiver can pair them and reject stragglers from a previous
transfer. The sample stream is at the BNO085's 200 Hz internal rate
(not the live 50 Hz decimated stream) -- one second of window =
200 sample frames.

If the central is subscribed but never writes `SHOT_TRANSFER_START`,
nothing is transmitted -- the characteristic is silent on idle
connections.

**Single-buffer mark.** The shot mark in firmware is single-buffer:
if shot N+1 lands while shot N's transfer is still in flight, the
transfer in flight completes from its already-allocated PSRAM
snapshot, but a *later* request would target shot N+1's window
instead. Realistic archery cadence (≥ 10 s between shots) and the
~5-6 s typical transfer time keep this rare; if it matters, ask the
firmware to expand to a 2-slot mark.

### Header frame -- 16 bytes, `frame_type = 0x00`

```
offset  size  type    field           encoding
------  ----  ------  --------------  ----------------------------------------
0       1     uint8   schema_version  currently 0x01
1       1     uint8   frame_type      0x00 (header)
2       2     uint16  total_samples   sample-frame count to expect, LE
4       2     uint16  onset_idx       0-based index of the release sample in
                                      this transfer, LE; pre-release samples
                                      occupy [0..onset_idx), the marked
                                      sample is at onset_idx, follow-through
                                      after that
6       4     uint32  onset_ms        release-onset timestamp, LE; matches
                                      the shot event's onset_ms
10      1     uint8   transfer_seq    shared with sample + trailer frames
11      5     uint8   reserved        zero
```

### Sample frame -- 28 bytes, `frame_type = 0x01`

```
offset  size  type    field           encoding
------  ----  ------  --------------  ----------------------------------------
0       1     uint8   schema_version  currently 0x01
1       1     uint8   frame_type      0x01 (sample)
2       2     uint16  chunk_idx       0-based index in this transfer, LE
4       4     uint32  t_ms            sample timestamp, firmware monotonic ms
8       2     int16   gyro_x          Q12.4 deg/s, divide by 16
10      2     int16   gyro_y          "
12      2     int16   gyro_z          "
14      2     int16   accel_x         Q8.8 m/s² (gravity removed), divide by 256
16      2     int16   accel_y         "
18      2     int16   accel_z         "
20      2     int16   quat_i          Q1.14, divide by 16384
22      2     int16   quat_j          "
24      2     int16   quat_k          "
26      2     int16   quat_w          "
```

Q-format scaling matches the IMU stream frame (sections
"Quaternion", "Gyro", "Linear acceleration") so a mobile decoder can
reuse the same helpers for both streams.

### Trailer frame -- 8 bytes, `frame_type = 0x02`

```
offset  size  type    field           encoding
------  ----  ------  --------------  ----------------------------------------
0       1     uint8   schema_version  currently 0x01
1       1     uint8   frame_type      0x02 (trailer)
2       2     uint16  sent_count      sample frames actually emitted, LE
4       1     uint8   status          0 OK, 1 truncated, 2 aborted
5       1     uint8   transfer_seq    shared with header + sample frames
6       2     uint8   reserved        zero
```

Status codes:

- **OK** (`0`) -- `sent_count == header.total_samples`; the full window
  was delivered.
- **truncated** (`1`) -- `sent_count < header.total_samples`; the
  transfer stopped early. The mbuf-retry loop ran out (the BLE link
  stalled mid-transfer); the mobile app should keep what it has and
  surface the partial sample count.
- **aborted** (`2`) -- no usable samples reached the mobile. Happens
  when the PSRAM ring is disabled, the window never filled (no shot
  was actually marked), or the subscriber dropped before the first
  sample frame. The mobile app should discard whatever fragments it
  received with this `transfer_seq`.

A trailer is always emitted even on the abort path so the mobile
parser has an unambiguous end-of-transfer marker.

**Not session-gated.** Like shot events, preshot transfers flow
whenever a central is subscribed -- they only run on explicit
request, so there is no cost to leaving the subscription active
outside a session.

### Pacing and bandwidth

Each notify allocates one mbuf from NimBLE's small pool; a tight loop
would starve the pool in ~12 frames. The firmware paces itself by
retrying mbuf allocation with 5 ms sleeps, which adapts the burst
rate to the actual BLE link rate (~200-300 notifies/s on a 15 ms
connection interval). A 4 s + 2 s window at 200 Hz = 1200 samples
therefore streams in ~5-6 s of air time at typical link rates.

## Hardware-tuning characteristics -- runtime DSP config

Two READ + WRITE characteristics let the mobile app push the firmware's
runtime DSP config without a reflash. Their layouts mirror the
`shot_detect_config_t` and `stability_config_t` structs in
`firmware/services/shot_detect/` and `firmware/services/stability/`
1:1.

**Persistence semantics.** Firmware stores tuning in RAM only. A reboot
or deep-sleep wake reverts to compile-time defaults, and the mobile
app re-pushes its saved config (under `<docs>/hardware_tuning.json`)
on each connect. This keeps the firmware NVS small, avoids flash wear
from frequent edits during a tuning session, and gives the mobile a
single source of truth -- when the two would otherwise disagree (after
a sensor reset, after a fresh install) the mobile copy wins.

**READ contract.** A READ always reflects the **current effective**
config, including any clamps applied by the setter. The mobile app
issues a READ on connect so the tuning UI shows what the firmware
actually has, then a WRITE on each Save.

**WRITE contract.** Both characteristics are Write With Response so a
tuning push is acknowledged or rejected at the ATT layer. A
length-mismatched or bad-schema payload is rejected with
`BLE_ATT_ERR_INVALID_PDU` and the firmware logs the malformed write
over UART; per-field clamps are applied silently for out-of-range
floats so a single bad value cannot stall the detector.

### Shot-detect tuning -- 24 bytes (`5d3ace09-...`)

```
offset  size  type     field                          encoding
------  ----  -------  -----------------------------  -------------------------
0       1     uint8    schema_version                 currently 0x01
1       1     uint8    reserved                       must be 0
2       4     float32  hold_gyro_threshold_rad_sq     (rad/s)^2; default 0.030
6       4     float32  accel_trigger_ratio            multiplier; default 6.0
10      4     float32  accel_trigger_floor_ms2        m/s^2; default 1.5
14      4     float32  gyro_trigger_ratio             multiplier; default 8.0
18      4     float32  gyro_trigger_floor_rad_s       rad/s; default 0.5
22      2     uint16   min_hold_ms                    LE; default 500, range 100..5000
```

Total: 24 bytes; fits inside the default ATT MTU. Field semantics
match the `SD_*` `#define` defaults in
`firmware/services/shot_detect/src/shot_detect.c`; the hardware-tuning
UI surfaces each as a labelled slider over the same range.

### Stability-score tuning -- 50 bytes (`5d3ace0a-...`)

```
offset  size  type     field               encoding (defaults match stability.c)
------  ----  -------  ------------------  ---------------------------------------
0       1     uint8    schema_version      currently 0x01
1       1     uint8    reserved            must be 0
2       4     float32  w_omega             rate-term weight; default 0.45
6       4     float32  w_accel_lf          sway-term weight; default 0.10
10      4     float32  w_disp              drift-term weight; default 0.45
14      4     float32  map_k               exp-decay rate; default 0.6931 (= ln 2)
18      4     float32  baseline_omega      rad/s; default 0.05
22      4     float32  baseline_accel_lf   m/s^2; default 0.20
26      4     float32  baseline_disp       rad; default 0.10
30      4     float32  alpha_omega         omega-RMS EMA; default 0.20
34      4     float32  alpha_accel_lf      sway-RMS EMA;  default 0.10
38      4     float32  alpha_disp          drift-RMS EMA; default 0.10
42      4     float32  alpha_qref          qref low-pass; default 0.04
46      4     float32  smoothing_alpha     display EMA;   default 0.25
```

Total: 50 bytes. Requires ATT MTU >= 53; the proactively-negotiated
247 has plenty of room. All twelve float32 fields are little-endian;
schema versioning lets a future revision extend the payload by
introducing a new schema_version (the firmware rejects unknown
versions with `BLE_ATT_ERR_INVALID_PDU`).

## Future expansion (not in v1)

- **Bundle-mode IMU stream.** Pack 4-8 frames per notify for power
  efficiency when latency tolerance is high (idle-screen background
  mode). Different characteristic UUID so old mobile builds connecting
  to new firmware fall back cleanly.

- **OTA firmware update.** Standard NimBLE OTA service. Out of scope
  for v1.

- **Multi-sensor / dual-bow.** A second BNO085 on the bow's stabiliser
  or on the archer's draw hand. Would reuse the same characteristic
  shape but be a separate device -- the mobile app supports one
  `SensorRepository` per connected device.

## Implementation references

- BNO085 datasheet & SH-2 protocol: <https://www.ceva-ip.com/product/bno085/>
- BLE GATT and ATT MTU: Bluetooth Core Spec v5.4 vol. 3 part F.
- Q-format fixed-point primer: any DSP textbook; we always use unsigned
  divisor for decode (`int16 / 16384.0`, never bit-shifts that mishandle
  the sign bit).
