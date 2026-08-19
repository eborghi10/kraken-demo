# Fault modes

Each mode is applied per channel by `kraken_faults/fault_injector`. A channel is
a named subscription/publication pair declared in
`kraken_faults/config/channels.yaml`; the shipped ones are `gnss`
(`sensor_msgs/NavSatFix`), `imu` (`sensor_msgs/Imu`) and `wheel`
(`nav_msgs/Odometry`).

Set one from a scenario file:

```yaml
- label: kill_gnss
  action: set_fault
  channel: gnss
  mode: dropout
```

or at runtime:

```bash
ros2 service call /fault_injector/set_fault kraken_interfaces/srv/SetFault \
  "{fault: {channel: gnss, mode: 1}}"
ros2 service call /fault_injector/clear_faults std_srvs/srv/Trigger
```

## `dropout` (mode 1)

| channel | behaviour |
| ------- | --------- |
| `gnss` | keeps publishing, but with `STATUS_NO_FIX` and unknown covariance |
| `imu` | stops publishing entirely |
| `wheel` | stops publishing the topic, **keeps broadcasting the last transform** |

GNSS reports `NO_FIX` rather than going silent because that is what a real
receiver does when it loses lock, and it is the harder case: a consumer that
only checks "did a message arrive" will sail straight past it.

The wheel case keeps its transform alive deliberately. `odom -> base_link` is
part of the TF chain the EKF needs in order to publish `map -> odom` at all, so
dropping it would break the whole tree rather than test the filter.

## `degrade` (mode 2)

Adds white noise of `noise_stddev` and multiplies the sensor's reported
covariance by `covariance_scale`. Native units per channel: metres for `gnss`,
rad/s for `imu`, m/s and rad/s for `wheel`. GNSS is additionally downgraded from
`STATUS_GBAS_FIX` to `STATUS_FIX`.

This models an *honest* degradation — the sensor gets worse and says so. A
well-tuned filter should widen its own uncertainty and lean on dead reckoning.

## `bias_ramp` (mode 3)

Adds `bias_rate` units per second, accumulating from the moment the fault is
set. GNSS walks off on a fixed 45° bearing; the IMU gets a growing yaw-rate
bias.

Unlike `degrade`, the reported covariance stays tight. The sensor is lying
confidently, which is the case filters handle worst.

**Known gap:** the stack currently has no innovation gating, so it follows a
GNSS spoof more or less exactly (~8 m of error for a 0.2 m/s ramp over 40 s).
`scenarios/gnss_spoof.yaml` asserts this failure so it stays visible. Adding a
Mahalanobis gate, or a consistency check between GNSS and dead reckoning, is an
open and very welcome contribution.

## `slip` (mode 4)

Odometry only. Multiplies reported linear and angular velocity by `slip_ratio`:

- `1.0` — no slip
- `< 1.0` — under-reports, e.g. dragged or locked wheels
- `> 1.0` — over-reports, e.g. wheels spinning free in mud

## `healthy` (mode 0)

Passes measurements straight through. Faults also revert to this automatically
once their `duration` elapses; a `duration` of zero means "until cleared".

## Adding a mode

1. Add the constant and any parameters to
   `kraken_interfaces/msg/FaultSpec.msg`.
2. Add it to `SUPPORTED` in `fault_injector.py` for the channel types it applies
   to, and implement it in the matching `_apply_*` method.
3. Add a scenario file that exercises it, with thresholds taken from actual
   measured runs rather than guessed.
