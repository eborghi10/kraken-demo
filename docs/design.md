# Design notes

## Why a shim instead of simulator support

Fault injection lives in a node between the sensors and the filter, not inside
the simulator. That decision buys three things:

- The same scenarios run against O3DE, against the headless model, or against a
  recorded bag, because none of them need to know about it.
- Faults are testable in CI without a GPU.
- The filter under test is the real one, wired the real way. Nothing is stubbed.

The cost is that faults are applied to *messages*, not to physics. Wheel slip
here corrupts the encoder reading; it does not actually make the tyres lose
grip. For localisation testing that distinction does not matter, because the
filter only ever sees the message. For controller testing it would.

## Why a headless simulator at all

O3DE is the better demo and a worse test harness: it needs a GPU, an authored
level, and a long build. The suite would then either not run in CI or run
rarely, and a robustness suite that runs rarely is decoration.

`kraken_sim` is a unicycle model with about a hundred lines of physics in it.
It owns `/clock`, so scenarios run at a configurable multiple of wall speed and
produce the same numbers on every machine.

**The time factor is load-bearing.** At 5× the EKF could not keep up with its
own input, and the same scenario returned 0.46 m one run and 1.46 m the next.
At 3× the spread across three runs is 0.69–1.39 m. Thresholds in this repo are
set from measured runs with roughly 2× margin, and each scenario file records
what was actually observed. If you raise `real_time_factor`, re-measure.

## Why errors are relative to the fault instant

`navsat_transform` anchors the map frame on a datum; the simulator anchors its
world frame on its own origin. Those never line up perfectly, and the residual
offset is a constant, not a localisation failure. Comparing the estimate and
the ground truth *in the frame of the pose pair captured when the fault was
injected* cancels it, leaving only the drift accumulated since.

## Why path length and rotation are checked

A robot that never moved has excellent localisation error. So does one wedged
against a wall, or one whose `cmd_vel` never arrived. Every scenario therefore
asserts a minimum ground-truth path length and a minimum accumulated rotation,
so that a run has to have actually happened before its error numbers are
allowed to mean anything.

Both are path totals rather than net displacement, because the trajectory loops
back on itself and net displacement would be near zero.

## Why worst-case *and* final error

Losing a sensor mid-turn produces a transient the filter then recovers from.
For some faults that transient is acceptable and a permanent offset is not, so
scenarios choose:

- `max_position_error` / `max_heading_error_deg` bound the running worst case
- `max_final_position_error` / `max_final_heading_error_deg` bound the settled value

`imu_dropout` is the clearest example: heading lags by ~30° during the turn and
comes back under 15°, while position never exceeds half a metre. Bounding the
peak there would just be a slow way of asserting that turning exists.

## Why the naive profile is kept

`ekf_naive.yaml` and `ekf_robust.yaml` differ by one fused measurement, the IMU
yaw rate. Without it, heading is only observable through GNSS, so losing the fix
leaves the estimate rotating at whatever yaw rate it last held.

Keeping the broken configuration, and asserting that it *stays* broken, is what
makes the comparison a measurement rather than a claim. If someone changes the
stack so the naive profile stops diverging, `gnss_dropout_naive` goes red and
tells them the demo no longer demonstrates anything.

## Known gaps

- No innovation gating, so `gnss_spoof` is followed rather than rejected.
- Faults are message-level, not physical.
- `Project/` is an O3DE skeleton; the level is not authored.
- The vehicle model is a unicycle with no dynamics, slip, or terrain.
