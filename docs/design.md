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
At 3× it keeps up, but nothing gates a simulator step on the consumers having
finished the previous one, so the interleaving still moves between runs. Holding
the seed fixed and repeating `total_gnss_dropout` eight times gives 0.136–0.405 m
(mean 0.183, sd 0.092). The values are quantised rather than noisy — runs repeat
exactly — so this is piecewise determinism across scheduling interleavings.

Varying the seed is the larger effect, because after GNSS loss the robot is dead
reckoning and each seed is a different random walk. Bounds are therefore set from
`sweep` distributions rather than single runs, and each scenario file records the
range observed. If you raise `real_time_factor`, re-measure.

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

## Why lateral velocity is fused

A differential-drive base cannot translate sideways, but nothing in the filter
knew that. `odom0_config` fused forward speed only and `imu0_config` yaw rate
only, so once GNSS stopped, lateral velocity was unobservable. Measured, the
estimate ramped to 0.98 m/s of sideways motion on a robot whose true lateral
velocity is exactly zero, and integrated it straight into cross-track error.

Heading error stayed under 2° throughout, which is why every metric kept
reporting a healthy filter. Position error that grows while heading error does
not is a velocity-state problem, not a tuning problem.

The simulator now publishes that zero with a covariance and both profiles fuse
it. It is a kinematic constraint rather than a sensor, so it belongs in both,
and the two configs still differ by exactly one measurement. It moved the robust
profile from 2.57 m mean (max 5.13) to 0.359 m (max 0.904) across seeds. Most of
what earlier notes here blamed on scheduling jitter was this.

## Why the naive profile is kept

`ekf_naive.yaml` and `ekf_robust.yaml` differ by one fused measurement, the IMU
yaw rate. Without it, heading is only observable through GNSS, so losing the fix
leaves the estimate rotating at whatever yaw rate it last held.

Keeping the broken configuration, and asserting that it *stays* broken, is what
makes the comparison a measurement rather than a claim. If someone changes the
stack so the naive profile stops diverging, `gnss_dropout_naive` goes red and
tells them the demo no longer demonstrates anything.

## Why Nav2 runs without a costmap layer

`kraken_nav` wires Smac Hybrid-A* to MPPI on top of the same filter output the
scenarios measure. There is no ranging sensor anywhere in this repo, so there is
nothing to build an obstacle layer from: both costmaps are rolling windows of
empty space, there is no map server, and MPPI runs without `CostCritic`. It
demonstrates the interface and the control loop, not obstacle avoidance.

It is wired to the flat headless model first, so the plumbing is verified
independently of terrain. `minimum_turning_radius` is a smoothness choice rather
than a kinematic limit — a unicycle can spin in place — and only becomes a real
constraint once the trailer is modelled, which is why the planner is Hybrid-A*
and not Navfn.

## Known gaps

- No innovation gating, so `gnss_spoof` is followed rather than rejected.
- Faults are message-level, not physical.
- `Project/` builds and the Editor runs on the GPU, but no level is authored, so
  nothing yet drives the ROS 2 Gem from the O3DE side.
- The vehicle model is a unicycle with no dynamics, slip, or terrain, so the
  commanded velocity is always achieved and navigation cannot fail physically.
