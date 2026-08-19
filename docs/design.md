# Design notes

## Why a shim instead of simulator support

Fault injection lives in a node between the sensors and the filter, not inside
the simulator. That decision buys three things:

- The same scenarios run against O3DE, against the headless model, or against a
  recorded bag, because none of them need to know about it.
- Faults are testable in CI without a GPU.
- The filter under test is the real one, wired the real way. Nothing is stubbed.

The cost is that faults are applied to *messages*, not to physics. `wheel_slip`
corrupts the encoder reading; it does not actually make the tyres lose grip. To
a filter fed only that message the two look much alike, which is why the shim is
enough for most of this suite. Terrain is the deliberate exception, and it lives
in the simulator for the reason given below.

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

## Why terrain is in the simulator, not the shim

Every other fault here is a lie told to the filter. Terrain is not: the wheels
turn at the commanded rate, the encoders report that honestly, and the ground
declines to convert it into motion. No shim between the sensors and the filter
can produce that, because the robot's true position has to change.

It is also the only fault in the suite with no cross-check available. A spoofed
fix disagrees with the wheels. A dead gyro stops saying anything. Slip produces
a consistent, plausible and entirely honest set of messages that happen to
describe a robot which is somewhere else.

`terrain_dropout` measures the consequence against an otherwise identical flat
run: 9.57 m of position error instead of 0.36 m, with heading error inside the
same bound, because heading comes from the gyro and the gyro is not wrong. The
spread across seeds falls from sd 0.25 to sd 0.022 — flat dead reckoning is a
random walk over the noise draws, slip is a bias.

The model is traction only: a scalar in (0, 1] per rectangular patch, scaling
forward and yaw rate together. There is no slope, so there is no downhill drift
and the nonholonomic constraint the filter now fuses stays true. Adding height
would break that constraint, which is the more interesting experiment and the
reason traction lives in its own module rather than three lines in the step
function.

## Known gaps

- No innovation gating, so `gnss_spoof` is followed rather than rejected.
- Sensor faults are message-level. Terrain is the one physical fault, and it is
  traction only: no slope, no load transfer, no lateral drift.
- `Project/` builds and the Editor runs on the GPU, but no level is authored, so
  nothing yet drives the ROS 2 Gem from the O3DE side.
- Nothing yet measures navigation. The terrain that makes the robot miss its
  commanded velocity is in the simulator, but the scorer still only reports
  localisation error, so Nav2's behaviour over it is unmeasured.
