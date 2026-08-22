# Design notes

## Why a shim instead of simulator support

Fault injection lives in a node between the sensors and the filter, not inside
the simulator. That decision buys three things:

- The same scenarios run against O3DE, against the headless model, or against a
  recorded bag, because none of them need to know about it.
- Faults are testable without a GPU, so the suite runs on any machine.
- The filter under test is the real one, wired the real way. Nothing is stubbed.

The cost is that faults are applied to *messages*, not to physics. `wheel_slip`
corrupts the encoder reading; it does not actually make the tyres lose grip. To
a filter fed only that message the two look much alike, which is why the shim is
enough for most of this suite. Terrain is the deliberate exception, and it lives
in the simulator for the reason given below.

## Why a headless simulator at all

O3DE is the better demo and a worse test harness: it needs a GPU, an authored
level, and a long build. The suite would then run rarely, and a robustness suite
that runs rarely is decoration.

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

## Why Nav2 runs without a map

`kraken_nav` wires Smac Hybrid-A* to a curvature-following controller on top of
the same filter output the scenarios measure. Obstacles come from one source:
the lidar the O3DE Kraken carries, read by an `ObstacleLayer` on both costmaps.
The headless kinematic model has no ranging sensor, so against it that layer
never receives an observation, the costmaps stay empty and what the scenarios
exercise is the interface and the control loop rather than obstacle avoidance.

There is no map server and no static layer either way. Both costmaps are
rolling windows, so the world is remembered only as far back as the window
reaches, and the only prior the machine is given is the geofence — a costmap
filter rather than a layer, because it is a boundary set for it and not
something it senses. Structure inside a row is measured rather than mapped:
`kraken_orchard` fits a line to the trunks either side out of the same cloud
and publishes the corridor between them, which needs no estimate of where the
robot is in the world.

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

## Why the goal error is scored twice

The scorer reports `goal_error_estimated` and `goal_error_true` separately. The
first is the distance from the goal to where the filter thinks the robot is; the
second is the distance to where it actually is. On good ground the two agree and
the pair looks redundant. That is the point of measuring both.

Nav2 decides it has arrived by comparing the goal against the transform tree,
which is fed by the filter. It has no other source of truth, so a goal check can
only ever confirm that the estimate arrived. Anything that biases the estimate
biases the arrival test by exactly the same amount, and the error cancels out of
every number the robot can compute about itself.

Terrain slip is such a bias. The wheels turn at the commanded rate and the
encoders report that rate honestly, so with no fix to correct it the estimate
advances at the commanded speed while the robot advances at 45% of it. Over the
12.65 m goal used by `nav_baseline` and `nav_terrain_dropout`, across 8 seeds:

| | flat, healthy fix | slippery, no fix | slippery, radar |
| --- | --- | --- | --- |
| `goal_error_estimated` | 0.241 +/- 0.033 | 0.239 +/- 0.006 | 0.357 +/- 0.032 |
| `goal_error_true` | 0.292 +/- 0.034 | 5.494 +/- 0.023 | 0.297 +/- 0.057 |
| `path_length` | 12.61 | 7.31 | 12.55 |
| `navigation_time_s` | 22.73 | 22.81 | 63.68 |
| Nav2 reported success | 8/8 | 8/8 | 8/8 |

These are the headless kinematic sim, which scales commanded velocity by a
traction factor. O3DE gives real friction and load transfer instead, so the
mechanism carries over but these magnitudes will not.

Take the middle column first. The believed error is the same as the control's
to well inside its own spread, and is if anything slightly smaller. The true
error is 19x worse. Every signal available to the robot -- the goal check, the
filter covariance, the encoders, the heading, which stays under 3 degrees --
says the run went as well as the control run did. This is the second failure in
this project that is invisible in heading and visible only in position; the
first was the unobservable lateral velocity above, and both were found by
scoring against ground truth rather than against the estimate.

The spread is the other half of it. `goal_error_true` varies by 0.023 m across
seeds, less than the flat case's 0.034 m, so the miss is not noise that a
longer run would average out. It is a systematic bias, reproducible to the
centimetre, which is what makes it dangerous rather than merely inaccurate.

Nav2 is not wrong to report success, and it is worth being precise about that.
It was asked to drive the estimate to a pose and it did. The failure is that
nothing in the stack is positioned to notice the estimate and the robot have
parted company, because the only instrument that could notice is the fix, and
the scenario removed it.

`nav_terrain_dropout` asserts on `navigation_succeeded`, which it did not always
do. Nav2 used to return SUCCEEDED in only 5 of 8 runs, thrashing near the goal
until the progress checker fired, and asserting either outcome would have made
the test flaky. That variation turned out to be the controller being told it
was a differential drive: given the robot's real kinematics it settles, and now
declares success in all 8 runs while 5.5 m from the goal. The finding survived
the fix and got sharper, because a controller that reliably believes it arrived
is worse news than one that sometimes gives up.

## Why there is a ground-speed radar

The third column is what fixes it. A Doppler ground-speed radar bounces a beam
off the ground and reads the returned shift, so it measures how fast the vehicle
is travelling over the surface rather than how fast its wheels are turning. It
is not exotic: agricultural machinery has carried these for decades, precisely
because wheel slip in a field is the normal case rather than the fault case.

It is the only sensor in the stack that observes the quantity the bias lives in.
GNSS would also catch it, but the scenario has removed GNSS, and that is the
realistic case under canopy or in a headland shadow. The gyro sees rotation and
is already correct. The encoders are, by construction, the thing being checked.

With it, true error falls from 5.494 m to 0.297 m and, more to the point,
belief and truth agree again: 0.357 against 0.297, a gap of 0.06 m where before
it was 5.26 m. The robot's account of itself can be believed, which is a
different and stronger property than the account being accurate. The true error
also lands on the control's 0.292 m, so the radar does not merely improve the
slippery case, it recovers the flat one.

The run takes 64 s where the control takes 23 s. That is the honest outcome:
the ground really is slippery and no sensor changes that, so covering 12.6 m
takes longer. Arriving late is what success looks like here. Arriving in 23 s is
what the version without the radar reports, and it is a lie.

An earlier version of this document claimed the radar run was the most
reproducible of the three, and that fixing the estimate had stabilised the
controller without touching it. Re-measuring after the controller was given the
robot's real kinematics reversed that: the unaided slippery case is now the
tightest of the three, at 0.26 s and 0.05 m. The thrashing was never the
estimate's doing. It was the controller of the day, MPPI, planning for a robot
that could pivot.

### Why the radar replaces the wheels rather than joining them

The obvious change is to fuse the radar alongside the existing wheel velocity.
It does not work, and the reason is worth stating because it generalises.

An EKF blends disagreeing measurements by inverse variance, so the sensor
claiming to be more precise wins. The wheels report 0.01 m/s of noise and the
radar 0.02, which weights the blend 4:1 towards the wheels and recovers about a
fifth of a 0.44 m/s disagreement. Tightening the radar to 0.005 m/s still only
reaches 0.45 m/s against a true 0.36.

The problem is not that the radar is too quiet. A covariance is a claim about
accuracy, and under slip the encoders' claim is false: they report 0.01 m/s of
noise while being 0.44 m/s wrong. No quantity of correct evidence outvotes a
confident lie, because confidence is exactly what the filter weighs by. So
`ekf_radar.yaml` removes wheel forward speed rather than supplementing it. The
wheels keep the nonholonomic constraint, which stays true under slip, because
slip is forward and the constraint is sideways.

This is the one profile that is not a single measurement from its neighbour. It
removes one and adds one, and the pairing is the point.

## Why the navigation scenarios run in real time

The localisation scenarios run at `real_time_factor: 3.0`. The navigation ones
run at 1.0, and the difference is not a preference.

Nav2's controller runs in wall time while the simulator advances sim time, so
running the clock three times fast gives it a third of the compute per simulated
second. Open-loop scenarios do not notice, because nothing in them reacts to the
result. A closed loop does. It was measured under MPPI, which optimises by
sampling rollouts and so felt it hardest — on flat ground it survived, because
tracking a plan that works needs little search; under slip, where every rollout
mispredicts, it did not, and the scenario became a measurement of the host's
spare CPU. The controller has since been replaced by `ArcTracker`, which is
cheap, but the clock stays at 1.0 for closed-loop runs: the coupling is the
point, not the particular controller.

This was found by accident. Adding the radar added two publishers, which was
enough extra load to change `nav_terrain_dropout` from a tight 5.32 +/- 0.13 m
to 3.95 +/- 2.29 m. The sensor could not have caused that, since the profile
under test does not fuse it; the load could, and did. In real time the scenario
returns to 5.33 +/- 0.08 m. The lesson is that a fast clock is free only for
open-loop measurements.

The radar's noise is also drawn from its own RNG stream, seeded separately, so
that adding a sensor leaves every other sensor's noise sequence untouched and
the existing scenarios' measured thresholds still mean what they meant. That
held: `total_gnss_dropout` reproduces bit-for-bit, with a path-length spread of
exactly zero across seeds.

## Known gaps

- No innovation gating, so `gnss_spoof` is followed rather than rejected.
- Sensor faults are message-level. Terrain is the one physical fault, and it is
  traction only: no slope, no load transfer, no lateral drift.
- `Project/` builds and the Editor runs on the GPU, but no level is authored, so
  nothing yet drives the ROS 2 Gem from the O3DE side.
- The navigation scenarios score a single goal at the end of the run. There is
  no cross-track error along the path and no stuck-and-recovery metric, so a
  controller that wanders badly but arrives still scores clean.
- The radar corrects the bias but nothing reports it. Wheel speed minus radar
  speed is the slip ratio, which is a number a fruit picker could act on and is
  not published anywhere.
- Nothing detects the bias when the radar is absent, which is still the case for
  every profile but `radar`. The stack has no cross-check that would fire.
