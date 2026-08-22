# Kraken Autonomy Stack

An autonomy stack for an Ackermann orchard robot — localisation, fault
injection, mission planning, path planning, control and row perception — and
the headless harness that measures all of it. Two questions, one robot:

**Does it still know where it is when the sensors lie to you?** The stack is
driven while its sensors are deliberately broken, and the estimate is scored
against ground truth. Every failure mode is a file, every file is a test, and
the whole suite runs headless in Docker in a couple of minutes.

**Can it do a day's work without being told where to point?** `kraken_nav`
covers a whole orchard: every aisle driven, headland turns solved as geometry
rather than searched for, no stop between one row and the next — as a Nav2
navigator plugin rather than a script above the stack.

Its ancestor is [o3de/ROSConDemo][roscondemo], which is where the O3DE +
ROS 2 project layout and the `docker/` conventions come from. That demo asks
"can a robot do a job in a nice-looking world". This one asks the narrower and
grumpier version: **when the GNSS fix disappears mid-turn, does the filter
notice, does it care, and does the job still get done?**

[roscondemo]: https://github.com/o3de/ROSConDemo

## The result

One EKF configuration, one extra fused measurement, same 40-second drive with
the GNSS fix killed at t=15s. Ten runs per profile, seeds 0-9:

| profile  | fuses                  | worst position error | worst heading error |
| -------- | ---------------------- | -------------------- | ------------------- |
| `naive`  | wheel speed + GNSS     | **21.8 ± 0.9 m**     | **179.9 ± 0.04°**   |
| `robust` | + IMU yaw rate         | **0.36 ± 0.25 m**    | **1.9 ± 1.2°**      |

Mean ± standard deviation over ten seeds. Reproduce with

    ros2 run kraken_scenarios sweep total_gnss_dropout -n 10

Do not quote a single run. The harness is not reproducible: the simulator steps
on a wall-clock timer while the filter, fault injector and scorer consume its
output over DDS, so the interleaving - and the result - moves from run to run.
The order of magnitude between the two profiles is the result; the second
decimal place is not.

Both look identical while the fix is healthy. That is the entire point: the bug
is invisible until the day it matters. The `naive` profile is kept, and kept
*failing*, as the control case.

## Quick start

Nothing to install but Docker.

```bash
git clone <your fork> kraken-demo && cd kraken-demo
docker compose -f docker/docker-compose.yml run --rm test
```

That builds the workspace and runs all ten scenarios against the headless
simulator, on the CPU, in a couple of minutes. Running one scenario by hand,
sweeping seeds, watching it in RViz and bringing up the O3DE orchard are all in
**[docs/getting-started.md](docs/getting-started.md)**.

## The two findings worth reading

Ten scenarios, four of which assert *failure* on purpose, because a documented
gap is worth more than an assumed one.

**`terrain_dropout`** is `total_gnss_dropout` with one field changed, and it is
the interesting one. Every sensor is healthy and every message is honest: the
wheels really are turning at the commanded rate, the encoders really do report
that, and the robot simply does not get there. There is no disagreement between
sources for a filter to detect, so cross-checking cannot help. The error is 26×
the flat case with heading error inside the bound the flat case passes, and a
*tenth* the spread — dead reckoning over flat ground is a random walk, this is a
systematic bias.

**`nav_terrain_dropout`** is what that costs you. Given a goal 12.65 m away over
the same ground, the robot stops 5.5 m short and reports SUCCEEDED in 8 runs out
of 8, with the same believed goal error, the same clean heading and the same
confident covariance as the control. Nav2 judges arrival from the estimate,
because the estimate is all it has, so a bias in the estimate moves the finish
line with it and cancels out of every number the robot can check about itself.
A Doppler ground-speed radar is the fix, and the run then takes 64 s instead of
23 — arriving late is what success looks like, and arriving on time is the lie.

The scenario table, the scoring rules and the numbers are in
[docs/localisation.md](docs/localisation.md); the arguments behind them are in
[docs/design.md](docs/design.md).

## Documentation

The subsystem write-ups live in [docs/](docs/), which is a Jekyll site if you
enable Pages on your fork. Start at [docs/index.md](docs/index.md).

| Page | What it covers |
| --- | --- |
| [Getting started](docs/getting-started.md) | Prerequisites, the suite, one scenario, sweeps, RViz, the O3DE coverage run, tests |
| [Simulation environment](docs/simulation.md) | The headless kinematic model, the O3DE scene, and which one is real about what |
| [Localisation](docs/localisation.md) | EKF profiles, what is fused and why, the scenario suite, how a run is scored |
| [Row perception](docs/perception.md) | Lidar cloud → trunks → the corridor the machine is standing in |
| [Navigation](docs/navigation.md) | Field layout, the skip rule, headland turn geometry, swept-footprint refusal, the geofence |
| [Control](docs/control.md) | Reading arcs out of a path, the curvature trim law, speed limits, refusing to drive |
| [Mission planning](docs/mission-planning.md) | The `CoverRows` action, the navigator plugin, the behaviour tree |
| [Fault modes](docs/faults.md) | What each injected fault does to which channel, and how to add one |
| [Design notes](docs/design.md) | Why the shim is outside the simulator, why terrain is the exception, the known gaps |

`Project/` holds the O3DE side. **It is a skeleton, not a finished level** — the
project registers and the ROS 2 Gem wiring is described, but the scene itself
has to be authored in the O3DE Editor. See [Project/README.md](Project/README.md).
Contributions very welcome; that is the biggest open piece of this repo.

## Repository layout

```
docker/                 Dockerfile.Stack (light, no GPU), Dockerfile.Simulation (O3DE, GPU),
                        Dockerfile.Viz, compose file, and the scripts that graft the
                        localisation sensors onto the upstream ROSConDemo Kraken
docs/                   the documentation site: one page per subsystem, plus fault
                        modes and design notes
patches/                patches applied to the O3DE engine and gems at image build time
Project/                O3DE project skeleton (see Project/README.md)
ros2_ws/src/
  kraken_interfaces/    FaultSpec, LocalisationScore, RowEstimate, SetFault, CoverRows
  kraken_sim/           headless kinematic simulator, owns /clock, models terrain traction
  kraken_faults/        the fault injector shim
  kraken_localisation/  EKF profiles (naive / robust / radar) + navsat_transform
  kraken_scenarios/     scenario files, scorer, runner, sweep, launch tests
  kraken_nav/           Nav2 plugins: CoverRowsNavigator, HeadlandPlanner, ArcTracker,
                        BT nodes, nav2.yaml, keepout mask, launch files
  kraken_orchard/       row perception: fits the trunk lines either side from the lidar
                        cloud and publishes the corridor between them
```

## Contributing

Good first issues: a new fault mode, a new scenario, an innovation gate that
catches `gnss_spoof`, or an O3DE level. See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

BSD-3-Clause. See [LICENSE](LICENSE).
