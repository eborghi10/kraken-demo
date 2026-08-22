# Simulation environment

*[Docs index](index.md)*

There are two simulators, and which one a run uses changes what is real about
it. Neither is the subject of the project; both exist to put the stack under
something that moves.

---

## 1. The headless model

`kraken_sim/headless_sim` is a unicycle with about a hundred lines of physics in
it. It exposes the same topic contract as the O3DE robot — ground truth, GNSS,
IMU, wheel odometry, ground-speed radar — so scenarios do not know which one
they are running against.

It **owns `/clock`** and steps at `real_time_factor` times wall speed, so the
suite is fast and needs no GPU. The factor is load-bearing rather than a
convenience: at 5× the EKF could not keep up with its own input and the same
scenario returned 0.46 m one run and 1.46 m the next. The reasoning, and why
the navigation scenarios run at 1.0 instead, is in
[design](design.md#why-a-headless-simulator-at-all).

What it models:

| | |
| --- | --- |
| Motion | unicycle, `cmd_vel` integrated directly, no steering limit |
| GNSS | 10 Hz `NavSatFix` about a configurable datum, which **must** match `navsat_transform`'s |
| IMU | 50 Hz, yaw-rate noise and a small constant bias |
| Wheels | 50 Hz odometry, speed noise, lateral slip noise, a 1% scale error |
| Radar | 20 Hz Doppler ground speed, its own RNG stream so adding it leaves other sensors' noise sequences untouched |
| Terrain | `kraken_sim/terrain`: a traction scalar in $(0,1]$ per rectangular patch, scaling forward and yaw rate together |

What it does not model: slope, load transfer, lateral drift, obstacles, and
anything you could range off. There is no lidar here, so the obstacle layer fed
from one stays empty — see
[design](design.md#why-nav2-runs-without-a-map).

**Terrain is the one physical fault.** Every other failure in the suite is a lie
told to the filter by [the injector](faults.md); slip has to change where the
robot actually is, so it lives in the simulator. See
[design](design.md#why-terrain-is-in-the-simulator-not-the-shim).

---

## 2. The O3DE scene

`Project/` is the O3DE side, descended from [o3de/ROSConDemo][roscondemo]. It
gives real physics, real trees and a real sensor: the Kraken carries an RGL
(Robotec GPU Lidar) that raycasts the level's meshes, which is what the
[costmap obstacle layer](design.md#why-nav2-runs-without-a-map) and
[row perception](perception.md) read.

**It is a skeleton, not a finished level.** The project registers and the ROS 2
Gem wiring is described, but the scene itself has to be authored in the Editor.
That is the biggest open piece of this repo.

Three things the Docker setup does to the upstream demo, worth knowing because
they explain otherwise baffling behaviour:

- **`docker/kraken_sensors.py`** grafts GNSS, IMU and wheel odometry onto the
  robot prefabs. Upstream's Kraken carries a lidar and nothing else, because
  that demo drives on ground truth and never estimates its own pose. The
  covariances written there are *claims*, not noise: the sensors read the
  physics exactly, and a fault degrades a sensor by scaling the covariance it
  arrives with, so a default of zero would make a degraded sensor
  indistinguishable from a perfect one.
- **`patches/0002-lidar-self-exclusion.patch`** stops the lidar reporting hits
  from the machine carrying it. RGL raycasts visual meshes, so without it the
  robot sees its own chassis.
- **The engine is not baked into the image.** `/o3de` is a bind mount, and
  `docker/o3de-setup.sh` clones and builds into it. It is idempotent, so a
  re-run after a failure resumes.

[roscondemo]: https://github.com/o3de/ROSConDemo

---

## 3. Which one to use

| | headless | O3DE |
| --- | --- | --- |
| GPU | no | yes, Vulkan |
| Clock | owns `/clock`, up to 3× wall speed | real time |
| Ranging | none | RGL lidar |
| Physics | kinematic; traction as a scalar | contact, friction, load transfer |
| Used by | the whole [scenario suite](localisation.md#4-the-robustness-suite) | the [coverage runs](navigation.md#8-results) |
| Bring-up | seconds | hours, the first time |

The [fault injector](faults.md) sits between the sensors and the filter in both
cases and neither simulator knows it exists. That is what lets the same scenario
file run against either — and it is why `simulator:=o3de` is a launch argument
rather than a different stack.

Magnitudes do not carry across. The headless model scales commanded velocity by
a traction factor; O3DE gives real friction instead. The mechanism a scenario
demonstrates carries over, the numbers do not.

---

## 4. Running them

Both are in [getting started](getting-started.md). Image layout, GPU flags and
the ROS distribution build arguments are in
[docker/README.md](https://github.com/eborghi10/kraken-demo/blob/master/docker/README.md);
what an authored level has to provide is in
[Project/README.md](https://github.com/eborghi10/kraken-demo/blob/master/Project/README.md).
