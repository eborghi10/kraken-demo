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

- **`docker/kraken_sensors.py`** grafts GNSS, IMU, wheel odometry and two
  cameras onto the robot prefabs. Upstream's Kraken carries a lidar and nothing
  else, because that demo drives on ground truth and never estimates its own
  pose. The covariances written there are *claims*, not noise: the sensors read
  the physics exactly, and a fault degrades a sensor by scaling the covariance
  it arrives with, so a default of zero would make a degraded sensor
  indistinguishable from a perfect one.
- **`patches/0002-lidar-self-exclusion.patch`** stops the lidar reporting hits
  from the machine carrying it. RGL raycasts visual meshes, so without it the
  robot sees its own chassis.
- **The engine is not baked into the image.** `/o3de` is a bind mount, and
  `docker/o3de-setup.sh` clones and builds into it. It is idempotent, so a
  re-run after a failure resumes.

[roscondemo]: https://github.com/o3de/ROSConDemo

### The cameras

Two, both colour only at 640×480, 60° vertical field of view — about 75°
horizontal at 4:3. Configured for 10 Hz; measured at 12, for the reason below.

| | `camera_front` | `camera_ground` |
| --- | --- | --- |
| Pose on `base_link` | x 2.35, z 1.40, level | x 2.65, z 0.90, pitched 45° down |
| Sees | down the row | a ~3.1 m strip of floor, 2.9–6.0 m ahead |
| Intended for | people and hazards | visual odometry |

Each is a child entity rather than another component on `base_link`, because the
pose is the whole point and a component cannot carry one. That also means their
extrinsics arrive on `/tf_static` for free, unlike `base_link` itself, whose
transform is deliberately silenced.

**Depth is off.** A rendered depth buffer is exact — no baseline, no matching
failures, no noise — and a sensor that cannot be wrong teaches nothing. The
depth publishers stay in the configuration because the component looks each of
its four publishers up by name; only the channel is disabled.

**Nothing subscribes to them yet.** They exist to be priced. `camera_ground` in
particular is named for a job nothing implements: a forward camera in a 3.5 m
aisle sees a self-similar receding tunnel, which is the degenerate case for
visual odometry, so the parallax has to come off the floor instead.

`KRAKEN_CAMERAS` picks which of them publish — `all` (the default), `none`, or a
comma separated list of frame names — and is read when the prefab is generated,
so changing it means re-running the script and restarting O3DE.

```sh
KRAKEN_CAMERAS=camera_front python3 docker/kraken_sensors.py "$PROJECT"
ros2 run kraken_scenarios sim_perf --label camera_front --duration 30
```

`sim_perf` reads the cost back off a running level without consuming a frame. It
times the IMU instead: O3DE renders against wall clock, so a render pass it
cannot afford shows up as its tick slipping, and every sensor slips with it.
Camera rate comes off `camera_info` rather than the image — same component, same
tick, a few hundred bytes instead of very nearly a megabyte. `--images`
subscribes to the images too, which measures transport rather than the render,
and adds load while doing it.

**Measured**, playground level, one Kraken, RTX 3500 Ada, images subscribed:

```
reference /kraken1/imu/data          50.04 Hz of 50.0 nominal  (100%)
/kraken1/camera_front/camera_info    12.00 Hz
/kraken1/camera_front/image_color    12.00 Hz  14.74 MB/s
/kraken1/camera_ground/camera_info   12.00 Hz
/kraken1/camera_ground/image_color   12.00 Hz  14.74 MB/s
```

Both cameras cost the sim tick nothing measurable: the IMU holds its full 50 Hz
and the clock stays at 1.0x while 29.5 MB/s of pixels crosses the graph. The
12 Hz is not the 10 Hz that was asked for — the gem services the sensor on a
render tick boundary, so the rate snaps to a divisor of the frame rate rather
than honouring the request exactly. 12 is the nearest one above 10.

**This needs a broker running, and without it you lose most of the frames
silently.** A frame is 640×480×4 = 1.23 MB. Against the stock
`net.core.rmem_max` of 208 kB it does not fit in a socket buffer, and since the
publisher is BEST_EFFORT a fragment lost to the overrun is never retransmitted
and takes its whole frame with it — `image_color` measured 0.95 Hz while
`camera_info` sat at 12. Nothing logs an error; the topic is simply mostly
empty.

The fix is to keep the frames out of the socket. `docker/cyclonedds.xml` turns
on CycloneDDS shared memory, which hands each frame over as a segment brokered
by iceoryx's RouDi. Start it before the stack or the sim:

```sh
docker compose -f docker/docker-compose.yml up -d roudi
```

RouDi arrives as a dependency of `ros-jazzy-cyclonedds`, so it is already in
both images, and one instance serves every container running with `ipc: host`.

Measured on an untuned host, 640×480 RGBA published at 12 Hz, subscriber rate:

| transport | received |
| --- | --- |
| CycloneDDS over UDP | 1.4 Hz |
| CycloneDDS over UDP, config asking for 16 MB buffers | 3.9 Hz |
| FastDDS, defaults | 1.0 Hz |
| **CycloneDDS over shared memory** | **11.95 Hz** |

The second row is the useful one: the config alone cannot fix this, because the
kernel clamps the request.

**RouDi is not optional, and it fails hard.** There is no fallback to UDP. A
Cyclone participant that cannot reach it waits 60 s and then aborts with
`IPC_INTERFACE__REG_ROUDI_NOT_AVAILABLE` — and that is every node using this
config, not only the ones touching cameras. So the compose services all
`depends_on: [roudi]`, RouDi runs `restart: unless-stopped`, and the
devcontainer starts one from `.devcontainer/ensure-roudi.sh` on every start
rather than at creation.

**It needs a shared `/tmp` as much as it needs `ipc: host`.** The segments live
in `/dev/shm`, which `ipc: host` shares, but clients register over a Unix socket
at `/tmp/roudi`, and that path is compiled into iceoryx — `iox-roudi` has no
option to move it. A container with a private `/tmp` therefore never finds the
broker and hangs, even though the shared memory itself is perfectly visible. So
every DDS container mounts the host `/tmp`, and so does the devcontainer via
`runArgs`. Measured container-to-container on a stock kernel: 11.95 Hz.

If ROS nodes start hanging for a minute and dying, check `ls -l /tmp/roudi`
first — not `pgrep`, which cannot see into another container's PID namespace.
Starting a second RouDi is refused rather than damaging
(`ICEORYX_ROUDI_MEMORY_MANAGER__ROUDI_STILL_RUNNING`).

**And every participant has to share a uid:gid.** RouDi scopes each segment to
the group that created it, so a client in a different group is refused at
registration — which is the same 60 s hang, not a warning. A root client and a
uid-1000 RouDi reject each other in both directions. All three images therefore
take `USER_UID`/`USER_GID` build arguments and run as that user rather than
root; the devcontainer gets there on its own through `updateRemoteUserUID`. It
has to be a build argument and not `--user` at run time, because RouDi names the
segment after `getgrgid()`, so the gid must exist in `/etc/group`. If your host
is not 1000:1000, export them before building:

```sh
export USER_UID=$(id -u) USER_GID=$(id -g)
docker compose -f docker/docker-compose.yml build
```

**The alternative, and why it is not the default.** Raising the host limit
reaches the same 12 Hz:

```sh
sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216
# lost on reboot; to keep it:
printf 'net.core.rmem_max=16777216\nnet.core.wmem_max=16777216\n' \
  | sudo tee /etc/sysctl.d/30-kraken-dds.conf
```

That needs root, applies machine-wide, and lives outside the repository, so a
fresh checkout on an untuned machine reproduces the bug with nothing to suggest
why. It is harmless to anything else sharing the machine — `rmem_max` is a
ceiling, not an allocation, so raising it makes no process consume more — but
the shared-memory route is self-contained, which is why it is the one wired up.
The socket buffer sizes in `docker/cyclonedds.xml` no longer carry the images at
all, but they still govern every other topic: `min` stays at 64 kB deliberately,
because Cyclone treats a `min` it cannot reach as fatal, so a hard 16 MB floor
would make the launcher exit with "failed to increase socket receive buffer
size" on an untuned host rather than merely running lossy.

Recorded as *not* fixes, because they look like ones: raising `MaxMessageSize`
to 65500 B to cut 686 fragments per frame down to 15 measured *worse* at
0.25 Hz, since coarser datagrams overrun a small buffer faster; and FastDDS,
whose shared memory is on by default but whose default 512 kB segment is too
small to hold a frame, so it silently falls back to UDP. Given a 64 MB segment
it matches Cyclone at 11.95 Hz, but adopting it would mean re-deriving the
loopback discovery pinning for the whole stack.

**What `none` actually buys.** It leaves the camera entities in place and clears
`Publishing Enabled`. In the gem that flag is tested in the tick callback
*before* `FrequencyTick`, which is the only thing that asks for a frame, and the
pipeline is taken off the render tick when it is built so it draws only on
request. A disabled camera therefore does no per-frame GPU work: `none` is a
genuine zero-render baseline.

What it does not free is memory. The `CameraSensor` and its render-to-texture
pipeline are built during `Activate` whatever the flag says, so a disabled camera
still holds its render target. Read the reference rate as the cost of rendering,
not as the cost of existing.

---

## 3. Which one to use

| | headless | O3DE |
| --- | --- | --- |
| GPU | no | yes, Vulkan |
| Clock | owns `/clock`, up to 3× wall speed | real time |
| Ranging | none | RGL lidar |
| Cameras | none | two, colour, 12 Hz |
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
