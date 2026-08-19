# O3DE project — scaffolding, no level yet

**This is not a working O3DE project yet.** The engine scaffolding is here and
the ROS 2 Gem is declared, but no scene is authored, so there is nothing to
load. This is the largest open piece of the repo and contributions are very
welcome.

Everything in `ros2_ws/` works today, against the headless simulator in
`kraken_sim`. O3DE is the nicer front end, not a prerequisite.

## What is committed here

The scaffolding the O3DE CLI generates (`CMakeLists.txt`, `Gem/`, `Registry/`,
`Platform/`, ...) is committed, because a build you cannot reproduce is not a
result. `docker/o3de-setup.sh` only regenerates it when `CMakeLists.txt` is
missing, so a fresh clone builds the same tree rather than whatever the CLI
emits on the day.

Asset caches and build trees are not committed; see `.gitignore`. The Mac, iOS,
Android and Windows resources came with the template and are dead weight on a
Linux-only project — prunable if they ever get in the way.

## What the level needs to provide

The ROS 2 side is fixed by `kraken_faults/config/channels.yaml` and the EKF
configs. A level is compatible if a robot in it publishes:

| topic | type | notes |
| ----- | ---- | ----- |
| `/ground_truth/odom` | `nav_msgs/Odometry` | true pose; the scorer's reference |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | ROS 2 Gem GNSS sensor |
| `/imu/data` | `sensor_msgs/Imu` | must populate `angular_velocity.z` |
| `/wheel/odom` | `nav_msgs/Odometry` | must populate `twist.twist.linear.x` |

and subscribes to `/cmd_vel` (`geometry_msgs/Twist`).

Two things must line up or the numbers become meaningless:

1. **The GNSS origin must match the datum** in
   `kraken_localisation/config/navsat_transform.yaml` (currently
   `52.2297, 21.0122`). The ROS 2 Gem sets this on its georeference component.
2. **The simulation must publish `/clock`** and every ROS node must run with
   `use_sim_time`. `kraken_sim` does this; O3DE does it too, so drop
   `kraken_sim` from the launch when using O3DE.

A flat, obstacle-free area is enough for the shipped scenarios. They measure
dead-reckoning drift, and a collision or a slope would be measuring something
else.

## Wiring the scenarios to O3DE

`kraken_scenarios/kraken_scenarios/launch_utils.py` composes the stack, and the
switch is already wired:

```bash
ros2 launch kraken_scenarios scenario.launch.py \
    scenario:=total_gnss_dropout simulator:=o3de
```

`simulator:=o3de` starts no simulator and expects an already-running O3DE
launcher to satisfy the topic contract above. The injector, filter, scorer and
runner are unchanged.

## Origin

The project layout and the `docker/` conventions follow
[o3de/ROSConDemo](https://github.com/o3de/ROSConDemo), which is Apache-2.0 / MIT.
No code from it is copied here.
