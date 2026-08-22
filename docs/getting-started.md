# Getting started

*[Docs index](index.md)*

Nothing to install but Docker. The whole scenario suite runs headless, on the
CPU, in a couple of minutes. A GPU is needed only for the O3DE scene.

---

## 1. Prerequisites

| Want to | Need |
| --- | --- |
| Run the scenario suite, sweep it, hack on the stack | Docker |
| Watch a run in RViz | Docker, an X server, `xhost +local:` |
| Run the O3DE orchard | An NVIDIA GPU with Vulkan, the [container toolkit][nvidia], ~120 GB of disk and several hours for the first engine build |

[nvidia]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker

```bash
git clone <your fork> kraken-demo && cd kraken-demo
```

Every command below is run from the repository root.

---

## 2. The scenario suite

```bash
docker compose -f docker/docker-compose.yml run --rm test
```

That builds the workspace and runs all ten scenarios against the
[headless simulator](simulation.md#1-the-headless-model). Reports land in
`/tmp/kraken_reports/<scenario>.json` inside the container; set
`KRAKEN_REPORT_DIR` to move them.

To poke at it by hand instead:

```bash
docker compose -f docker/docker-compose.yml run --rm stack
# inside:
colcon build --symlink-install && . install/setup.bash
ros2 launch kraken_scenarios scenario.launch.py scenario:=total_gnss_dropout
```

`scenario.launch.py` takes `report:=`, `seed:=`, `namespace:=` and
`simulator:=headless|o3de`. The scenario names are the file stems in
`ros2_ws/src/kraken_scenarios/scenarios/`.

**Do not quote a single run.** The harness is not reproducible to the decimal
place — see [design](design.md#why-a-headless-simulator-at-all). Measure a
distribution:

```bash
ros2 run kraken_scenarios sweep total_gnss_dropout -n 10
ros2 run kraken_scenarios sweep total_gnss_dropout -n 8 --fixed-seed   # jitter only
```

Against the O3DE scene, add `--simulator o3de --namespace kraken1
--spawn-point line1`. The spawn point is not optional there in practice: O3DE
outlives the stack, so without it every run after the first starts wherever the
previous one stopped.

---

## 3. Watching a run

RViz lives in its own image and only subscribes, so it can be started and killed
mid-run without disturbing anything. On the **host** first:

```bash
xhost +local:
docker compose -f docker/docker-compose.yml run --rm viz
```

---

## 4. The orchard coverage run

This one needs the O3DE scene, so build it first — see
[simulation §2](simulation.md#2-the-o3de-scene) and
[docker/README.md](https://github.com/eborghi10/kraken-demo/blob/master/docker/README.md).

```bash
export KRAKEN_ROOT=... O3DE_HOME=...

# Always start a fresh simulator. Spawning into a running one stacks a second
# robot on the same point and they collide.
docker rm -f kraken_ns
docker compose -f docker/docker-compose.yml run --rm -d --name kraken_ns sim \
  /o3de/ROSConDemo/Project/build/linux/bin/profile/ROSConDemo.GameLauncher

docker compose -f docker/docker-compose.yml run --rm --entrypoint bash stack -c '
  source /opt/ros/$ROS_DISTRO/setup.bash && cd $KRAKEN_WS
  colcon build --packages-select kraken_interfaces kraken_nav kraken_scenarios
  source install/setup.bash
  ros2 run kraken_scenarios sim_admin spawn line1 kraken1
  ros2 launch kraken_nav orchard.launch.py namespace:=kraken1 localisation:=ekf &
  sleep 55
  ros2 action send_goal /kraken1/cover_rows kraken_interfaces/action/CoverRows \
    "{aisles: 18, aisle_pitch: 3.5, aisle_skip: 0,
      row_near_x: 3.0, row_far_x: 43.0, row_heading_deg: -90.0}"'
```

`aisle_skip: 0` lets the [skip rule](navigation.md#the-skip-rule) work it out
from the turning circle. What the machine then does is
[mission planning](mission-planning.md).

---

## 5. Tests and figures

Neither needs a simulator or a GPU:

```bash
colcon test --packages-select kraken_nav      # 12 tests, the turn geometry
colcon test --packages-select kraken_orchard  # row fitting
python3 docs/figures/make_figures.py          # regenerates the SVGs, and
                                              # asserts the same reference table
```

---

## 6. Where to go next

- Something to break: [fault modes](faults.md), then add a scenario file.
- Something to fix: `gnss_spoof` is
  [followed rather than rejected](design.md#known-gaps) for want of an
  innovation gate.
- Something to build: `Project/` is a skeleton with no authored level, which is
  the biggest open piece of the repo.

Conventions and review expectations are in
[CONTRIBUTING.md](https://github.com/eborghi10/kraken-demo/blob/master/CONTRIBUTING.md).
