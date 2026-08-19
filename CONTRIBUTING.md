# Contributing

## Getting set up

Everything runs in Docker; you do not need ROS 2 on your machine.

```bash
docker compose -f docker/docker-compose.yml run --rm test    # build + full suite
docker compose -f docker/docker-compose.yml run --rm stack   # interactive shell
```

Inside the interactive shell:

```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch kraken_scenarios scenario.launch.py scenario:=total_gnss_dropout
colcon test --packages-select kraken_scenarios && colcon test-result --all --verbose
```

## Adding a scenario

Add a YAML file to `ros2_ws/src/kraken_scenarios/scenarios/` and its name to
`SCENARIOS` in `test/test_scenarios.py`. No other code changes.

**Run it at least three times and set the thresholds from what you measure**,
with roughly 2× margin, then record the observed values in a comment in the
file. Thresholds picked by guesswork are worse than no thresholds: they fail for
unrelated reasons and get loosened until they assert nothing.

A scenario that asserts a *failure* is legitimate and welcome — see
`gnss_spoof.yaml`. Say clearly in the description why the failure is expected.

## Adding a fault mode

See [docs/faults.md](docs/faults.md#adding-a-mode).

## Things we would like help with

- **An O3DE level.** `Project/` is a skeleton. This is the biggest open piece.
- **Innovation gating**, so `gnss_spoof` can be caught rather than documented.
- **More failure modes**: multipath, GNSS/IMU time desync, partial constellation
  loss, wheel encoder quantisation.
- **A second filter backend** to compare against `robot_localization`.

## Style

- Python, 4 spaces, 99 columns.
- Comments explain *why*, not *what*. If a constant was measured, say what it
  was measured from.
- Keep scenario logic in YAML and filter tuning in YAML. Python is for
  mechanism.
