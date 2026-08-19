# Docker images

Two images, following the split used by [o3de/ROSConDemo][roscondemo]:

| file | image | contains | needs a GPU |
| ---- | ----- | -------- | ----------- |
| `Dockerfile.Stack` | `kraken_stack` | ROS 2 workspace: localisation, fault injection, headless sim, scenarios | no |
| `Dockerfile.Simulation` | `kraken_sim` | the above plus O3DE and the demo project | **yes** |

[roscondemo]: https://github.com/o3de/ROSConDemo

`Dockerfile.Stack` is the one you want. It builds in about a minute and runs the
whole scenario suite. `Dockerfile.Simulation` builds O3DE from source: tens of
gigabytes, hours, and an NVIDIA container runtime to run it.

`Dockerfile.Stack` has three stages: `base` (dependencies), `dev` (base plus
editor tooling, used by `.devcontainer`) and `stack` (base plus a baked build).
`stack` is last, so a build with no `--target` gives you that and skips `dev`.

## Devcontainer

"Dev Containers: Reopen in Container" in VS Code builds the `dev` stage, mounts
the repository at `/data/workspace` so paths match the images above, and runs
`colcon build`. Sources are mounted rather than copied, so edits are live and
only a rebuild is needed, not an image rebuild.

The container joins the host network and gets the X11 socket, so `rviz2` and
`rqt_plot` work.

## Stack

```bash
docker compose -f docker/docker-compose.yml run --rm test    # build + full suite
docker compose -f docker/docker-compose.yml run --rm stack   # interactive shell
```

`docker-compose.yml` mounts `ros2_ws/src` read-only into the container, so you
can edit on the host and rebuild inside without rebuilding the image.

Or without compose:

```bash
docker build -t kraken_stack -f docker/Dockerfile.Stack .
docker run --rm -it kraken_stack
```

## Simulation

```bash
docker build -t kraken_sim -f docker/Dockerfile.Simulation .
docker compose -f docker/docker-compose.yml run --rm sim
```

Requires the [NVIDIA container toolkit][nvidia]. [`rocker`][rocker] also works
and handles X11 for you:

```bash
rocker --x11 --nvidia --network=host kraken_sim
```

[nvidia]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker
[rocker]: https://github.com/osrf/rocker

Note that `Project/` is currently a skeleton without an authored level, so this
image builds the engine and the project but has no scene to load yet. See
[../Project/README.md](../Project/README.md).

## Choosing a ROS distribution

Both files take `ROS_VERSION` and `UBUNTU_VERSION` build arguments and default
to Jazzy on Noble:

```bash
docker build -t kraken_stack \
  --build-arg ROS_VERSION=humble --build-arg UBUNTU_VERSION=jammy \
  -f docker/Dockerfile.Stack .
```

Only Jazzy is exercised in CI.

## Networking

The stack image pins CycloneDDS to loopback via `/etc/cyclonedds.xml`.
Multicast discovery is unreliable inside containers, and everything these
images run is single-host. If you split the simulator and the stack across
containers or machines, you will need to relax that.
