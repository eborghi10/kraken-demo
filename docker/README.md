# Docker images

Two images, following the split used by [o3de/ROSConDemo][roscondemo]:

| file | image | contains | needs a GPU |
| ---- | ----- | -------- | ----------- |
| `Dockerfile.Stack` | `kraken_stack` | ROS 2 workspace: localisation, fault injection, headless sim, scenarios | no |
| `Dockerfile.Simulation` | `kraken_sim` | dependencies for O3DE and the ROS 2 Gem; the engine itself is bind-mounted | **yes** |

[roscondemo]: https://github.com/o3de/ROSConDemo

`Dockerfile.Stack` is the one you want. It builds in about a minute and runs the
whole scenario suite. `Dockerfile.Simulation` carries only the dependencies; the
engine is cloned and built into a bind mount by `o3de-setup.sh`, which takes
hours and upwards of 120 GB the first time.

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

The engine is deliberately **not** baked into the image. Baking it in costs
roughly twice the disk, because the LFS clone layer and the build tree layer
both persist, and it turns every project change into a multi-hour rebuild with
no incremental linking. `/o3de` is a bind mount instead.

On the host, once:

```bash
xhost +local:                      # or the container cannot reach the X server
export O3DE_HOME=/somewhere/with/120GB   # defaults to $HOME/o3de
mkdir -p "$O3DE_HOME"              # let Docker create it and it lands root-owned
```

Then:

```bash
docker compose -f docker/docker-compose.yml build sim
docker compose -f docker/docker-compose.yml run --rm sim
./docker/o3de-setup.sh             # inside the container, first run only
```

`o3de-setup.sh` clones the engine and `o3de-extras`, generates the project
scaffolding that is deliberately not committed, and builds. It is idempotent:
each step is skipped if its output exists, so a re-run after a failure resumes
rather than restarting. It checks for a Vulkan device before starting, because
Atom is Vulkan-only and would otherwise build for hours and then refuse to run.

Requires the [NVIDIA container toolkit][nvidia]. Pass `NVIDIA_DRIVER_CAPABILITIES=all`,
not just `--gpus all`: the latter grants only `compute,utility`, and Vulkan needs
`graphics`. [`rocker`][rocker] handles X11 and the GPU flags for you:

```bash
rocker --x11 --nvidia --network=host kraken_sim
```

[nvidia]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker
[rocker]: https://github.com/osrf/rocker

Note that `Project/` is currently a skeleton without an authored level, so this
builds the engine and the project but has no scene to load yet. See
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
