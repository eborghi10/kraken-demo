#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Populates the /o3de bind mount: clones the engine and the ROS 2 Gem, generates
# the project scaffolding, and builds. Run it inside the `sim` container.
#
#   docker compose -f docker/docker-compose.yml run --rm sim
#   ./docker/o3de-setup.sh
#
# Idempotent: every step is skipped if its output is already there, so re-running
# after a failure resumes rather than starting over. The build is incremental.
set -euo pipefail

O3DE_BRANCH="${O3DE_BRANCH:-development}"
O3DE_REPO="${O3DE_REPO:-https://github.com/o3de/o3de.git}"
O3DE_EXTRAS_REPO="${O3DE_EXTRAS_REPO:-https://github.com/o3de/o3de-extras.git}"
BUILD_CONFIG="${BUILD_CONFIG:-profile}"

# `development` moves under you, which is fine for getting started and
# disqualifying for a result you intend to reproduce. Set these to the SHAs this
# script prints when it finishes.
O3DE_COMMIT="${O3DE_COMMIT:-}"
O3DE_EXTRAS_COMMIT="${O3DE_EXTRAS_COMMIT:-}"

env_hint="not set - run this inside the sim container as the default user (sudo and su drop the container environment)"
: "${O3DE_ROOT:?${env_hint}}"
: "${O3DE_EXTRAS_ROOT:?${env_hint}}"
: "${KRAKEN_PROJECT:?${env_hint}}"
: "${KRAKEN_WS:?${env_hint}}"

say() { printf '\n=== %s ===\n' "$*"; }

clone_pinned() {
    local repo="$1" dest="$2" commit="$3"
    if [[ -d "${dest}/.git" ]]; then
        return 0
    fi
    git clone --single-branch -b "${O3DE_BRANCH}" "${repo}" "${dest}"
    if [[ -n "${commit}" ]]; then
        # Not a shallow clone, so any commit on the branch is already local.
        git -C "${dest}" checkout --detach "${commit}"
    fi
    git -C "${dest}" lfs install
    git -C "${dest}" lfs pull
}

# Fail here rather than after an hours-long build that cannot be run.
say "checking the GPU is reachable"
if ! vulkaninfo --summary 2>/dev/null | grep -q 'deviceName'; then
    echo "No Vulkan device. O3DE's Atom renderer is Vulkan-only, so this would" >&2
    echo "build and then refuse to start. Check on the host:" >&2
    echo "  docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \\" >&2
    echo "    kraken_sim vulkaninfo --summary" >&2
    exit 1
fi
vulkaninfo --summary 2>/dev/null | grep 'deviceName' | head -1

avail_gb=$(df -BG --output=avail /o3de | tail -1 | tr -dc '0-9')
say "disk available on /o3de: ${avail_gb} GB"
if (( avail_gb < 120 )); then
    echo "WARNING: an engine clone plus a profile build tree plus the asset" >&2
    echo "cache usually lands north of 120 GB. Free space up before starting," >&2
    echo "or this fails partway through." >&2
    if [[ -t 0 ]]; then
        read -rp "Continue anyway? [y/N] " reply
        [[ "${reply}" == [yY] ]] || exit 1
    else
        echo "Set ALLOW_LOW_DISK=1 to proceed non-interactively." >&2
        [[ "${ALLOW_LOW_DISK:-}" == "1" ]] || exit 1
    fi
fi

set +u
. "/opt/ros/${ROS_DISTRO}/setup.sh"
set -u

# The bind mount shadows whatever the image chowned, and Docker creates a
# missing mount source as root, so /o3de can arrive unwritable.
if [[ ! -w /o3de ]]; then
    say "taking ownership of /o3de"
    sudo chown "$(id -u):$(id -g)" /o3de
    [[ -w /o3de ]] || { echo "/o3de still not writable" >&2; exit 1; }
fi

say "cloning O3DE and o3de-extras (${O3DE_BRANCH})"
clone_pinned "${O3DE_REPO}" "${O3DE_ROOT}" "${O3DE_COMMIT}"
clone_pinned "${O3DE_EXTRAS_REPO}" "${O3DE_EXTRAS_ROOT}" "${O3DE_EXTRAS_COMMIT}"

if [[ ! -d "${O3DE_ROOT}/python/runtime" ]]; then
    say "fetching the O3DE python runtime"
    "${O3DE_ROOT}/python/get_python.sh"
fi

say "registering engine, gems and project"
"${O3DE_ROOT}/scripts/o3de.sh" register -ep "${O3DE_ROOT}"
# ROS2 depends on sibling gems in o3de-extras (LevelGeoreferencing, and more in
# later versions), so register the whole directory rather than chasing them by
# name one failed registration at a time.
"${O3DE_ROOT}/scripts/o3de.sh" register --all-gems-path "${O3DE_EXTRAS_ROOT}/Gems"

# Project/ holds only project.json in git; the scaffolding the CLI generates is
# deliberately not committed. Generate it elsewhere and copy in what is missing,
# so the committed project.json (which declares the ROS2 gem) survives.
if [[ ! -f "${KRAKEN_PROJECT}/CMakeLists.txt" ]]; then
    say "generating project scaffolding"
    tmp="$(mktemp -d)"
    "${O3DE_ROOT}/scripts/o3de.sh" create-project \
        --project-path "${tmp}/KrakenDemo" --project-name KrakenDemo
    # create-project registers what it generates; drop that entry before the
    # directory goes away or the manifest keeps a dangling /tmp path.
    "${O3DE_ROOT}/scripts/o3de.sh" register --remove -pp "${tmp}/KrakenDemo"
    rm -f "${tmp}/KrakenDemo/project.json"
    cp -rn "${tmp}/KrakenDemo/." "${KRAKEN_PROJECT}/"
    rm -rf "${tmp}"
fi

"${O3DE_ROOT}/scripts/o3de.sh" register -pp "${KRAKEN_PROJECT}"

say "configuring"
cd "${KRAKEN_PROJECT}"
# The image exports O3DE_ROOT, which collides with CMake's reserved
# <PACKAGENAME>_ROOT convention for find_package(o3de). It points at the engine
# we want, so adopt the policy rather than let find_package ignore it.
#
# CMake 3.28 scans every C++20 translation unit for module dependencies, which
# needs clang-scan-deps and costs a pass over the whole tree. O3DE uses no
# C++20 modules, so turn the scan off rather than depend on a tool we never use.
cmake -B build/linux -G "Ninja Multi-Config" \
    -DCMAKE_POLICY_DEFAULT_CMP0144=NEW \
    -DCMAKE_CXX_SCAN_FOR_MODULES=OFF \
    -DLY_DISABLE_TEST_MODULES=ON \
    -DLY_STRIP_DEBUG_SYMBOLS=ON

say "building (${BUILD_CONFIG}) - this takes hours the first time"
cmake --build build/linux --config "${BUILD_CONFIG}" \
    --target Editor AssetProcessor KrakenDemo.GameLauncher KrakenDemo.Assets

say "building the ROS 2 workspace"
cd "${KRAKEN_WS}"
colcon build --symlink-install

cat <<EOF

Done. The editor:

  ${KRAKEN_PROJECT}/build/linux/bin/${BUILD_CONFIG}/Editor

Built from:

  O3DE_COMMIT=$(git -C "${O3DE_ROOT}" rev-parse HEAD)
  O3DE_EXTRAS_COMMIT=$(git -C "${O3DE_EXTRAS_ROOT}" rev-parse HEAD)

Set those in the environment to rebuild this exact engine later. The default
branch is '${O3DE_BRANCH}', which moves.

There is no level yet. Author one that publishes /ground_truth/odom, /gnss/fix,
/imu/data and /wheel/odom and subscribes to /cmd_vel; the GNSS georeference must
match the datum in kraken_localisation/config/navsat_transform.yaml. See
Project/README.md.

Then drive scenarios against it instead of the headless model:

  ros2 launch kraken_scenarios scenario.launch.py \\
      scenario:=total_gnss_dropout simulator:=o3de
EOF
