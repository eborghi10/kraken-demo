#!/usr/bin/env bash
#
# The simulation depends on three changes that live outside this repository, in
# the O3DE engine and gem checkouts. They are kept here so a fresh machine can
# reproduce a working build, and so an engine update does not silently lose them.
#
# Run inside the sim container, then rebuild and rebake:
#
#   patches/apply.sh
#   cmake --build /o3de/ROSConDemo/Project/build/linux --config profile \
#       --target RGL RGL.Editor ROS2Sensors -j $(nproc)
#   /o3de/ROSConDemo/Project/build/linux/bin/profile/AssetProcessorBatch \
#       --project-path=/o3de/ROSConDemo/Project
#
set -euo pipefail

# Where the engine, gems and demo are checked out. Not O3DE_ROOT: the sim image
# already uses that for the engine itself.
O3DE_DIR=${O3DE_DIR:-/o3de}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

apply() {
  local repo=$1 patch=$2
  if git -C "$repo" apply --reverse --check "$patch" 2>/dev/null; then
    echo "already applied: $(basename "$patch")"
  else
    git -C "$repo" apply "$patch"
    echo "applied: $(basename "$patch")"
  fi
}

# 1. The RGL gem does not compile with the clang O3DE ships; the offending moves
#    are upstream's, so the warning is disabled per target rather than edited out.
apply "$O3DE_DIR/o3de-rgl-gem" "$HERE/0001-rgl-build-with-clang.patch"

# 2. RGL raycasts visual meshes, so a lidar sees the chassis it is bolted to -
#    something the PhysX path never showed because that geometry has no collider.
#    Excludes the robot's own entities from its raycasts.
apply "$O3DE_DIR/extras" "$HERE/0002-lidar-self-exclusion.patch"

# 3. Select RGL in the Kraken prefabs.
O3DE_DIR="$O3DE_DIR" python3 "$HERE/set_lidar_implementation.py"
