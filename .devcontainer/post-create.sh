#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Runs once when the devcontainer is created. The image ships dependencies only;
# the workspace lives in the bind mount, so it has to be built here.
set -euo pipefail

# ROS's setup.sh reads unset variables, so it cannot run under `set -u`.
set +u
. "/opt/ros/${ROS_DISTRO}/setup.sh"
set -u

cd "${KRAKEN_WS}"
colcon build --symlink-install

cat <<EOF

Workspace built. In a new terminal:

  colcon test --packages-select kraken_scenarios --event-handlers console_direct+
  colcon test-result --all --verbose

  ros2 launch kraken_scenarios scenario.launch.py scenario:=total_gnss_dropout

EOF
