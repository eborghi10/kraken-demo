#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Runs on every container start, not just creation. CycloneDDS is configured for
# shared memory, and a participant that cannot reach iceoryx's RouDi waits 60 s
# and then aborts: without this, every ROS node in the container dies rather
# than merely running slowly.
#
# Starting is unconditional. Testing for the socket first is not reliable: it
# lives on the shared host /tmp, so a hard-killed container leaves a stale one
# behind and we would skip the start and hang. A duplicate RouDi refuses itself
# on the lock before touching the existing segments, which is the safer check.
set -euo pipefail

log=/tmp/iox-roudi.log

# setsid, not nohup: RouDi installs its own SIGHUP handler, which replaces the
# SIG_IGN nohup sets, so it has to leave this session to survive postStart.
setsid "/opt/ros/${ROS_DISTRO}/bin/iox-roudi" >"$log" 2>&1 </dev/null &

for _ in $(seq 20); do
    if grep -q "RouDi is ready for clients" "$log" 2>/dev/null; then
        echo "Started RouDi (log: $log)."
        exit 0
    fi
    if grep -q "ROUDI_STILL_RUNNING" "$log" 2>/dev/null; then
        echo "RouDi already running elsewhere; left it alone."
        exit 0
    fi
    sleep 0.5
done

echo "WARNING: RouDi did not come up. ROS nodes will hang for 60 s and abort." >&2
echo "         See $log" >&2
