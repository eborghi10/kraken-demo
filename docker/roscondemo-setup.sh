#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Brings up the O3DE Apple Kraken orchard (ROSConDemo) on THIS engine.
#
#   docker compose -f docker/docker-compose.yml run --rm sim
#   ./docker/roscondemo-setup.sh   # inside the container
#
# Run docker/o3de-setup.sh first: this script needs the engine and o3de-extras
# already cloned and registered, and reuses both.
#
# WHY THIS SCRIPT EXISTS
#
# ROSConDemo was last touched in 2023. It targets O3DE 2.1.0 with ROS 2 Humble
# on Ubuntu 22.04, and declares a single `ROS2` gem. Since then the ROS 2 Gem
# was split into ROS2 / ROS2Controllers / ROS2Sensors / ROS2RobotImporter, so
# the demo names components and headers that have moved gem. It does not build
# unmodified.
#
# The port turns out to be small, because the APIs the demo actually calls
# (ROS2Interface::Get, ROS2Names::GetNamespacedName, ROS2Conversions::ToROS)
# all stayed in the core gem. What moved is one header and two link targets.
# That is why patching upstream in place beats forking it: the delta is small
# enough to re-apply against a newer engine, and keeping it as a patch keeps
# the 2.9 GB of CC-BY / CC-BY-NC orchard assets out of this repository.
#
# The patches are applied in place and guarded, so re-running is a no-op.
set -euo pipefail

ROSCON_REPO="${ROSCON_REPO:-https://github.com/o3de/ROSConDemo.git}"
ROSCON_ROOT="${ROSCON_ROOT:-/o3de/ROSConDemo}"
BUILD_CONFIG="${BUILD_CONFIG:-profile}"

env_hint="not set - run this inside the sim container as the default user (sudo and su drop the container environment)"
: "${O3DE_ROOT:?${env_hint}}"
: "${O3DE_EXTRAS_ROOT:?${env_hint}}"

say() { printf '\n=== %s ===\n' "$*"; }

[[ -f "${O3DE_ROOT}/engine.json" ]] || {
    echo "No engine at ${O3DE_ROOT}. Run docker/o3de-setup.sh first." >&2
    exit 1
}

# Atom is Vulkan-only, so a headful editor build that cannot reach a GPU is
# hours of work for something that refuses to start. Fail now instead.
say "checking the GPU is reachable"
if ! vulkaninfo --summary 2>/dev/null | grep -q 'deviceName'; then
    echo "No Vulkan device. Check 'xhost +local:' on the host and that the" >&2
    echo "NVIDIA container toolkit is installed." >&2
    exit 1
fi
vulkaninfo --summary 2>/dev/null | grep 'deviceName' | head -1

say "cloning ROSConDemo"
if [[ -d "${ROSCON_ROOT}/.git" ]]; then
    echo "already at ${ROSCON_ROOT}"
else
    # The orchard art is ~1.4 GB of LFS objects. Without them the level opens
    # to missing-asset placeholders rather than an orchard.
    git clone "${ROSCON_REPO}" "${ROSCON_ROOT}"
    git -C "${ROSCON_ROOT}" lfs install
    git -C "${ROSCON_ROOT}" lfs pull
fi

PROJECT="${ROSCON_ROOT}/Project"

say "patching the project for this engine"

# Patches are applied to a pristine checkout on every run. That makes this
# idempotent without a pile of guards, and keeps the whole delta against
# upstream visible with `git -C /o3de/ROSConDemo diff`.
git -C "${ROSCON_ROOT}" checkout -- Project/project.json Project/Gem

# 1. project.json: the engine version gate, and the enabled gem list.
#
#    The demo declares its gems in Gem/enabled_gems.cmake, which is how O3DE
#    2.1 did it. 2.7 reads project.json instead and silently ignores that file,
#    so without this every Atom, PhysX and Terrain target the demo links is
#    "not found" at generate time.
#
#    Two of the names in that list no longer exist. PhysX is now PhysX5, which
#    still exports Gem::PhysX.* through a legacy alias, so enabling it is
#    enough and the source needs no change.
#
#    enabled_gems.cmake is rewritten from the same list rather than left alone.
#    It is still read at CMake time, and PhysX5 carries gem_alt_name "PhysX",
#    so the stale file enabling "PhysX" while project.json enables "PhysX5"
#    resolves to one gem enabled twice - which registration rejects as two
#    providers of the unique 'Physics' service.
ENGINE_VERSION="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["version"])' "${O3DE_ROOT}/engine.json")"
python3 - "${PROJECT}/project.json" "${PROJECT}/Gem/enabled_gems.cmake" "${ENGINE_VERSION}" <<'PY'
import json, sys

project_path, gems_cmake_path, engine_version = sys.argv[1], sys.argv[2], sys.argv[3]
with open(project_path) as fh:
    project = json.load(fh)

# enabled_gems.cmake, translated to names this engine actually ships, plus the
# gems the ROS 2 Gem split created.
gems = [
    "ROSConDemo",
    "ROS2", "ROS2Controllers", "ROS2Sensors",
    "Atom", "Atom_Feature_Common", "CommonFeaturesAtom", "Atom_AtomBridge",
    "PhysX5", "PhysXCommon", "ScriptCanvasPhysics",
    "LmbrCentral", "AudioSystem", "CameraFramework", "DebugDraw",
    "EditorPythonBindings", "EMotionFX", "GameState", "ImGui",
    "LandscapeCanvas", "LyShine", "PrimitiveAssets", "PrefabBuilder",
    "SaveData", "ScriptEvents", "StartingPointInput", "TextureAtlas",
    "WhiteBox", "DiffuseProbeGrid", "Terrain", "Vegetation",
]

project["engine_version"] = engine_version
project["gem_names"] = gems

with open(project_path, "w") as fh:
    json.dump(project, fh, indent=4)
    fh.write("\n")

with open(gems_cmake_path, "w") as fh:
    fh.write("\nset(ENABLED_GEMS\n")
    for gem in gems:
        fh.write(f"    {gem}\n")
    fh.write(")\n")

print(f"project.json + enabled_gems.cmake -> engine {engine_version}, {len(gems)} gems")
PY

# 2. Headers the demo includes that moved, and the calls that moved with them.
#    ROS2Bus.h and ROS2Conversions.h are untouched.
#
#    MotorizedJoints simply changed gem when the ROS 2 Gem was split, and so did
#    VehicleDynamics. Both moved namespace as well as path, which the include
#    rewrite alone does not cover.
grep -rl 'ROS2/Manipulation/MotorizedJoints' "${PROJECT}/Gem" \
    | xargs -r sed -i 's|ROS2/Manipulation/MotorizedJoints|ROS2Controllers/Manipulation/MotorizedJoints|g'
grep -rl 'ROS2::PidMotorController' "${PROJECT}/Gem" \
    | xargs -r sed -i 's|ROS2::PidMotorController|ROS2Controllers::PidMotorController|g'
echo "include path + namespace -> ROS2Controllers::PidMotorController"

grep -rl 'ROS2/VehicleDynamics' "${PROJECT}/Gem" \
    | xargs -r sed -i \
        -e 's|ROS2/VehicleDynamics|ROS2Controllers/VehicleDynamics|g' \
        -e 's|ROS2::VehicleDynamics|ROS2Controllers::VehicleDynamics|g'
echo "include path + namespace -> ROS2Controllers::VehicleDynamics"

#    ROS2Names is not a move but an API change: the static utility class became
#    an EBus, so ROS2/Utilities/ROS2Names.h is gone and the seven call sites
#    have to fetch the result through BroadcastResult instead of returning it.
#    All of them are GetNamespacedName assigned to a fresh local, which is why
#    one rule covers them; the auto has to become an explicit type because the
#    variable is now declared before it is filled in.
grep -rl 'ROS2/Utilities/ROS2Names.h' "${PROJECT}/Gem" \
    | xargs -r sed -i \
        -e 's#ROS2/Utilities/ROS2Names\.h#ROS2/ROS2NamesBus.h#' \
        -e 's#^\(\s*\)\(auto\|AZStd::string\) \([A-Za-z0-9_]*\) = ROS2Names::GetNamespacedName(\(.*\));#\1AZStd::string \3;\n\1ROS2NamesRequestBus::BroadcastResult(\3, \&ROS2NamesRequests::GetNamespacedName, \4);#'
echo "ROS2Names:: static calls -> ROS2NamesRequestBus"

#    Same story for the clock: GetROSTimestamp left the ROS2 interface for a
#    dedicated bus, so the result comes back through the out-parameter and the
#    local needs a real type.
grep -rl 'GetROSTimestamp' "${PROJECT}/Gem" \
    | xargs -r sed -i \
        -e 's|#include <ROS2/ROS2Bus.h>|#include <ROS2/Clock/ROS2ClockRequestBus.h>\n#include <ROS2/ROS2Bus.h>|' \
        -e 's|^\(\s*\)auto timestamp = ROS2Interface::Get()->GetROSTimestamp();|\1builtin_interfaces::msg::Time timestamp;\n\1ROS2ClockRequestBus::BroadcastResult(timestamp, \&ROS2ClockRequests::GetROSTimestamp);|'
echo "ROS2Interface::GetROSTimestamp -> ROS2ClockRequestBus"

#    ROS2::Utils::GetGameOrEditorComponent<T>() is gone entirely. It existed to
#    paper over the game/editor component split when fetching a component off an
#    entity; the frame component now answers on its own bus, addressed by entity
#    id, which works in both contexts and makes the helper unnecessary. The
#    pointer it returned was used for two values, so both are read up front and
#    the concrete type is no longer named - which retires the include too.
#    GetFrameID became GetNamespacedFrameID; the other accessors on that bus
#    return the frame name without the namespace, which is not what the caller
#    wants for a message header.
grep -rl 'GetGameOrEditorComponent' "${PROJECT}/Gem" \
    | xargs -r sed -i \
        -e 's|#include <ROS2/Frame/ROS2FrameComponent.h>|#include <ROS2/Frame/ROS2FrameComponentBus.h>|' \
        -e 's|^\(\s*\)auto frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());|\1AZStd::string robotNamespace;\n\1ROS2FrameComponentBus::EventResult(robotNamespace, GetEntityId(), \&ROS2FrameComponentRequests::GetNamespace);\n\1AZStd::string robotFrameId;\n\1ROS2FrameComponentBus::EventResult(robotFrameId, GetEntityId(), \&ROS2FrameComponentRequests::GetNamespacedFrameID);|' \
        -e '/auto robotNamespace = frame->GetNamespace();/d' \
        -e 's|frame->GetFrameID()|robotFrameId|'
echo "Utils::GetGameOrEditorComponent -> ROS2FrameComponentBus"

#    ImGui/ImGuiPass.h is an Atom internal header these days, not something a
#    gem can include. The demo only ever wanted the imgui drawing calls, which
#    live in the 3rdParty header the ImGui gem exports; the bus it derives from
#    comes from ImGuiBus.h, which it already includes.
grep -rl 'ImGui/ImGuiPass.h' "${PROJECT}/Gem" \
    | xargs -r sed -i 's|#include <ImGui/ImGuiPass.h>|#include <imgui/imgui.h>|'
echo "ImGui/ImGuiPass.h -> imgui/imgui.h"

# 3. Link targets.
#
#    ROS2 still exports a .Static, but the gems carved out of it never did -
#    they expose .API instead, so copying the .Static suffix across produces a
#    target that does not exist.
#
#    CommonFeaturesAtom keeps legacy AtomLyIntegration_CommonFeatures aliases,
#    but only up to .Editor; there is no legacy .Editor.Static, so that one
#    reference has to move to the current name.
#
#    Atom_Feature_Common exposes .Public rather than .Static, and the core ROS2
#    gem has .Editor.API but no .Editor.Static. Both are the same story: the
#    consumer-facing target was renamed, not removed.
#
#    ImGui has to move from PRIVATE to PUBLIC as well. KrakenEffectorComponent.h
#    includes imgui and both module .cpp files include that header, so the
#    include directory has to reach anything linking ROSConDemo.Static, not just
#    ROSConDemo.Static itself. Dropped here and re-added in the PUBLIC block
#    below so it is never listed twice.
sed -i '/^[[:space:]]*Gem::ImGui\.Static[[:space:]]*$/d' "${PROJECT}/Gem/CMakeLists.txt"
sed -i \
    -e 's|\(\s*\)Gem::ROS2\.Static|\1Gem::ROS2.Static\n\1Gem::ROS2Controllers.API\n\1Gem::ImGui.Static|' \
    -e 's|\(\s*\)Gem::ROS2\.Editor\.Static|\1Gem::ROS2.Editor.API\n\1Gem::ROS2Controllers.Editor.API|' \
    -e 's|Gem::AtomLyIntegration_CommonFeatures\.Editor\.Static|Gem::CommonFeaturesAtom.Editor.Static|' \
    -e 's|Gem::Atom_Feature_Common\.Static|Gem::Atom_Feature_Common.Public|' \
    "${PROJECT}/Gem/CMakeLists.txt"
echo "link targets -> ROS2Controllers.API, ImGui.Static PUBLIC, CommonFeaturesAtom, Atom_Feature_Common.Public"

# 4. target_depends_on_ros2_packages() still exists, but it is defined in a
#    helper module inside the ROS 2 Gem rather than by the engine. Gems shipped
#    in o3de-extras get it for free because the ROS 2 Gem is configured before
#    them; a project gem is not, so it has to be included explicitly. Guarded
#    on COMMAND so this keeps working if that ordering ever changes.
python3 - "${PROJECT}/Gem/CMakeLists.txt" <<'PY'
import sys

path = sys.argv[1]
with open(path) as fh:
    body = fh.read()

shim = """# Added by docker/roscondemo-setup.sh: the ROS 2 Gem defines
# target_depends_on_ros2_packages() in a helper module, and a project gem can be
# configured before that gem has been processed.
if(NOT COMMAND target_depends_on_ros2_packages)
    include(${ROS2_TARGET_DEPENDS_CMAKE})
endif()

"""

with open(path, "w") as fh:
    fh.write(shim + body)
print("cmake shim -> target_depends_on_ros2_packages")
PY

# Every rule above matches a single line, so a second use of the same symbol
# further down the file is silently left behind and only surfaces as a compile
# error minutes later. Cheaper to find it here.
stale=$(grep -rnE 'ROS2Names::|GetGameOrEditorComponent|frame->|ROS2Interface::Get\(\)->GetROSTimestamp|ImGui/ImGuiPass\.h|ROS2/VehicleDynamics/|ROS2::PidMotorController|ROS2::VehicleDynamics' "${PROJECT}/Gem/Source" || true)
if [ -n "${stale}" ]; then
    echo "ERROR: replaced APIs are still referenced after patching:" >&2
    printf '%s\n' "${stale}" >&2
    exit 1
fi
echo "patch check -> no stale references to replaced APIs"


set +u
. "/opt/ros/${ROS_DISTRO}/setup.sh"
set -u

say "registering the project"
"${O3DE_ROOT}/scripts/o3de.sh" register -pp "${PROJECT}"

say "configuring"
cd "${PROJECT}"
# Same two policies as o3de-setup.sh: O3DE_ROOT collides with CMake's reserved
# <PACKAGENAME>_ROOT convention, and O3DE uses no C++20 modules so the module
# dependency scan is a pass over the whole tree for nothing.
cmake -B build/linux -G "Ninja Multi-Config" \
    -DCMAKE_POLICY_DEFAULT_CMP0144=NEW \
    -DCMAKE_CXX_SCAN_FOR_MODULES=OFF \
    -DLY_DISABLE_TEST_MODULES=ON \
    -DLY_STRIP_DEBUG_SYMBOLS=ON \
    -DROS2_TARGET_DEPENDS_CMAKE="${O3DE_EXTRAS_ROOT}/Gems/ROS2/Code/ros2_target_depends.cmake"

say "building (${BUILD_CONFIG}) - hours on a first run"
cmake --build build/linux --config "${BUILD_CONFIG}" \
    --target Editor AssetProcessor ROSConDemo.GameLauncher ROSConDemo.Assets

cat <<EOF

Done.

  Editor:   ${PROJECT}/build/linux/bin/${BUILD_CONFIG}/Editor
  Launcher: ${PROJECT}/build/linux/bin/${BUILD_CONFIG}/ROSConDemo.GameLauncher -LoadLevel=playground

Levels are Main, playground and Test. Main is the full orchard and is heavy;
playground is a few trees and one Kraken, which is the one to iterate against.
Note the lowercase p - the level directory is 'playground' and -LoadLevel is
case sensitive here, whatever upstream's README says.
EOF
