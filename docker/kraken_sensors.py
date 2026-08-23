#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Give the Apple Kraken the sensors the localisation stack subscribes to.

Called by roscondemo-setup.sh against a pristine ROSConDemo checkout:

    python3 docker/kraken_sensors.py /o3de/ROSConDemo/Project

Upstream's Kraken carries a lidar and nothing else, because the demo drives on
ground truth and never estimates its own pose. Both robot prefabs are patched:
apple_kraken and apple_kraken_shiny spawn from v2, apple_kraken_rusty from v1.

The variances below are covariance, not noise. These sensors read the physics
exactly, and kraken_faults degrades a sensor by scaling the covariance it
arrives with, so the gem's default of zero would leave a degraded sensor
indistinguishable from a perfect one. They are a claim about the sensor, and
the point of the experiment is what happens when the claim is wrong.

GNSS answers with a position on the WGS84 ellipsoid, which it gets by asking
the Georeference bus where the level origin is, so the level needs an origin
component or every fix reads as null island.

Wheel odometry is the project's own component rather than the gem's, which
requires SkidSteeringModelService and would never activate on an Ackermann
robot. Its configuration is copied off the robot's own drive component so the
two agree on which axles exist; the covariances it claims are its C++ defaults,
because unlike the sensors above it is ours to set them on.

Prefabs are read and written with simplejson and Decimal, the way the ROS 2
Gem's own FrameConversion.py does it, so untouched floats round-trip as the
literals they were written as rather than as the nearest repr.
"""

import copy
import hashlib
import os
import sys
from decimal import Decimal
from pathlib import Path

import simplejson as json

# The datum kraken_sim publishes around and navsat_transform is configured
# with, so a run in O3DE and a run in the headless sim land in the same place.
# A different one puts the estimate and the ground truth in frames offset by
# the distance between the two.
DATUM = {"Latitude": Decimal("52.2297"), "Longitude": Decimal("21.0122"), "Altitude": Decimal("0.0")}

ROBOT_PREFABS = ("apple_kraken_v1", "apple_kraken_v2")
LEVEL_PREFABS = ("Levels/Main/Main.prefab", "Levels/playground/playground.prefab")


def sensor_configuration(frequency, msg_type, topic):
    return {
        "Frequency (HZ)": Decimal(frequency),
        "Publishers": {msg_type: {"Type": msg_type, "Topic": topic}},
    }


SENSORS = [
    (
        "ROS2ImuSensorComponent",
        {
            "SensorConfiguration": sensor_configuration("50.0", "sensor_msgs::msg::Imu", "imu/data"),
            "imuSensorConfiguration": {
                "AccelerationVariance": [Decimal("0.01")] * 3,
                "AngularVelocityVariance": [Decimal("0.0004")] * 3,
                "OrientationVariance": [Decimal("0.001")] * 3,
            },
        },
    ),
    (
        "ROS2GNSSSensorComponent",
        {"SensorConfiguration": sensor_configuration("10.0", "sensor_msgs::msg::NavSatFix", "gnss/fix")},
    ),
    (
        "ROS2OdometrySensorComponent",
        {"SensorConfiguration": sensor_configuration("50.0", "nav_msgs::msg::Odometry", "ground_truth/odom")},
    ),
]

# An O3DE entity looks down its own +Y axis, while base_link's forward is +X, so
# every camera carries the same -90 degree yaw and differs only in pitch. Euler
# angles compose as Rx*Ry*Rz (AZ::Quaternion::CreateFromEulerRadiansXYZ), so the
# middle term tilts the already-yawed camera downwards. The one worked example in
# the prefab is the follow camera at [0, 20, -90], which sits behind and above
# the robot and looks forward and down at it.
CAMERA_YAW = Decimal("-90.0")

# Colour only: a rendered depth buffer is exact, and a sensor that cannot be
# wrong is not worth simulating. Each channel is also its own render pipeline.
CAMERAS = [
    {
        # Above the lidar mount (z 0.966) and behind the front of the body
        # (x 2.70), looking level down the row. 60 degrees vertical is about 75
        # horizontal at 4:3, which spans the aisle two metres out and still
        # leaves enough angular resolution to resolve a person down the row.
        "frame": "camera_front",
        "translate": ["2.35", "0.0", "1.40"],
        "pitch": "0.0",
        "vertical_fov": "60.0",
        "far_clip": "100.0",
    },
    {
        # Pitched into the ground ahead of the front axle. Orchard floor is the
        # one surface always in view and always textured, so it is where frame
        # to frame parallax actually lives; a camera down the row sees a
        # self-similar tunnel and degenerates.
        "frame": "camera_ground",
        "translate": ["2.65", "0.0", "0.90"],
        "pitch": "45.0",
        "vertical_fov": "60.0",
        "far_clip": "30.0",
    },
]


def enabled_cameras():
    """Which cameras publish, read from KRAKEN_CAMERAS: 'all', 'none', or a
    comma separated list of frame names. A camera with publishing off keeps its
    entity and its pose, so the three configurations differ only in the render,
    which is the thing being priced."""
    setting = os.environ.get("KRAKEN_CAMERAS", "all").strip()
    known = {camera["frame"] for camera in CAMERAS}
    if setting == "all":
        return known
    if setting == "none":
        return set()
    chosen = {name.strip() for name in setting.split(",") if name.strip()}
    unknown = chosen - known
    if unknown:
        raise SystemExit(
            "KRAKEN_CAMERAS names no such camera: %s (have %s)"
            % (", ".join(sorted(unknown)), ", ".join(sorted(known)))
        )
    return chosen


def camera_publishers(frame):
    """Keys are the gem's own configuration names, not message types: the camera
    publishes four topics off one component and looks each up by name."""
    image = "sensor_msgs::msg::Image"
    info = "sensor_msgs::msg::CameraInfo"
    return {
        "Color Image": {"Type": image, "Topic": f"{frame}/image_color"},
        "Color Camera Info": {"Type": info, "Topic": f"{frame}/camera_info"},
        "Depth Image": {"Type": image, "Topic": f"{frame}/image_depth"},
        "Depth Camera Info": {"Type": info, "Topic": f"{frame}/depth_camera_info"},
    }

# Measured off the prefab's own transforms: the front axle sits 2.2 m ahead of
# the rear, the wheels 0.35 m either side of centre, and the axles ride 0.30 m
# above the ground. The demo fills none of this in, so the drive model steers on
# the class defaults of 2.0, 1.0 and 0.35 - close enough to drive with, but
# odometry integrating a 0.35 m radius would over-report every metre by a sixth.
GEOMETRY = {"Wheelbase": Decimal("2.2"), "Track": Decimal("0.7"), "WheelRadius": Decimal("0.3")}


def wheel_odometry(components):
    model = next(
        c["m_template"]
        for c in components.values()
        if c.get("m_template", {}).get("$type") == "AckermannVehicleModelComponent"
    )
    configuration = copy.deepcopy(model["VehicleConfiguration"])
    configuration["Wheelbase"] = GEOMETRY["Wheelbase"]
    configuration["Track"] = GEOMETRY["Track"]
    for axle in configuration["AxlesConfigurations"]:
        axle["WheelRadius"] = GEOMETRY["WheelRadius"]
    return {
        "SensorConfiguration": sensor_configuration("50.0", "nav_msgs::msg::Odometry", "wheel/odom"),
        "Vehicle configuration": configuration,
    }


def component_id(*parts):
    """Prefab component keys are unique uint64s. Derived from the name so that
    re-running rewrites the same prefab rather than growing a second copy of
    every sensor."""
    digest = hashlib.sha256("|".join(parts).encode()).digest()
    return int.from_bytes(digest[:8], "big") >> 1


def entity_id(*parts):
    """Entity keys are the same idea, but the prefab's own run in the low 2^48,
    so stay there rather than picking a number the editor renumbers on load."""
    digest = hashlib.sha256("|".join(parts).encode()).digest()
    return int.from_bytes(digest[:8], "big") % (1 << 47)


def read(path):
    with open(path) as fh:
        return json.load(fh, parse_float=Decimal)


def write(path, data):
    with open(path, "w") as fh:
        json.dump(data, fh, indent=4)
        fh.write("\n")


def add_sensors(path):
    data = read(path)
    body = next(e for e in data["Entities"].values() if e.get("Name") == "base_link")
    components = body["Components"]
    present = {c.get("m_template", {}).get("$type") for c in components.values()}
    added = []
    sensors = SENSORS + [("KrakenWheelOdometryComponent", wheel_odometry(components))]
    for type_name, template in sensors:
        if type_name in present:
            continue
        identifier = component_id(path.name, type_name)
        # Sensors have no editor component of their own, so they are carried by
        # a wrapper, the same way the lidar already on this robot is.
        components[f"Component_[{identifier}]"] = {
            "$type": "GenericComponentWrapper",
            "Id": identifier,
            "m_template": {"$type": type_name, **template},
        }
        added.append(type_name)
    if stop_publishing_true_odometry(components):
        added.append("ground truth odom transform off")
    added += add_cameras(data, path.name)
    write(path, data)
    return added


def camera_entity(prefab_name, camera, key, parent, publishing):
    """Cameras are the first thing here that needs an entity of its own rather
    than another component on base_link: the pose is the whole point, and a
    component cannot carry one."""
    frame = camera["frame"]

    def identifier(part):
        return component_id(prefab_name, frame, part)

    return {
        "Id": key,
        "Name": frame,
        "Components": {
            f"Component_[{identifier('transform')}]": {
                "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
                "Id": identifier("transform"),
                "Parent Entity": parent,
                "Transform Data": {
                    "Translate": [Decimal(v) for v in camera["translate"]],
                    "Rotate": [Decimal("0.0"), Decimal(camera["pitch"]), CAMERA_YAW],
                },
            },
            f"Component_[{identifier('frame')}]": {
                "$type": "ROS2FrameEditorComponent",
                "Id": identifier("frame"),
                "ROS2FrameConfiguration": {
                    # Same strategy as the lidar sibling: take the namespace
                    # from base_link above rather than inventing another level.
                    "Namespace Configuration": {"Namespace Strategy": 1},
                    "Frame Name": frame,
                },
            },
            f"Component_[{identifier('camera')}]": {
                # Added bare rather than wrapped, unlike the sensors above: the
                # camera does have an editor component, and it and the runtime
                # one it builds both claim the ROS2CameraSensor service.
                "$type": "ROS2CameraSensorEditorComponent",
                "Id": identifier("camera"),
                "SensorConfig": {
                    "Frequency (HZ)": Decimal("10.0"),
                    # Set from KRAKEN_CAMERAS. False skips the render too, not
                    # just the publish: the gem tests this in the tick callback
                    # before FrequencyTick, which is the only thing that asks
                    # for a frame. The render target stays allocated either way.
                    "Publishing Enabled": publishing,
                    "Publishers": camera_publishers(frame),
                },
                "CameraSensorConfig": {
                    "VerticalFieldOfViewDeg": Decimal(camera["vertical_fov"]),
                    "Width": 640,
                    "Height": 480,
                    "Color": True,
                    "Depth": False,
                    "ClipNear": Decimal("0.1"),
                    "ClipFar": Decimal(camera["far_clip"]),
                },
            },
        },
    }


def add_cameras(data, prefab_name):
    body = next(e for e in data["Entities"].values() if e.get("Name") == "base_link")
    order = next(
        c for c in body["Components"].values() if c.get("$type") == "EditorEntitySortComponent"
    )["Child Entity Order"]
    enabled = enabled_cameras()
    added = []
    for camera in CAMERAS:
        key = f"Entity_[{entity_id(prefab_name, camera['frame'])}]"
        fresh = key not in data["Entities"]
        # Rewritten rather than skipped when present, the way the georeference
        # origin is, so re-running with a different KRAKEN_CAMERAS moves an
        # already patched prefab to the new selection instead of keeping the old.
        data["Entities"][key] = camera_entity(
            prefab_name, camera, key, body["Id"], camera["frame"] in enabled
        )
        if fresh:
            order.append(key)
            added.append(camera["frame"])
    return added


def stop_publishing_true_odometry(components):
    """The frame component on base_link has no ROS 2 frame above it, so it
    publishes odom -> base_link from the entity's real world pose. Leaving that
    on hands the filter the answer: the odom frame would be exact and no amount
    of wheel slip could ever show up in it. Dead reckoning owns that edge."""
    frame = next(
        c
        for c in components.values()
        if c.get("$type") == "ROS2FrameEditorComponent"
        and c["ROS2FrameConfiguration"].get("Frame Name") == "base_link"
    )
    was_publishing = frame["ROS2FrameConfiguration"].get("Publish Transform", True)
    frame["ROS2FrameConfiguration"]["Publish Transform"] = False
    return was_publishing


def add_georeference(path):
    data = read(path)
    container = data["ContainerEntity"]
    components = container["Components"]
    identifier = component_id(path.name, "georeference")
    key = f"Component_[{identifier}]"
    was_absent = key not in components
    # Written unconditionally rather than skipped when present, so that a level
    # carrying an older origin gets the current one instead of keeping it.
    components[key] = {
        "$type": "GeoReferenceLevelEditorComponent",
        "Id": identifier,
        "Controller": {
            "Configuration": {
                "EnuOriginWGS84": DATUM,
                # The ENU origin is read off an entity's world transform, and
                # the transform holding it has no initialiser - naming no
                # entity leaves it uninitialised, which reads out as a fix of
                # nan rather than as identity. The level's own container entity
                # sits at the world origin, which is the frame wanted anyway.
                "EnuOriginLocationEntityId": container["Id"],
            }
        },
    }
    write(path, data)
    return was_absent


def main(argv):
    if len(argv) != 2:
        print(f"usage: {Path(argv[0]).name} <ROSConDemo/Project>", file=sys.stderr)
        return 2
    project = Path(argv[1])

    for robot in ROBOT_PREFABS:
        added = add_sensors(project / "Assets" / "Kraken" / robot / f"{robot}.prefab")
        print(f"{robot} -> {', '.join(added) if added else 'sensors already present'}")

    for level in LEVEL_PREFABS:
        path = project / level
        placed = add_georeference(path)
        print(f"{path.name} -> georeference origin {'added' if placed else 'updated'}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
