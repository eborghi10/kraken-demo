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
    write(path, data)
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
