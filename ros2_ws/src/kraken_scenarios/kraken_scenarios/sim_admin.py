"""Spawn and remove Kraken robots in the running O3DE level.

Two things about the demo's spawner are easy to get wrong:

* The spawn point name goes in the request's ``xml`` field. ``reference_frame``
  is only checked for the literal "wgs84", so putting the name there leaves
  ``xml`` empty and the robot appears at the level's default pose - the world
  origin - rather than the row that was asked for.
* The service can take longer to reply than a client is willing to wait while
  still succeeding. Retrying a "no reply" stacks robots on one spawn point.
  Never retry; list the topics instead.
"""
import argparse

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node

ROBOT_STEMS = ("apple_kraken_rusty", "apple_kraken_shiny")


def call(node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def spawn(node, spawn_point, namespace):
    client = node.create_client(SpawnEntity, "/spawn_entity")
    client.wait_for_service(timeout_sec=30)

    request = SpawnEntity.Request()
    request.name = "apple_kraken_rusty"
    request.robot_namespace = namespace
    request.xml = spawn_point

    result = call(node, client, request, 60.0)
    if result is None:
        print("spawn: no reply within 60 s - check the topic list, do not retry")
        return 1
    print("spawn: %s %s" % (result.success, result.status_message))
    return 0 if result.success else 1


def purge(node):
    client = node.create_client(DeleteEntity, "/delete_entity")
    client.wait_for_service(timeout_sec=30)

    removed = []
    for stem in ROBOT_STEMS:
        for index in range(26):
            name = "%s_%d" % (stem, index)
            result = call(node, client, DeleteEntity.Request(name=name), 10.0)
            if result is not None and result.success:
                removed.append(name)

    print("deleted %d: %s" % (len(removed), ", ".join(removed) or "none"))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    spawn_parser = sub.add_parser("spawn")
    spawn_parser.add_argument("spawn_point", help="a level spawn point, e.g. line4")
    spawn_parser.add_argument("namespace")
    sub.add_parser("purge")
    args = parser.parse_args()

    rclpy.init()
    node = Node("sim_admin")
    try:
        code = spawn(node, args.spawn_point, args.namespace) if args.action == "spawn" else purge(node)
    finally:
        rclpy.shutdown()
    return code
