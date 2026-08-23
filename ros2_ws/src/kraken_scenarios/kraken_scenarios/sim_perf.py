# SPDX-License-Identifier: BSD-3-Clause
"""Price the O3DE cameras without consuming a frame.

Run it against a level that is already up, the way `simulator:=o3de` expects:

    ros2 run kraken_scenarios sim_perf --label all --duration 30

O3DE has no real_time_factor - that is the headless sim's - so it renders
against wall clock, and a render pass it cannot afford shows up as its tick
slipping. Every sensor it publishes slips with it, which makes the achieved rate
of a sensor with nothing to do with cameras, the IMU at a nominal 50 Hz, the
cheapest honest measure of what a camera costs. No GPU counters, no O3DE
instrumentation, nothing to build.

Camera rate is read off camera_info rather than the image: same component, same
tick, a few hundred bytes instead of very nearly a megabyte. Subscribing to the
images is the one thing here that would change the number being measured, so it
is off unless --images asks for it, and then transport is what is being read,
not the render.

One run says nothing. The selection is baked into the prefab, so each point
needs the prefab regenerated and O3DE restarted:

    KRAKEN_CAMERAS=none         python3 docker/kraken_sensors.py <project>
    KRAKEN_CAMERAS=camera_front python3 docker/kraken_sensors.py <project>
    KRAKEN_CAMERAS=all          python3 docker/kraken_sensors.py <project>

The reference rate across those three is the answer; the absolute number is a
property of the machine and travels nowhere.
"""
import argparse
import collections
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu

# Best effort is what a reader asks for, not what it forces: it matches a
# reliable publisher too, and never adds a retransmit to the thing being timed.
PROBE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class Probe(Node):
    def __init__(self):
        super().__init__("sim_perf")
        self.counts = collections.Counter()
        self.payload = collections.Counter()
        self.watched = {}
        self.clock_first = None
        self.clock_last = None
        self.create_subscription(Clock, "/clock", self._on_clock, PROBE_QOS)

    def watch(self, topic, message):
        if topic in self.watched:
            return
        self.watched[topic] = self.create_subscription(
            message, topic, self._counter(topic), PROBE_QOS
        )

    def _counter(self, topic):
        def callback(msg):
            self.counts[topic] += 1
            data = getattr(msg, "data", None)
            if data is not None:
                self.payload[topic] += len(data)

        return callback

    def _on_clock(self, msg):
        seconds = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.clock_first is None:
            self.clock_first = seconds
        self.clock_last = seconds

    def reset(self):
        self.counts.clear()
        self.payload.clear()
        self.clock_first = None
        self.clock_last = None


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def qualify(namespace, topic):
    namespace = namespace.strip("/")
    return "/%s/%s" % (namespace, topic) if namespace else "/%s" % topic


def discover_cameras(node, namespace):
    """Read the cameras off the graph rather than keeping a second copy of the
    list that kraken_sensors.py already owns. Ends with /camera_info, so the
    depth_camera_info sibling is not mistaken for another camera."""
    prefix = qualify(namespace, "")
    found = [
        name
        for name, types in node.get_topic_names_and_types()
        if name.startswith(prefix)
        and name.endswith("/camera_info")
        and "sensor_msgs/msg/CameraInfo" in types
    ]
    return sorted(found)


def summarise(node, wall, reference, nominal_hz, label):
    rates = {
        topic: round(count / wall, 2) for topic, count in sorted(node.counts.items())
    }
    throughput = {
        topic: round(total / wall / 1e6, 2)
        for topic, total in sorted(node.payload.items())
        if total
    }
    advanced = (
        None
        if node.clock_first is None or node.clock_last is None
        else node.clock_last - node.clock_first
    )
    achieved = rates.get(reference, 0.0)
    return {
        "label": label,
        "wall_s": round(wall, 2),
        # None when nothing publishes /clock, which is worth seeing rather than
        # reporting as a real time factor of zero.
        "real_time_factor": None if advanced is None else round(advanced / wall, 3),
        # A headless sim left running beside O3DE also publishes /clock, and
        # reading the wrong one is silent. Naming the publisher makes it not.
        "clock_publishers": sorted(
            info.node_name for info in node.get_publishers_info_by_topic("/clock")
        ),
        "reference_topic": reference,
        "reference_hz": achieved,
        "reference_nominal_hz": nominal_hz,
        "reference_fraction": round(achieved / nominal_hz, 3) if nominal_hz else None,
        "rates_hz": rates,
        "throughput_mb_s": throughput,
    }


def report(summary):
    factor = summary["real_time_factor"]
    print(
        "sim_perf: label=%s wall=%.1f s clock=%s"
        % (
            summary["label"] or "(none)",
            summary["wall_s"],
            "no /clock" if factor is None else "%.3f x" % factor,
        )
    )
    publishers = summary["clock_publishers"]
    if len(publishers) > 1:
        print("  /clock has %d publishers (%s) - the factor above is not"
              " attributable to one simulator" % (len(publishers), ", ".join(publishers)))
    elif publishers:
        print("  /clock from %s" % publishers[0])
    fraction = summary["reference_fraction"]
    print(
        "  reference %-40s %6.2f Hz of %.1f nominal%s"
        % (
            summary["reference_topic"],
            summary["reference_hz"],
            summary["reference_nominal_hz"],
            "" if fraction is None else "  (%.0f%%)" % (100 * fraction),
        )
    )
    for topic, rate in summary["rates_hz"].items():
        if topic == summary["reference_topic"]:
            continue
        throughput = summary["throughput_mb_s"].get(topic)
        print(
            "  %-50s %6.2f Hz%s"
            % (topic, rate, "" if throughput is None else "  %.2f MB/s" % throughput)
        )
    if not summary["rates_hz"].get(summary["reference_topic"]):
        print("  reference never published - wrong namespace, or the level is not up")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--namespace", default="kraken1",
                        help="robot namespace the sensors publish under")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="wall seconds to sample over")
    parser.add_argument("--settle", type=float, default=5.0,
                        help="wall seconds to discover and let rates steady before counting")
    parser.add_argument("--reference", default="imu/data",
                        help="sensor whose achieved rate stands in for the tick")
    parser.add_argument("--reference-hz", type=float, default=50.0,
                        help="what that sensor is configured to publish at")
    parser.add_argument("--images", action="store_true",
                        help="also subscribe to the images, which measures transport "
                             "rather than the render, and adds load while doing it")
    parser.add_argument("--label", default="",
                        help="recorded in the report, e.g. the KRAKEN_CAMERAS value")
    parser.add_argument("--report", help="write the summary here as json")
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    reference = qualify(args.namespace, args.reference)
    node.watch(reference, Imu)
    try:
        spin_for(node, args.settle)
        for topic in discover_cameras(node, args.namespace):
            node.watch(topic, CameraInfo)
            if args.images:
                node.watch(topic.rsplit("/", 1)[0] + "/image_color", Image)
        # Discovery and the first frames are not the steady state being priced.
        spin_for(node, args.settle)

        node.reset()
        started = time.monotonic()
        spin_for(node, args.duration)
        summary = summarise(
            node, time.monotonic() - started, reference, args.reference_hz, args.label
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    report(summary)
    if args.report:
        with open(args.report, "w") as handle:
            json.dump(summary, handle, indent=2)
            handle.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
