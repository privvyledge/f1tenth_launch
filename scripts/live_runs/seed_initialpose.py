#!/usr/bin/env python3
"""Seed a localizer's initial pose during a bag replay.

    python3 seed_initialpose.py --ns gosling1 --x -0.008 --y -0.004 --yaw -0.951

`ros2 topic pub` cannot do this job. It stamps the message with WALL time, and
AMCL running under `use_sim_time` then tries to look the pose up at a stamp
~13000 s in the future of the bag clock and drops it with

    Failed to transform initial pose in time (Lookup would require
    extrapolation into the future)

leaving AMCL quietly sitting on the (0,0,0) default from localizer_amcl.yaml —
a localizer that looks healthy and is simply in the wrong place.

So: run as a real node with use_sim_time, wait until odom->base_link is
actually in the buffer, and stamp the pose with time 0, which tf2 reads as
"latest available" and therefore cannot extrapolate past.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="gosling1")
    ap.add_argument("--x", type=float, default=-0.008)
    ap.add_argument("--y", type=float, default=-0.004)
    ap.add_argument("--yaw", type=float, default=-0.951)
    ap.add_argument("--times", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--sigma-xy", type=float, default=0.05)
    ap.add_argument("--sigma-yaw", type=float, default=0.02)
    # Defaults keep the bag-replay behaviour this script was written for. Pass
    # `--ns "" --no-use-sim-time` on a LIVE stack: a raw
    # `ros2 launch f1tenth_launch bringup.launch.py` leaves
    # use_f1tenth_namespace False, so TF is on the bare /tf and there is no
    # /clock publisher to drive sim time.
    ap.add_argument("--use-sim-time", dest="use_sim_time",
                    action=argparse.BooleanOptionalAction, default=True,
                    help="sim time for bag replay (default); "
                         "--no-use-sim-time for a live robot")
    args = ap.parse_args()

    # The /tf remap is load-bearing and is the same trap 00_env.sh's tf_has_edge
    # documents: tf2_ros' TransformListener subscribes to the ABSOLUTE names
    # /tf and /tf_static, so putting the node in a namespace moves the node but
    # not its TF subscriptions. It would then wait forever on a /tf nobody
    # publishes and report "odom->base_link never arrived" against a healthy tree.
    #
    # An empty --ns is the un-namespaced live stack. Remapping /tf to //tf then
    # matches nothing and the node waits out its timeout against a perfectly
    # healthy tree, so skip the remap entirely in that case.
    init_args = ["--ros-args"]
    if args.ns:
        init_args += ["-r", f"/tf:=/{args.ns}/tf",
                      "-r", f"/tf_static:=/{args.ns}/tf_static"]
    rclpy.init(args=init_args if args.ns else None)
    # use_sim_time has to be set at construction: the clock is built when the
    # node is, so flipping the parameter afterwards leaves a system clock behind.
    node = Node("initialpose_seeder", namespace=args.ns or "/",
                parameter_overrides=[
                    Parameter("use_sim_time", Parameter.Type.BOOL,
                              args.use_sim_time)])

    buf = Buffer()
    TransformListener(buf, node)
    pub = node.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)

    # Wait for the replayed odom->base_link. Seeding before it arrives is the
    # same failure by a different route: AMCL has no odometry to attach the
    # pose to and discards it.
    #
    # The timeout MUST be wall time. get_clock() here is the sim clock, which
    # reads 0 until the first /clock message and then jumps to bag time — so a
    # sim-time deadline of "now + 60 s" is computed as 60 and blows through the
    # instant the bag clock (~1.8e9) arrives, aborting on the first spin.
    deadline = time.monotonic() + args.timeout
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("odom", "base_link", Time()):
            break
        if time.monotonic() > deadline:
            node.get_logger().error("odom->base_link never arrived; not seeding")
            rclpy.shutdown()
            return 1

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"          # stamp left at 0 == "latest" for tf2
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    msg.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    cov = [0.0] * 36
    cov[0] = cov[7] = args.sigma_xy ** 2
    cov[35] = args.sigma_yaw ** 2
    msg.pose.covariance = cov

    # Repeat: the first publish routinely lands before AMCL's subscription is
    # matched, and a dropped seed is invisible — AMCL keeps running on the
    # (0,0,0) YAML default and reports healthy.
    for _ in range(args.times):
        pub.publish(msg)
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(
        f"seeded ({args.x:+.3f}, {args.y:+.3f}, {math.degrees(args.yaw):+.2f} deg)")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
