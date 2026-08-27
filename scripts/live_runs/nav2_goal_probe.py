#!/usr/bin/env python3
"""Send NavigateToPose goals during a bag replay and measure what Nav2 does.

    python3 nav2_goal_probe.py --ns gosling1 \
        --goals-csv /mnt/shared_dir/maps/20260805/truth_mapping_drive_170025.csv \
        --goal-rows 150,250,350 --out-json /mnt/shared_dir/run/nav2_probe.json

LIVE ROBOT
----------
    python3 nav2_goal_probe.py --ns "" --no-use-sim-time \
        --goals "x,y,yaw" --out-json /mnt/shared_dir/run/nav2_live.json

Both flags are required and neither is the default. `--ns ""` because bringup
defaults use_f1tenth_namespace:=False, and `--no-use-sim-time` because a live
stack publishes no /clock -- a sim-time node sees time frozen at 0, so every
timeout here either fires instantly or never. Defaults were left at the replay
values so existing invocations are unchanged.

`--cmd-topic` defaults to cmd_vel_nav2, which pairs with launching Nav2 as
`cmd_vel_topic:=cmd_vel_nav2`: the smoother AND behavior_server both move off
cmd_vel (bug-125), so nothing reaches the VESC and the car cannot move. That is
the safe parked configuration. For a real closed-loop drive, launch with the
default cmd_vel and pass --cmd-topic cmd_vel.

WHY A NODE AND NOT `ros2 action send_goal`
------------------------------------------
The CLI can send the goal, but it cannot answer the questions this test exists
to ask: how long the planner took, whether the path was continuous, whether the
controller emitted bounded velocity, and whether a recovery fired. It also has
the same wall-clock trap as `ros2 topic pub` under a bag replay — see
seed_initialpose.py. This runs as a real sim-time node instead.

WHAT THE MEASUREMENT MEANS
--------------------------
The bag drives the pose, so the loop is OPEN: the controller steers toward the
path from wherever the bag has put the car, and the goal is never reached.
That is expected and is not a failure. What is being tested is that
bt_navigator accepts a goal, planner_server returns a path through free space
inside its 5 s budget, controller_server emits continuous bounded velocity, and
nothing crashes, hangs the lifecycle manager, or spins at 100 % CPU.

GOALS COME FROM POSES THE CAR ACTUALLY OCCUPIED
-----------------------------------------------
Those are guaranteed to be in mapped free space, so a failed plan is a Nav2
problem and never a "you asked for a goal inside a wall" problem. Generate the
CSV with goal_poses_from_bag.py; any (x, y, yaw) triple can be passed with
--goals instead.

**Do not feed this maps/*/truth_<bag>.csv.** That file is the map->odom
TRANSFORM over time, not a robot pose — on mapping_drive_170025 it never leaves
a 0.3 m ball around the origin, so every goal lands on top of the robot and the
test measures nothing. It cost one full run on 2026-08-06 to notice.

WAIT FOR ACTIVE, NOT FOR TOPICS
-------------------------------
nav2's costmap topics and action servers are created at CONFIGURE. They are
therefore visible, and the action server rejects every goal with no log line at
all, for the ~2 s until the lifecycle manager ACTIVATES the servers. That looks
exactly like a Nav2 defect and is not one — it cost the first run of this probe.
So the gate is each server's own lifecycle state, not the presence of anything.
"""

import argparse
import csv
import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Path
from nav2_msgs.action import BackUp, NavigateToPose
from tf2_ros import Buffer, TransformListener

# action_msgs/msg/GoalStatus
STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
          4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def path_length(path_msg):
    total = 0.0
    poses = path_msg.poses
    for a, b in zip(poses, poses[1:]):
        total += math.hypot(b.pose.position.x - a.pose.position.x,
                            b.pose.position.y - a.pose.position.y)
    return total


class Probe(Node):

    def __init__(self, ns, cmd_topic, use_sim_time=True):
        # use_sim_time has to be set at construction -- the clock is built when
        # the node is, so setting it afterwards leaves a node already bound to
        # the wrong clock. Same reason as seed_initialpose.py.
        super().__init__('nav2_goal_probe', namespace=ns or '/',
                         parameter_overrides=[
                             Parameter('use_sim_time', Parameter.Type.BOOL,
                                       use_sim_time)])
        self.ns = ns
        # An empty ns is the un-namespaced live stack (bringup defaults
        # use_f1tenth_namespace:=False). Building '/{ns}/plan' with ns '' gives
        # '//plan', which is not the same topic as '/plan' -- so every name has
        # to go through here rather than through an f-string.
        self.pfx = f'/{ns}' if ns else ''
        self.buf = Buffer()
        TransformListener(self.buf, self)

        self.client = ActionClient(self, NavigateToPose,
                                   f'{self.pfx}/navigate_to_pose')

        # Everything below is reset per goal by arm().
        self.plans = []          # (sim_t, n_poses, length_m)
        self.local_plans = 0
        self.cmds = []           # (sim_t, vx, wz)
        self.feedback = []       # (sim_t, distance_remaining, n_recoveries)
        self.recording = False

        self.create_subscription(Path, f'{self.pfx}/plan', self._on_plan, 10)
        self.create_subscription(Path, f'{self.pfx}/local_plan', self._on_local_plan, 10)
        self.create_subscription(Twist, f'{self.pfx}/{cmd_topic}', self._on_cmd, 10)

        # The recovery behaviours do not honour cmd_vel_topic — nav2_behaviors
        # creates its publisher on the relative name `cmd_vel` and neither this
        # repo's launch nor upstream navigation_launch.py remaps it. Watch the
        # plain topic separately so a recovery that escapes the diversion is
        # counted rather than inferred.
        self.raw_cmds = []
        self.create_subscription(Twist, f'{self.pfx}/cmd_vel', self._on_raw_cmd, 10)

    # -- sim clock -----------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- callbacks -----------------------------------------------------------
    def _on_plan(self, msg):
        if self.recording:
            self.plans.append((self.now(), len(msg.poses), path_length(msg)))

    def _on_local_plan(self, msg):
        if self.recording:
            self.local_plans += 1

    def _on_cmd(self, msg):
        if self.recording:
            self.cmds.append((self.now(), msg.linear.x, msg.angular.z))

    def _on_raw_cmd(self, msg):
        if self.recording:
            self.raw_cmds.append((self.now(), msg.linear.x, msg.angular.z))

    def on_feedback(self, fb):
        if self.recording:
            f = fb.feedback
            self.feedback.append((self.now(), float(f.distance_remaining),
                                  int(f.number_of_recoveries)))

    def arm(self):
        self.plans, self.cmds, self.feedback, self.local_plans = [], [], [], 0
        self.raw_cmds = []
        self.recording = True

    def disarm(self):
        self.recording = False

    # -- helpers -------------------------------------------------------------
    def map_pose(self):
        """base_link in the map frame, or None."""
        try:
            t = self.buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def spin_until(self, predicate, wall_timeout, poll=0.05):
        """Spin until predicate() is true. Wall time on purpose: the sim clock
        stops dead if the replay stalls, and a sim-time deadline would then
        wait forever instead of reporting the stall."""
        deadline = time.monotonic() + wall_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=poll)
            if predicate():
                return True
        return False

    def spin_for(self, wall_seconds, predicate=None):
        deadline = time.monotonic() + wall_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate is not None and predicate():
                return True
        return False

    def wait_active(self, server, wall_timeout):
        """Block until <server> reports lifecycle state ACTIVE.

        A nav2 server's action server and costmap topics exist from CONFIGURE
        onward, but nav2_util::SimpleActionServer rejects every goal while
        inactive and logs nothing when it does. Asking the node its own state
        is the only gate that means what it looks like it means."""
        cli = self.create_client(GetState, f'{self.pfx}/{server}/get_state')
        if not cli.wait_for_service(timeout_sec=wall_timeout):
            self.get_logger().error(f'{server}/get_state never appeared')
            return False
        deadline = time.monotonic() + wall_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            fut = cli.call_async(GetState.Request())
            if self.spin_until(lambda: fut.done(), 5.0) and fut.result() is not None:
                if fut.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    return True
            self.spin_for(0.5)
        self.get_logger().error(f'{server} never reached ACTIVE')
        return False


def summarize(p, sent_sim, start_pose, goal):
    """Turn one goal's raw capture into the numbers the test is judged on."""
    out = {}
    out['plans'] = len(p.plans)
    out['local_plans'] = p.local_plans
    if p.plans:
        t0, n0, l0 = p.plans[0]
        out['first_plan_latency_s'] = round(t0 - sent_sim, 3)
        out['first_plan_poses'] = n0
        out['first_plan_length_m'] = round(l0, 3)
        out['plan_length_max_m'] = round(max(l for _, _, l in p.plans), 3)
    else:
        out['first_plan_latency_s'] = None

    cmds = p.cmds
    out['cmd_count'] = len(cmds)
    if cmds:
        span = cmds[-1][0] - cmds[0][0]
        gaps = [b[0] - a[0] for a, b in zip(cmds, cmds[1:])]
        out['cmd_hz'] = round(len(cmds) / span, 2) if span > 0 else None
        out['cmd_gap_max_s'] = round(max(gaps), 3) if gaps else None
        out['cmd_vx_min'] = round(min(c[1] for c in cmds), 3)
        out['cmd_vx_max'] = round(max(c[1] for c in cmds), 3)
        out['cmd_wz_absmax'] = round(max(abs(c[2]) for c in cmds), 3)
        out['cmd_nonzero'] = sum(1 for c in cmds if abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6)
    out['raw_cmd_vel_count'] = len(p.raw_cmds)
    if p.raw_cmds:
        out['raw_cmd_vel_nonzero'] = sum(
            1 for c in p.raw_cmds if abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6)
        out['raw_cmd_vel_vx_absmax'] = round(max(abs(c[1]) for c in p.raw_cmds), 3)

    if p.feedback:
        out['recoveries'] = max(f[2] for f in p.feedback)
        out['distance_remaining_first'] = round(p.feedback[0][1], 3)
        out['distance_remaining_last'] = round(p.feedback[-1][1], 3)
        out['feedback_count'] = len(p.feedback)
    else:
        out['recoveries'] = None
        out['feedback_count'] = 0

    if start_pose:
        out['start_pose'] = [round(v, 3) for v in start_pose]
        out['straight_line_to_goal_m'] = round(
            math.hypot(goal[0] - start_pose[0], goal[1] - start_pose[1]), 3)
    return out


def load_goals(args):
    if args.goals:
        goals = []
        for triple in args.goals.split(';'):
            x, y, yaw = (float(v) for v in triple.split(','))
            goals.append((x, y, yaw))
        return goals

    rows = []
    with open(args.goals_csv) as fh:
        for row in csv.DictReader(fh):
            rows.append((float(row['x']), float(row['y']), float(row['yaw'])))
    if not rows:
        raise SystemExit(f'no rows in {args.goals_csv}')

    if args.goal_rows:
        idx = [int(i) for i in args.goal_rows.split(',')]
    elif len(rows) <= 8:
        # A CSV this short is a pre-picked goal list (goal_poses_from_bag.py),
        # not a trajectory to sample. Taking fractions of it returns the same
        # row twice.
        idx = list(range(len(rows)))
    else:
        n = len(rows)
        idx = [int(n * f) for f in (0.35, 0.6, 0.85)]
    return [rows[min(i, len(rows) - 1)] for i in idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='gosling1',
                    help='pass --ns "" for a live un-namespaced bringup '
                         '(use_f1tenth_namespace defaults False)')
    # Defaults True so every existing bag-replay invocation is unchanged.
    # A live stack has no /clock, so a sim-time node sees time frozen at 0 and
    # every timeout below either fires instantly or never.
    ap.add_argument('--use-sim-time', dest='use_sim_time',
                    action='store_true', default=True,
                    help='default; for a bag replay')
    ap.add_argument('--no-use-sim-time', dest='use_sim_time',
                    action='store_false',
                    help='REQUIRED on a live robot')
    ap.add_argument('--cmd-topic', default='cmd_vel_nav2',
                    help='velocity smoother output to watch (the launch arg '
                         'cmd_vel_topic; cmd_vel_nav2 in a dry/bag run)')
    ap.add_argument('--goals-csv',
                    help='map-frame truth CSV (stamp,x,y,yaw) to draw goals from')
    ap.add_argument('--goal-rows', help='comma-separated row indices into the CSV')
    ap.add_argument('--goals', help='explicit goals: "x,y,yaw;x,y,yaw"')
    ap.add_argument('--goal-timeout', type=float, default=45.0,
                    help='wall seconds to let each goal run before cancelling')
    ap.add_argument('--settle', type=float, default=20.0,
                    help='wall seconds to wait for map->base_link before starting')
    ap.add_argument('--gap', type=float, default=5.0,
                    help='wall seconds between goals')
    ap.add_argument('--exercise-backup', action='store_true',
                    help='after the goals, call the BackUp recovery directly '
                         'and report which topic its velocity came out on')
    ap.add_argument('--out-json', default='')
    args = ap.parse_args()

    if not args.goals and not args.goals_csv:
        raise SystemExit('pass --goals or --goals-csv')

    goals = load_goals(args)

    # Same /tf remap trap as seed_initialpose.py: TransformListener subscribes
    # to the absolute /tf, so a namespaced node without this listens to a topic
    # nobody publishes and reports an empty tree.
    # An empty --ns is the un-namespaced live stack: remapping /tf to //tf
    # would point the listener at a topic nobody publishes and report an empty
    # tree, so skip the remap entirely in that case.
    init_args = None
    if args.ns:
        init_args = ['--ros-args',
                     '-r', f'/tf:=/{args.ns}/tf',
                     '-r', f'/tf_static:=/{args.ns}/tf_static']
    rclpy.init(args=init_args)
    p = Probe(args.ns, args.cmd_topic, use_sim_time=args.use_sim_time)
    log = p.get_logger()

    report = {'ns': args.ns, 'cmd_topic': args.cmd_topic, 'goals': []}

    # The action server has to exist before anything else is worth trying.
    if not p.client.wait_for_server(timeout_sec=args.settle):
        log.error(f'{p.pfx}/navigate_to_pose action server never appeared')
        report['fatal'] = 'no navigate_to_pose action server'
        _finish(report, args, p, rc=1)

    # ...and existing is not the same as accepting goals. See the header.
    for server in ('bt_navigator', 'planner_server', 'controller_server'):
        if not p.wait_active(server, args.settle):
            report['fatal'] = f'{server} not ACTIVE'
            _finish(report, args, p, rc=1)
    log.info('bt_navigator, planner_server and controller_server are ACTIVE')

    # And map->base_link has to be real, or every goal fails for a reason that
    # has nothing to do with Nav2.
    if not p.spin_until(lambda: p.map_pose() is not None, args.settle):
        log.error('map->base_link never arrived — localizer not up or not seeded')
        report['fatal'] = 'no map->base_link'
        _finish(report, args, p, rc=1)

    log.info(f'localized at {p.map_pose()}; sending {len(goals)} goals')

    for i, (gx, gy, gyaw) in enumerate(goals):
        entry = {'index': i, 'goal': [round(gx, 3), round(gy, 3), round(gyaw, 3)]}
        start_pose = p.map_pose()

        msg = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        # Stamp 0 = "latest available", the same reason seed_initialpose.py
        # uses it: a wall-clock stamp is a future extrapolation under sim time.
        ps.header.stamp = Time(seconds=0).to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        qx, qy, qz, qw = yaw_to_quat(gyaw)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        msg.pose = ps

        p.arm()
        sent_wall = time.monotonic()
        sent_sim = p.now()
        send_future = p.client.send_goal_async(msg, feedback_callback=p.on_feedback)

        if not p.spin_until(lambda: send_future.done(), 15.0):
            entry['outcome'] = 'no response to send_goal in 15 s'
            log.error(f'goal {i}: {entry["outcome"]}')
            p.disarm()
            report['goals'].append(entry)
            continue

        handle = send_future.result()
        entry['accept_latency_s'] = round(time.monotonic() - sent_wall, 3)
        entry['accepted'] = bool(handle.accepted)
        if not handle.accepted:
            entry['outcome'] = 'REJECTED by bt_navigator'
            log.error(f'goal {i}: rejected')
            p.disarm()
            report['goals'].append(entry)
            continue

        log.info(f'goal {i} accepted -> ({gx:.2f}, {gy:.2f}); watching '
                 f'{args.goal_timeout:.0f} s')
        result_future = handle.get_result_async()
        finished = p.spin_for(args.goal_timeout, lambda: result_future.done())

        if finished and result_future.result() is not None:
            code = result_future.result().status
            entry['status'] = STATUS.get(code, code)
            entry['outcome'] = f'terminated: {entry["status"]}'
        else:
            entry['status'] = 'STILL_RUNNING'
            entry['outcome'] = 'still navigating at timeout (expected: the bag '
            entry['outcome'] += 'drives the pose, so the goal is never reached)'
            cancel_future = handle.cancel_goal_async()
            p.spin_until(lambda: cancel_future.done(), 10.0)
            p.spin_for(2.0)

        entry.update(summarize(p, sent_sim, start_pose, (gx, gy)))
        p.disarm()
        report['goals'].append(entry)
        log.info(f'goal {i}: {json.dumps(entry)}')

        p.spin_for(args.gap)

    if args.exercise_backup:
        report['backup'] = exercise_backup(p, log)

    _finish(report, args, p, rc=0)


def exercise_backup(p, log):
    """Call the BackUp recovery directly and see which topic carries its
    velocity.

    Waiting for the behaviour tree to reach BackUp on its own does not work as
    a test: its RoundRobin clears both costmaps and waits 5 s first, so a 30 s
    goal window ends before BackUp is ever ticked, and the run then reports a
    clean cmd_vel that proves nothing. Calling the action settles it in 2 s."""
    out = {}
    cli = ActionClient(p, BackUp, f'/{p.ns}/backup')
    if not cli.wait_for_server(timeout_sec=10.0):
        out['outcome'] = 'no /backup action server'
        return out

    goal = BackUp.Goal()
    goal.target.x = 0.3          # metres, the value in the recovery subtree
    goal.speed = 0.5
    goal.time_allowance.sec = 10

    p.arm()
    fut = cli.send_goal_async(goal)
    if not p.spin_until(lambda: fut.done(), 10.0):
        p.disarm()
        out['outcome'] = 'no response from /backup'
        return out
    handle = fut.result()
    out['accepted'] = bool(handle.accepted)
    if not handle.accepted:
        p.disarm()
        out['outcome'] = 'BackUp rejected'
        return out

    res = handle.get_result_async()
    p.spin_for(15.0, lambda: res.done())
    if res.done() and res.result() is not None:
        out['status'] = STATUS.get(res.result().status, res.result().status)
    else:
        handle.cancel_goal_async()
        p.spin_for(2.0)
        out['status'] = 'TIMED_OUT'

    out['on_diverted_topic'] = len(p.cmds)
    out['on_raw_cmd_vel'] = len(p.raw_cmds)
    out['raw_cmd_vel_nonzero'] = sum(
        1 for c in p.raw_cmds if abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6)
    if p.raw_cmds:
        out['raw_cmd_vel_vx_absmax'] = round(max(abs(c[1]) for c in p.raw_cmds), 3)
    out['verdict'] = (
        'recovery velocity ESCAPES the cmd_vel_topic diversion'
        if out['raw_cmd_vel_nonzero'] else
        'no motion on cmd_vel — recovery did not command, or was diverted')
    p.disarm()
    log.info(f'backup exercise: {json.dumps(out)}')
    return out


def _finish(report, args, node, rc):
    text = json.dumps(report, indent=2)
    print(text)
    if args.out_json:
        with open(args.out_json, 'w') as fh:
            fh.write(text + '\n')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
