"""
What frontier_explore and wander share: a node that sends its own Nav2 goals.

* Inputs: the map (``map_topic``, default ``map`` = /<ns>/map), the keepout
  filter's mask (``keepout_topic``, default ``keepout_filter_mask``; optional),
  and the robot's pose from tf (``map_frame`` -> ``base_frame``).
* Output: ``NavigateToPose`` goals on ``navigate_to_pose`` (Nav2 in the same
  namespace), one at a time, each cancelled after ``goal_timeout`` seconds.
* Status: JSON on ``~/status`` (std_msgs/String, reliable + transient local,
  so a late subscriber gets the last one), on every tick (``tick_period``)::

    {"mode": "frontier", "state": "navigating", "detail": "...",
     "goal": [x, y] | null, "goals_sent": 3, "goals_succeeded": 2,
     "goals_failed": 0, "known_m2": 31.4, "stamp": 1791980170.0, ...}

  ``state`` is one of ``waiting`` (for Nav2, the map or tf; `detail` says
  which), ``searching``, ``navigating``, ``done`` (frontier only) or
  ``stopped`` (the last message, on shutdown).
* Stop: SIGINT or SIGTERM cancels the active goal, waits up to
  ``CANCEL_WAIT`` seconds for Nav2 to confirm, publishes ``stopped`` and exits
  0. rclpy's own signal handling is off for this: it would shut the context
  down before the cancel could be sent.
"""

import json
import math
import signal
import time

from cpsl_nav import grid as g
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros

CANCEL_WAIT = 1.5
_LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=QoSReliabilityPolicy.RELIABLE)
_SUCCEEDED = 4      # action_msgs/GoalStatus.STATUS_SUCCEEDED


def pose(frame, x, y, yaw, stamp=None):
    """Return a PoseStamped at (x, y) facing `yaw`."""
    p = PoseStamped()
    p.header.frame_id = frame
    if stamp is not None:
        p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return p


class GoalRunner(Node):
    """Base node: map, keepout, pose, one NavigateToPose goal at a time, status."""

    mode = ''

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('keepout_topic', 'keepout_filter_mask')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('tick_period', 1.0)
        self.declare_parameter('keepout_margin', 0.2)
        p = self.get_parameter
        self.map_frame = p('map_frame').value
        self.base_frame = p('base_frame').value
        self.goal_timeout = float(p('goal_timeout').value)
        self.keepout_margin = float(p('keepout_margin').value)

        self.grid = None
        self.keepout = None
        self.state, self.detail = 'waiting', 'starting'
        self.goal = None                # (x, y) of the active goal
        self.counts = {'goals_sent': 0, 'goals_succeeded': 0, 'goals_failed': 0}
        self._handle = None
        self._goal_future = None
        self._goal_started = 0.0
        self._cancelling = False
        self._timed_out = False
        self._stopped = False

        self.create_subscription(OccupancyGrid, p('map_topic').value, self._on_map, _LATCHED)
        self.create_subscription(OccupancyGrid, p('keepout_topic').value, self._on_keepout,
                                 _LATCHED)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '~/status', _LATCHED)
        self.create_timer(float(p('tick_period').value), self._tick)

    # ---- inputs --------------------------------------------------------

    def _on_map(self, msg):
        self.grid = g.Grid.from_msg(msg)

    def _on_keepout(self, msg):
        self.keepout = g.Grid.from_msg(msg)

    def robot_xy(self):
        """(x, y) of the base frame in the map frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def blocked(self, x, y):
        """Tell whether (x, y) is in (or within keepout_margin of) a keepout zone."""
        return g.blocked_by_mask(self.keepout, x, y, self.keepout_margin)

    # ---- goals ---------------------------------------------------------

    @property
    def busy(self):
        """Tell whether a goal is being sent or is active."""
        return self._goal_future is not None or self._handle is not None

    def send(self, xy, robot_xy=None):
        """Send a NavigateToPose goal to `xy`, facing away from `robot_xy`."""
        if self._stopped:
            return
        yaw = 0.0
        if robot_xy is not None:
            yaw = math.atan2(xy[1] - robot_xy[1], xy[0] - robot_xy[0])
        goal = NavigateToPose.Goal()
        goal.pose = pose(self.map_frame, xy[0], xy[1], yaw)
        self.goal = (round(float(xy[0]), 3), round(float(xy[1]), 3))
        self.counts['goals_sent'] += 1
        self._goal_started = time.monotonic()
        self._cancelling = self._timed_out = False
        self.state, self.detail = 'navigating', f'goal {self.goal}'
        self.get_logger().info(f'{self.mode}: goal {self.goal}')
        self._goal_future = self.nav.send_goal_async(goal)
        self._goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):
        self._goal_future = None
        handle = fut.result()
        if handle is None or not handle.accepted:
            self._finish(False, 'goal rejected')
            return
        self._handle = handle
        if self._cancelling:            # stop arrived while the goal was in flight
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        res = fut.result()
        ok = res is not None and res.status == _SUCCEEDED
        self._handle = None
        self._finish(ok, 'goal reached' if ok else f'goal ended with status '
                     f'{res.status if res else "?"}')

    def _finish(self, ok, detail):
        goal, cancelled = self.goal, self._cancelling
        self.goal = None
        self.counts['goals_succeeded' if ok else 'goals_failed'] += 0 if cancelled else 1
        self.detail = detail
        if self.state == 'navigating':
            self.state = 'searching'
        self.on_goal_done(ok, goal, cancelled)

    def cancel(self):
        """Cancel the active goal (its result still arrives, flagged cancelled)."""
        self._cancelling = True
        if self._handle is not None:
            return self._handle.cancel_goal_async()
        return None

    def on_goal_done(self, ok, goal, cancelled):
        """Override: a goal finished (`cancelled` if this node cancelled it)."""

    def step(self, robot_xy):
        """Override: one tick with Nav2, the map and the pose all available."""

    # ---- tick / status -------------------------------------------------

    def _tick(self):
        if self._stopped:
            return
        waiting = None
        if not self.nav.server_is_ready():
            waiting = 'nav2 (navigate_to_pose)'
        elif self.grid is None:
            waiting = 'map'
        robot = self.robot_xy() if waiting is None else None
        if waiting is None and robot is None:
            waiting = f'tf {self.map_frame} -> {self.base_frame}'
        if waiting is not None:
            if not self.busy:
                self.state, self.detail = 'waiting', f'waiting for {waiting}'
        elif (self._handle is not None and not (self._cancelling or self._timed_out)
              and time.monotonic() - self._goal_started > self.goal_timeout):
            self.get_logger().warning(f'{self.mode}: goal {self.goal} timed out; cancelling')
            self._handle.cancel_goal_async()
            self._timed_out = True      # counted as a failure: it was not reached
            self.detail = 'goal timed out'
        else:
            self.step(robot)
        self.publish_status()

    def status(self):
        """Override to add fields; the base fields are always there."""
        return dict(mode=self.mode, state=self.state, detail=self.detail,
                    goal=list(self.goal) if self.goal else None, **self.counts,
                    known_m2=round(self.grid.known_area(), 2) if self.grid else 0.0,
                    stamp=round(time.time(), 3))

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.status())))

    # ---- shutdown ------------------------------------------------------

    def stop(self, executor):
        """Cancel any goal, wait for Nav2 to confirm (<= CANCEL_WAIT s), publish stopped."""
        self._stopped = True            # no more ticks: no new goals, no status after this
        end = time.monotonic() + CANCEL_WAIT
        if self._goal_future is not None:
            self._cancelling = True
            while self._goal_future is not None and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.05)
        fut = self.cancel() if self._handle is not None else None
        while fut is not None and not fut.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        cancelled = fut is not None and fut.done()
        self.goal = None
        self.state = 'stopped'
        self.detail = 'goal cancelled' if cancelled else 'no active goal'
        self.publish_status()
        flush = time.monotonic() + 0.2
        while time.monotonic() < flush:
            executor.spin_once(timeout_sec=0.05)


def run(node_cls, args=None):
    """Spin `node_cls` until SIGINT/SIGTERM, then stop it cleanly; return 0."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.append(True))
    node = node_cls()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stopping and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
        node.stop(executor)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
