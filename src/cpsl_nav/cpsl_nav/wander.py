"""
Wander: random reachable goals in known free space, one after another.

With no goal active, every tick (``tick_period``, 1 s):

1. draw up to ``candidates`` random free cells at least ``min_distance``
   (1 m) and at most ``max_distance`` (0 = no limit) from the robot, with
   ``clearance`` to obstacles and unknown space, none in a keepout zone (the
   keepout filter's mask plus ``keepout_margin``) (cpsl_nav/grid.py);
2. ask Nav2's planner for a path to each in turn (``compute_path_to_pose``;
   the global costmap carries the keepout filter, so a path never crosses a
   zone), and send ``NavigateToPose`` to the first with a non-empty path;
3. if none has a path, try again next tick (detail ``no reachable goal``).

``seed`` (-1 = from the clock) seeds the draws. Status on ``~/status`` (see
goal_runner.py) adds ``rejected`` (candidates with no path).
"""

import math
import time

from cpsl_nav import grid as g
from cpsl_nav.goal_runner import GoalRunner, pose, run
from nav2_msgs.action import ComputePathToPose
import numpy as np
from rclpy.action import ActionClient

PLAN_TIMEOUT = 10.0    # s for one candidate's path


class Wander(GoalRunner):
    """The wander node."""

    mode = 'wander'

    def __init__(self):
        super().__init__('wander')
        for name, default in (('min_distance', 1.0), ('max_distance', 0.0),
                              ('clearance', 0.3), ('candidates', 8), ('seed', -1)):
            self.declare_parameter(name, default)
        self.cfg = {name: self.get_parameter(name).value for name in (
            'min_distance', 'max_distance', 'clearance', 'candidates')}
        seed = int(self.get_parameter('seed').value)
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.rejected = 0
        self._queue = []            # candidates waiting for a path check
        self._checking = False
        self._check_started = 0.0
        self._robot = None

    def step(self, robot):
        if self._checking and time.monotonic() - self._check_started > PLAN_TIMEOUT:
            self.get_logger().warning('planner did not answer; drawing new candidates')
            self._checking, self._queue = False, []
        if self.busy or self._checking:
            return
        if not self.planner.server_is_ready():
            self.state, self.detail = 'waiting', 'waiting for nav2 (compute_path_to_pose)'
            return
        c = self.cfg
        max_d = float(c['max_distance']) or math.inf
        self._queue = g.sample_goals(self.grid, robot, self.rng, int(c['candidates']),
                                     float(c['min_distance']), max_d, float(c['clearance']),
                                     self.blocked)
        if not self._queue:
            self.state, self.detail = 'searching', 'no free goal far enough away'
            return
        self._robot = robot
        self.state, self.detail = 'searching', 'checking candidate goals'
        self._check_next()

    def _check_next(self):
        if not self._queue:
            self._checking = False
            self.detail = 'no reachable goal'
            return
        self._checking = True
        self._check_started = time.monotonic()
        x, y = self._queue[0]
        goal = ComputePathToPose.Goal(goal=pose(self.map_frame, x, y, 0.0), use_start=False)
        self.planner.send_goal_async(goal).add_done_callback(self._on_plan_accepted)

    def _on_plan_accepted(self, fut):
        handle = fut.result()
        if handle is None or not handle.accepted:
            self._on_plan(None)
            return
        handle.get_result_async().add_done_callback(lambda f: self._on_plan(f.result()))

    def _on_plan(self, res):
        if not self._checking or not self._queue:     # given up on (PLAN_TIMEOUT)
            return
        xy = self._queue.pop(0)
        if self._stopped:
            return
        if res is not None and res.status == 4 and len(res.result.path.poses) > 1:
            self._checking = False
            self._queue = []
            self.send(xy, self._robot)
        else:
            self.rejected += 1
            self._check_next()

    def status(self):
        s = super().status()
        s['rejected'] = self.rejected
        return s


def main(args=None):
    return run(Wander, args)


if __name__ == '__main__':
    main()
