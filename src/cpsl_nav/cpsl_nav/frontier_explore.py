"""
Frontier exploration: drive Nav2 to the boundary between known free space and unknown space.

Every tick (``tick_period``, 1 s) on the latest map:

1. frontier cells = free cells with an unknown 4-neighbour, clustered
   8-connected; clusters smaller than ``min_frontier_size`` cells are dropped
   (cpsl_nav/grid.py);
2. with no goal active, the cluster with the lowest
   ``distance - size_weight * length`` is chosen, skipping goals within
   ``blacklist_radius`` of a failed goal and goals in a keepout zone (the
   keepout filter's mask, plus ``keepout_margin``), and sent to Nav2;
3. with a goal active, if no frontier cell is left within
   ``explored_radius`` of it, the goal has done its job: it is cancelled and
   the next frontier chosen;
4. when no frontier is left (``done_after`` ticks in a row), the state is
   ``done`` with detail ``no frontiers`` (or ``no reachable frontiers`` when
   frontiers remain but every one was blacklisted or is in a keepout zone).
   The node keeps watching: a frontier that reappears is explored.

Status on ``~/status`` (see goal_runner.py) adds ``frontiers`` (clusters on
the last map) and ``blacklisted`` (failed goals).
"""

from cpsl_nav import grid as g
from cpsl_nav.goal_runner import GoalRunner, run


class FrontierExplore(GoalRunner):
    """The frontier explorer node."""

    mode = 'frontier'

    def __init__(self):
        super().__init__('frontier_explore')
        for name, default in (('min_frontier_size', 8), ('size_weight', 0.5),
                              ('goal_clearance', 0.25), ('blacklist_radius', 0.5),
                              ('min_goal_distance', 0.3), ('explored_radius', 0.4),
                              ('done_after', 3)):
            self.declare_parameter(name, default)
        self.cfg = {name: self.get_parameter(name).value for name in (
            'min_frontier_size', 'size_weight', 'goal_clearance', 'blacklist_radius',
            'min_goal_distance', 'explored_radius', 'done_after')}
        self.blacklist = []
        self.n_frontiers = 0
        self._empty_ticks = 0

    def step(self, robot):
        c = self.cfg
        found = g.frontiers(self.grid, int(c['min_frontier_size']), float(c['goal_clearance']))
        self.n_frontiers = len(found)
        if self.busy:
            if (self.goal is not None and not self._cancelling
                    and not g.frontier_near(self.grid, self.goal, float(c['explored_radius']))):
                self.get_logger().info(f'frontier at {self.goal} explored; next')
                self.cancel()
            return
        target = g.choose(found, robot, self.grid.resolution, self.blacklist,
                          float(c['blacklist_radius']), float(c['min_goal_distance']),
                          self.blocked, float(c['size_weight']))
        if target is None:
            self._empty_ticks += 1
            if self._empty_ticks >= int(c['done_after']):
                self.state = 'done'
                self.detail = 'no reachable frontiers' if found else 'no frontiers'
            else:
                self.state, self.detail = 'searching', 'looking for frontiers'
            return
        self._empty_ticks = 0
        self.send(target.goal, robot)

    def on_goal_done(self, ok, goal, cancelled):
        if not ok and not cancelled and goal is not None:
            self.blacklist.append(goal)
            self.get_logger().info(f'goal {goal} failed; blacklisted')

    def status(self):
        s = super().status()
        s.update(frontiers=self.n_frontiers, blacklisted=len(self.blacklist))
        return s


def main(args=None):
    return run(FrontierExplore, args)


if __name__ == '__main__':
    main()
