# 3. Localize on a saved map

Serve a saved map and let AMCL (Nav2's particle filter) find the robot in it. You need a map from
[tutorial 2](02_mapping.md) (`~/maps/room.yaml`) and the same robot topics. Mapping and
localization both publish `map -> odom`, so run **only one of them** at a time.

## 1. Start localization

```bash
ros2 launch cpsl_nav console_slam.launch.py mode:=localization namespace:=cpsl_ugv_1 \
  platform:=create3 map_yaml:=$HOME/maps/room.yaml
```

`map_yaml` is required in this mode; the launch fails at start, naming the file, if it does not
exist. You should see the `[console_slam] mode=localization ...` line, then the lifecycle manager
bringing up `map_server` and `amcl` (`Managed nodes are active` when done). Nothing else
happens yet: AMCL publishes no `map -> odom` until it is told where the robot is.

```bash
ros2 topic list | grep -E "map|amcl|initialpose"    # /cpsl_ugv_1/map, /cpsl_ugv_1/amcl_pose, /cpsl_ugv_1/initialpose
```

The map is served on `/cpsl_ugv_1/map`. The shipped maps in `src/cpsl_nav/maps/` (for example
`cpsl.yaml`) are lab rooms and are only useful in those rooms. A map's `image:` is a path relative
to its `.yaml`, so keep the pair together.

## 2. Give the initial pose

AMCL listens on `/cpsl_ugv_1/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, frame `map`).
Either click it in RViz, or publish it.

In RViz:

```bash
ros2 launch cpsl_nav rviz_nav.launch.py namespace:=cpsl_ugv_1
```

Set **Fixed Frame** to `map`, add `/cpsl_ugv_1/map`, `/cpsl_ugv_1/scan` and
`/cpsl_ugv_1/particle_cloud` (type `PoseArray`), then click **2D Pose Estimate** in the toolbar and
drag on the map from the robot's position in the direction it faces. `rviz_nav.launch.py` remaps
RViz's `/initialpose` into the namespace, which is why the click reaches AMCL.

Or from a terminal (here the robot is at the map's origin, facing +x):

```bash
ros2 topic pub --once /cpsl_ugv_1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

## 3. Check it, then drive a little

```bash
ros2 topic echo /cpsl_ugv_1/amcl_pose --once        # the estimate, in frame map
ros2 topic echo /cpsl_ugv_1/tf --once               # now includes map -> odom
```

You should see the particle cloud collapse around the robot and the scan line up with the walls
of the map. If the scan is rotated or offset from the walls, the initial pose was wrong: send it
again. Driving the robot a few metres (teleop as in tutorial 2) lets the filter converge before
you start Nav2.

AMCL parameters (particle counts, motion noise, update thresholds) are the `amcl:` section of
`config/slam_<platform>.yaml`; see [tutorial 6](06_tuning_params.md).

Next: [navigate with Nav2](04_nav2_and_rviz.md).
