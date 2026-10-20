# 5. Keepout zones and exploration

Two things on top of tutorial 4: no-go zones Nav2 will not plan through, and two explorers that
send goals for you. Both are plain `ros2 launch` commands; they need Nav2 running as in
[tutorial 4](04_nav2_and_rviz.md).

## 1. Keepout zones

`console_nav.launch.py` always runs a keepout filter, and both costmaps of every platform file
carry it (`filters: ["keepout_filter"]`). With no mask it serves a one-cell free mask, which means
no zones. The mask is an ordinary map image on the same grid as your map: **black cells are
keepout, everything else is ignored.**

To make one from the map of tutorial 2:

```bash
cd ~/maps
cp room.pgm room.keepout.pgm          # same size, resolution and origin as the map
sed 's/room.pgm/room.keepout.pgm/' room.yaml > room.keepout.yaml
```

Open `room.keepout.pgm` in any image editor (GIMP, for example): fill everything white and paint
the forbidden areas black. Paint each zone **larger than the area itself, by the robot's radius
plus a margin** (the Create 3 file uses `robot_radius: 0.175`): Nav2 applies the filter after the
inflation layer, so a zone is not inflated, and a tight zone lets the planner run the robot's
centre along its edge. Then start Nav2 with it:

```bash
ros2 launch cpsl_nav console_nav.launch.py namespace:=cpsl_ugv_1 platform:=create3 \
  keepout_mask:=$HOME/maps/room.keepout.yaml
```

To change the mask without restarting Nav2:

```bash
ros2 service call /cpsl_ugv_1/filter_mask_server/load_map nav2_msgs/srv/LoadMap \
  "{map_url: $HOME/maps/room.keepout.yaml}"
```

The result code is `0` on success. Then send a goal that lies inside a zone (tutorial 4): the
planner should refuse it or route around it.

## 2. Exploration

`console_explore.launch.py` starts one node in the namespace. It needs Nav2 (above) and a map
publishing on `/cpsl_ugv_1/map` (mapping or localization, tutorials 2-3), and sends one
`NavigateToPose` goal at a time. It avoids keepout zones by reading the mask on
`/cpsl_ugv_1/keepout_filter_mask`.

```bash
ros2 launch cpsl_nav console_explore.launch.py mode:=frontier namespace:=cpsl_ugv_1
ros2 launch cpsl_nav console_explore.launch.py mode:=wander namespace:=cpsl_ugv_1
```

- **`frontier`** (node `/cpsl_ugv_1/frontier_explore`) drives to the nearest large frontier, where
  known free space meets unknown space. It cancels a goal once its frontier has been mapped,
  blacklists goals that fail, and reports `done` / `no frontiers` when nothing is left. With
  `mode:=mapping` the map grows as the robot explores.
- **`wander`** (node `/cpsl_ugv_1/wander`) drives to random free goals at least 1 m away, each
  checked for a path with Nav2's `compute_path_to_pose` before it is sent. It reports `no free goal
  far enough away` until enough of the map is known. `seed:=N` makes the sequence repeatable.

Other arguments: `goal_timeout` (default 90 s, then the goal is cancelled as failed), `map_topic`,
`keepout_topic`, `base_frame`, `map_frame`, `tf_topic` (default `/<ns>/tf`; an absolute name).

Watch what it is doing (the status is latched, so you get the last message at once):

```bash
ros2 topic echo /cpsl_ugv_1/frontier_explore/status      # or /cpsl_ugv_1/wander/status
```

It is JSON, once a second: `mode`, `state` (`waiting`, `searching`, `navigating`, `done`,
`stopped`), `detail`, `goal`, `goals_sent`, `goals_succeeded`, `goals_failed`, `known_m2`, plus
`frontiers` / `blacklisted` (frontier) or `rejected` (wander).

Stop with Ctrl-C (SIGINT): the node cancels its active goal, waits up to 1.5 s for Nav2 to confirm,
publishes `stopped` and exits 0. When exploring with `mode:=mapping`, save the map afterwards
(tutorial 2, section 3).

Next: [tuning parameters](06_tuning_params.md).
