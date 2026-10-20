# Developing CPSL_ROS2_Nav

For people changing this repo. Users want the [README](../README.md) and the
[tutorials](tutorials/01_install_and_build.md). The tf and topic protocol every launch file follows is
[TF_AND_TOPICS.md](https://github.com/cpsl-research/console/blob/main/docs/TF_AND_TOPICS.md).

## Layout

One ament_python package, `src/cpsl_nav/`; `scripts/deps/*.sh` are the apt dependencies
(`_common.sh` holds the shared helpers: root only, idempotent, system packages only).

| Path | What |
|---|---|
| `launch/console_{slam,nav,explore}.launch.py` | the maintained launch files; their header docstrings are the spec |
| `launch/{slam_sync,localization,nav2,rviz_nav}.launch.py` | older, pre-protocol; they prefix frames with the namespace. `rviz_nav` is protocol-clean |
| `cpsl_nav/params.py` | merge + rewrite of parameter files (no ROS imports); also a CLI |
| `cpsl_nav/{goal_runner,frontier_explore,wander}.py`, `grid.py` | the explorers; `grid.py` is numpy only |
| `cpsl_nav/tf_static_relay.py` | copies `/tf_static` onto `/<ns>/tf_static` (off by default) |
| `config/` | `slam_<kind>.yaml`, `nav2_<kind>.yaml`, `keepout_filter.yaml`; older and variant files below |
| `maps/`, `rviz_cfgs/` | lab maps (installed); RViz layouts from lab robots |

`setup.py` installs `launch/*`, `config/*`, `rviz_cfgs/*` and `maps/*` by glob, so a new file needs no
edit. With `colcon build --symlink-install` an edit to any of them takes effect without a rebuild.
The three executables are `frontier_explore`, `wander` and `tf_static_relay`.

## How a console launch runs

`console_slam` and `console_nav` do not pass a YAML file straight to the nodes. They call
`params.effective()` (slam) or `params.nav2_effective()` (Nav2), write the result to a temporary file,
hand that to the nodes and delete it on shutdown. Every node gets explicit `/tf` and `/tf_static`
remaps because stock launch files stay on the global topics.

## `params.py` rules

1. A platform file is **complete**; there is no base file to inherit from.
2. `merge()` raises `ParamsError` for an override key the platform file lacks, or one that turns a
   mapping into a scalar or back. Never relax this: it is what catches misspelt keys.
3. `rewrite()` turns root keys into `/<ns>/<node>` and applies launch-decided `sets`; sets for a node
   with no section are skipped (a mode only rewrites what it runs).
4. `nav2_rewrite()` keeps root keys short (stock `bringup_launch.py` nests the file in the namespace)
   and sets frames and topics by key name ([tutorial 6](tutorials/06_tuning_params.md) lists them).
   A frame it cannot classify (last segment not `map`/`odom`/`base_link`/`base_footprint`) raises.
5. Topics it writes are absolute; frames are never prefixed.

## Adding a platform kind

1. Copy the closest `config/slam_<kind>.yaml` and `config/nav2_<kind>.yaml`; keep the same pair of names.
2. Change what differs (frames as published, scan source, radius, speeds). Say in the header comment
   what is tested and what is not.
3. Keep `filters: ["keepout_filter"]` in both Nav2 costmaps and `enable_stamped_cmd_vel: false`
   unless your robot takes `TwistStamped`.
4. Add the kind to `KINDS` in `test/test_nav2_params.py` (the tests parametrize over it), run the
   tests, then `python3 -m cpsl_nav.params ...` ([tutorial 6](tutorials/06_tuning_params.md)) to read the result.
5. A file named `slam_<x>.yaml` is a platform to `platform:=<x>`, so name variants so they do not clash.

## Tests

Pure Python, no ROS needed (PyYAML, numpy, pytest). From `src/cpsl_nav/`:

```bash
python3 -m pytest test/test_params.py test/test_nav2_params.py test/test_grid.py -q   # 62 passed
```

`colcon test --packages-select cpsl_nav` also runs the ament checks: pep257 passes, copyright is
skipped, and **flake8 fails** (96 style findings in the older launch files and `setup.py`; the
`console_*` files and `cpsl_nav/` are clean of them). Nothing tests the launch files end to end; run
`--show-args` after editing one and check a real launch on a robot or simulator.

## Map storage

A map is `<name>.yaml` + `<name>.pgm` (map_server format); a keepout mask is `<name>.keepout.yaml` +
`.keepout.pgm`. `.gitignore` excludes `*.pgm`, `*.data` and `*.posegraph`, so a map to commit needs
`git add -f`. Maps in `src/cpsl_nav/maps/` are lab rooms; whether to commit a new one is a lab
decision. A map's `image:` must be relative to its YAML.

## Known state

- `package.xml` still says `description: TODO`.
- The `rviz_cfgs/*.rviz` files name lab topics (`/cpslCreate3/...`, `/cpsl_uav_1/...`).

## Appendix: files kept but not exercised

Referenced by no launch file, test or tutorial; kept by decision, not because they are known to work
with the `console_*` launch files: `localization_radar.yaml`, `nav2_uav_radar.yaml`,
`nav2_ugv_dwb.yaml`, `nav2_ugv_voxel.yaml`, `slam_athena.yaml`, `slam_radar.yaml`,
`slam_radar_scan_matching.yaml`. Used by the older launch files only: `slam.yaml`,
`localization.yaml`, `nav2_ugv.yaml`, `nav2_uav.yaml`, `slam_from_previous.yaml`. Check a variant's
frames and topics with `python3 -m cpsl_nav.params` before relying on it.
