"""
Tests for the Nav2 half of cpsl_nav.params (no ROS needed).

Against the real platform files (nav2_create3.yaml, nav2_rover.yaml) and the
lab's nav2_ugv.yaml. The point is the one a launched-but-never-transforming
Nav2 would hide: after the rewrite every frame is the launch's, every scan
source reads the launch's scan topic, every static layer the namespaced map,
and both costmaps carry the keepout filter on the namespaced info topic.
"""

import copy
import glob
import os

from cpsl_nav import params as p
import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
KINDS = ('create3', 'rover')
NS = 'robot1'
SCAN = '/robot1/livox/scan'


def _ros(params, node):
    return dict(p.sections(params))[node]


def _costmaps(params):
    return [_ros(params, f'{c}/{c}') for c in ('local_costmap', 'global_costmap')]


@pytest.fixture(scope='module', params=KINDS)
def kind(request):
    return request.param


@pytest.fixture(scope='module')
def platform(kind):
    return p.load(p.platform_file(CONFIG, kind, stem='nav2'))


@pytest.fixture(scope='module')
def out(platform):
    return p.nav2_rewrite(platform, namespace=NS, scan_topic='livox/scan')


@pytest.fixture(scope='module')
def lab():
    return p.load(os.path.join(CONFIG, 'nav2_ugv.yaml'))


def test_every_slam_platform_has_a_nav2_file():
    kinds = sorted(os.path.basename(f)[len('slam_'):-len('.yaml')]
                   for f in glob.glob(os.path.join(CONFIG, 'slam_*.yaml')))
    for kind in KINDS:
        assert kind in kinds
        assert p.platform_file(CONFIG, kind, stem='nav2').endswith(f'nav2_{kind}.yaml')
    with pytest.raises(p.ParamsError, match='nav2_px4.yaml'):
        p.platform_file(CONFIG, 'px4', stem='nav2')


def test_platform_file_nodes(platform):
    names = {n for n, _ in p.sections(platform)}
    assert {'bt_navigator', 'controller_server', 'planner_server', 'behavior_server',
            'velocity_smoother', 'collision_monitor', 'smoother_server', 'waypoint_follower',
            'docking_server', 'local_costmap/local_costmap',
            'global_costmap/global_costmap'} <= names


def test_platform_files_already_name_unprefixed_frames(platform):
    assert p.leftovers(platform) == []
    assert 'cpsl_ugv_1' not in yaml.safe_dump(platform)


def test_frames(out):
    bt = _ros(out, 'bt_navigator')
    assert (bt['global_frame'], bt['robot_base_frame']) == ('map', 'base_link')
    lc, gc = _costmaps(out)
    assert (lc['global_frame'], lc['robot_base_frame']) == ('odom', 'base_link')
    assert (gc['global_frame'], gc['robot_base_frame']) == ('map', 'base_link')
    bs = _ros(out, 'behavior_server')
    assert (bs['global_frame'], bs['local_frame'], bs['robot_base_frame']) == \
        ('map', 'odom', 'base_link')
    cm = _ros(out, 'collision_monitor')
    assert (cm['base_frame_id'], cm['odom_frame_id']) == ('base_link', 'odom')
    dk = _ros(out, 'docking_server')
    assert (dk['base_frame'], dk['fixed_frame']) == ('base_link', 'odom')


def test_scan_map_and_keepout_topics_are_absolute(out):
    for c in _costmaps(out):
        ol = c['obstacle_layer']
        assert ol['observation_sources'] == 'scan'
        assert ol['scan']['topic'] == SCAN and ol['scan']['data_type'] == 'LaserScan'
        assert ol['scan']['min_obstacle_height'] < 0.0      # base_link scan points sit at z = 0
        assert ol['scan']['raytrace_min_range'] == 0.0
        assert c['map_topic'] == '/robot1/map' and c['static_layer']['map_topic'] == '/robot1/map'
        assert c['filters'] == ['keepout_filter']
        kf = c['keepout_filter']
        assert kf['plugin'] == p.KEEPOUT_PLUGIN and kf['enabled'] is True
        assert kf['filter_info_topic'] == '/robot1/costmap_filter_info'
    cm = _ros(out, 'collision_monitor')
    assert cm['observation_sources'] == ['scan'] and cm['scan']['topic'] == SCAN


def test_twist_and_relative_command_output(out):
    for name, ros in p.sections(out):
        assert ros['use_sim_time'] is False, name
        if 'enable_stamped_cmd_vel' in ros:
            assert ros['enable_stamped_cmd_vel'] is False, name
    assert _ros(out, 'collision_monitor')['cmd_vel_out_topic'] == 'cmd_vel'   # /<ns>/cmd_vel


def test_platform_specifics(kind, platform):
    lc, gc = _costmaps(platform)
    fp = _ros(platform, 'controller_server')['FollowPath']
    vs = _ros(platform, 'velocity_smoother')
    assert fp['vx_max'] == vs['max_velocity'][0]
    if kind == 'create3':
        assert lc['robot_radius'] == gc['robot_radius'] == 0.175
        assert vs['odom_topic'] == 'odom'
    else:
        assert 'robot_radius' not in lc and len(yaml.safe_load(lc['footprint'])) == 4
        assert vs['odom_topic'] == '/odometry/wheels'


def test_root_keys_stay_short(out, platform):
    # bringup_launch.py nests the whole file under the namespace itself.
    assert set(out) == set(platform)


def test_lab_file_rewrite(lab):
    assert p.leftovers(lab), 'the lab yaml no longer names cpsl_ugv_1 frames'
    out = p.nav2_rewrite(lab, namespace=NS, scan_topic='scan', odom_topic='odom')
    assert p.leftovers(out) == []
    dumped = yaml.safe_dump(out)
    assert 'cpsl_ugv_1' not in dumped and 'odom_repub' not in dumped
    assert _ros(out, 'bt_navigator')['odom_topic'] == 'odom'
    # lab users keep their prefixed tree by passing it
    out = p.nav2_rewrite(lab, base_frame='cpsl_ugv_1/base_footprint',
                         odom_frame='cpsl_ugv_1/odom')
    assert _ros(out, 'bt_navigator')['robot_base_frame'] == 'cpsl_ugv_1/base_footprint'
    assert _ros(out, 'bt_navigator')['odom_topic'] == 'odom_repub'    # empty = keep the file's


def test_tuning_is_kept(lab, kind, platform):
    out = p.nav2_rewrite(platform)
    for node in ('controller_server', 'planner_server', 'behavior_server'):
        a, b = _ros(platform, node), _ros(out, node)
        for k, v in a.items():
            if k not in p.FRAME_KEYS and k != 'use_sim_time':
                assert b[k] == v, (node, k)
    if kind == 'create3':      # same MPPI critics as the lab's
        assert _ros(platform, 'controller_server')['FollowPath']['critics'] == \
            _ros(lab, 'controller_server')['FollowPath']['critics']


def test_unknown_frame_fails_loudly(platform):
    bad = copy.deepcopy(platform)
    _ros(bad, 'bt_navigator')['robot_base_frame'] = 'cpsl_ugv_1/chassis'
    with pytest.raises(p.ParamsError, match='chassis'):
        p.nav2_rewrite(bad)


def test_odom_and_cmd_vel_args(platform):
    out = p.nav2_rewrite(platform, odom_topic='/r/odom', cmd_vel_topic='/r/cmd_vel')
    assert _ros(out, 'bt_navigator')['odom_topic'] == '/r/odom'
    assert _ros(out, 'velocity_smoother')['odom_topic'] == '/r/odom'
    assert _ros(out, 'collision_monitor')['cmd_vel_out_topic'] == '/r/cmd_vel'


def test_overrides_and_knob_defaults(tmp_path, kind):
    path = p.platform_file(CONFIG, kind, stem='nav2')
    knobs = p.values(path, 'controller_server',
                     ['FollowPath.vx_max', 'general_goal_checker.xy_goal_tolerance'])
    infl = p.values(path, 'local_costmap/local_costmap', ['inflation_layer.inflation_radius'])
    assert knobs['FollowPath.vx_max'] > 0 and infl['inflation_layer.inflation_radius'] > 0
    over = tmp_path / 'nav2.yaml'
    over.write_text(yaml.safe_dump({
        'controller_server': {p.PARAMS: {'FollowPath': {'vx_max': 0.1}}},
        'local_costmap': {'local_costmap': {p.PARAMS: {'inflation_layer': {
            'inflation_radius': 0.2}}}}}))
    out = p.nav2_effective(path, str(over), namespace=NS)
    assert _ros(out, 'controller_server')['FollowPath']['vx_max'] == 0.1
    assert _costmaps(out)[0]['inflation_layer']['inflation_radius'] == 0.2
    assert _ros(out, 'controller_server')['FollowPath']['vy_max'] == \
        _ros(p.load(path), 'controller_server')['FollowPath']['vy_max']
    over.write_text(yaml.safe_dump({
        'controller_server': {p.PARAMS: {'FollowPath': {'vx_maks': 0.1}}}}))
    with pytest.raises(p.ParamsError, match='vx_maks'):
        p.nav2_effective(path, str(over))
    with pytest.raises(p.ParamsError, match='nope'):
        p.values(path, 'controller_server', ['FollowPath.nope'])


def test_written_file_round_trips(tmp_path, out):
    path = p.write(out, directory=str(tmp_path))
    with open(path) as f:
        assert yaml.safe_load(f) == out


def test_keepout_file_and_sets(tmp_path):
    kf = p.load(os.path.join(CONFIG, 'keepout_filter.yaml'))
    mask = p.write_empty_mask(str(tmp_path))
    out = p.rewrite(kf, NS, p.keepout_sets(NS, mask, 'map'))
    info = out['/robot1/costmap_filter_info_server'][p.PARAMS]
    srv = out['/robot1/filter_mask_server'][p.PARAMS]
    assert info['type'] == 0
    assert info['filter_info_topic'] == '/robot1/costmap_filter_info'
    assert info['mask_topic'] == srv['topic_name'] == '/robot1/keepout_filter_mask'
    assert srv['yaml_filename'] == mask and srv['frame_id'] == 'map'
    with pytest.raises(p.ParamsError, match='not found'):
        p.keepout_sets(NS, str(tmp_path / 'missing.yaml'))


def test_empty_mask_is_one_free_cell(tmp_path):
    path = p.write_empty_mask(str(tmp_path))
    meta = yaml.safe_load(open(path))
    assert meta['mode'] == 'trinary'
    with open(os.path.join(str(tmp_path), meta['image']), 'rb') as f:
        data = f.read()
    assert data.startswith(b'P5\n1 1\n255\n') and data[-1] == 254    # white = free


def test_cli_nav2(capsys):
    path = p.platform_file(CONFIG, 'create3', stem='nav2')
    assert p.main([path, '--mode', 'nav2', '--namespace', NS]) == 0
    out = yaml.safe_load(capsys.readouterr().out)
    assert _costmaps(out)[0]['map_topic'] == '/robot1/map'
