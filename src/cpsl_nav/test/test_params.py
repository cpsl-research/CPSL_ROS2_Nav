"""Tests for cpsl_nav.params (no ROS needed)."""

import glob
import os

from cpsl_nav import params as p
import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
KNOBS = ('max_laser_range', 'correlation_search_space_dimension', 'minimum_travel_distance')


def _write(tmp_path, name, data):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data))
    return str(path)


@pytest.fixture
def base(tmp_path):
    return _write(tmp_path, 'slam_test.yaml', {
        'slam_toolbox': {p.PARAMS: {
            'max_laser_range': 12.0, 'base_frame': 'base_link', 'scan_topic': '/scan',
            'ceres_solver_options': {'num_threads': 2}}},
        'amcl': {p.PARAMS: {'max_particles': 2000, 'base_frame_id': 'base_link'}},
        'map_server': {p.PARAMS: {'yaml_filename': ''}},
    })


def test_platform_files_exist_and_load():
    kinds = sorted(os.path.basename(f)[len('slam_'):-len('.yaml')]
                   for f in glob.glob(os.path.join(CONFIG, 'slam_*.yaml')))
    for kind in ('create3', 'rover'):
        assert kind in kinds
        path = p.platform_file(CONFIG, kind)
        data = p.load(path)
        assert {'slam_toolbox', 'amcl', 'map_server'} <= set(data)
        assert set(p.values(path, 'slam_toolbox', KNOBS)) == set(KNOBS)


def test_platform_file_missing_and_bad_kind(tmp_path):
    with pytest.raises(p.ParamsError, match='no parameter file'):
        p.platform_file(str(tmp_path), 'px4')
    with pytest.raises(p.ParamsError, match='bad platform kind'):
        p.platform_file(CONFIG, '../config/slam_create3')


def test_load_rejects_section_without_ros_parameters(tmp_path):
    path = _write(tmp_path, 'bad.yaml', {'slam_toolbox': {'max_laser_range': 1.0}})
    with pytest.raises(p.ParamsError, match='ros__parameters'):
        p.load(path)


def test_merge_overrides_existing_keys_only(base):
    merged = p.merge(p.load(base), {'slam_toolbox': {p.PARAMS: {
        'max_laser_range': 4.0, 'ceres_solver_options': {'num_threads': 4}}}})
    sp = merged['slam_toolbox'][p.PARAMS]
    assert sp['max_laser_range'] == 4.0
    assert sp['ceres_solver_options'] == {'num_threads': 4}
    assert sp['base_frame'] == 'base_link'
    assert merged['amcl'][p.PARAMS]['max_particles'] == 2000
    # base unchanged
    assert p.load(base)['slam_toolbox'][p.PARAMS]['max_laser_range'] == 12.0


@pytest.mark.parametrize('override, match', [
    ({'slam_toolbox': {p.PARAMS: {'max_laser_rnage': 4.0}}}, 'max_laser_rnage'),
    ({'slam_tolbox': {p.PARAMS: {'max_laser_range': 4.0}}}, 'slam_tolbox'),
    ({'slam_toolbox': {p.PARAMS: {'ceres_solver_options': 3}}}, 'mapping'),
    ({'slam_toolbox': {p.PARAMS: {'max_laser_range': {'x': 1}}}}, 'mapping'),
])
def test_merge_rejects_unknown_or_reshaped_keys(base, override, match):
    with pytest.raises(p.ParamsError, match=match):
        p.merge(p.load(base), override)


def test_overrides_missing_or_empty_is_no_overrides(base, tmp_path):
    assert p.load_overrides('') == {}
    assert p.load_overrides(str(tmp_path / 'nope.yaml')) == {}
    assert p.effective(base, str(tmp_path / 'nope.yaml')) == p.rewrite(p.load(base))


def test_rewrite_root_keys_and_sets(base):
    out = p.rewrite(p.load(base), '/robot1/', {'slam_toolbox': {'base_frame': 'base_footprint'},
                                               'nav2_ghost': {'x': 1}})
    assert set(out) == {'/robot1/slam_toolbox', '/robot1/amcl', '/robot1/map_server'}
    assert out['/robot1/slam_toolbox'][p.PARAMS]['base_frame'] == 'base_footprint'
    assert set(p.rewrite(p.load(base), '')) == {'/slam_toolbox', '/amcl', '/map_server'}


def test_topic_resolution():
    assert p.topic('robot1', 'map') == '/robot1/map'
    assert p.topic('', 'map') == '/map'
    assert p.topic('robot1', '/livox/scan') == '/livox/scan'


def test_slam_sets_mapping_namespaces_map_topic():
    sets = p.slam_sets('mapping', 'robot1', '/livox/scan', 'base_link', 'odom')
    s = sets['slam_toolbox']
    assert s['map_name'] == '/robot1/map'
    assert (s['scan_topic'], s['base_frame'], s['odom_frame'], s['map_frame']) == \
        ('/livox/scan', 'base_link', 'odom', 'map')
    assert s['mode'] == 'mapping' and s['use_sim_time'] is False
    assert set(sets) == {'slam_toolbox'}


def test_slam_sets_localization(tmp_path):
    map_yaml = tmp_path / 'room.yaml'
    map_yaml.write_text('image: room.pgm\n')
    sets = p.slam_sets('localization', 'robot1', 'scan', 'base_link', 'odom',
                       map_yaml=str(map_yaml))
    assert set(sets) == {'map_server', 'amcl'}
    assert sets['map_server']['yaml_filename'] == str(map_yaml)
    assert sets['map_server']['frame_id'] == 'map'
    a = sets['amcl']
    assert (a['base_frame_id'], a['odom_frame_id'], a['global_frame_id'], a['scan_topic']) == \
        ('base_link', 'odom', 'map', 'scan')


def test_slam_sets_localization_needs_existing_map(tmp_path):
    with pytest.raises(p.ParamsError, match='needs a map'):
        p.slam_sets('localization', map_yaml='')
    with pytest.raises(p.ParamsError, match='not found'):
        p.slam_sets('localization', map_yaml=str(tmp_path / 'missing.yaml'))
    with pytest.raises(p.ParamsError, match='unknown mode'):
        p.slam_sets('explore')


def test_effective_end_to_end_and_write(base, tmp_path):
    overrides = _write(tmp_path, 'robot1.yaml',
                       {'slam_toolbox': {p.PARAMS: {'max_laser_range': 4.0}}})
    sets = p.slam_sets('mapping', 'robot1', 'scan', 'base_link', 'odom')
    out = p.effective(base, overrides, 'robot1', sets)
    path = p.write(out, directory=str(tmp_path))
    back = yaml.safe_load(open(path))
    sp = back['/robot1/slam_toolbox'][p.PARAMS]
    assert sp['max_laser_range'] == 4.0
    assert sp['map_name'] == '/robot1/map'
    assert back['/robot1/amcl'][p.PARAMS]['max_particles'] == 2000


def test_override_of_real_platform_file(tmp_path):
    path = p.platform_file(CONFIG, 'create3')
    overrides = _write(tmp_path, 'o.yaml', {'slam_toolbox': {p.PARAMS: dict.fromkeys(KNOBS, 1.0)}})
    out = p.effective(path, overrides, 'create3_1', p.slam_sets('mapping', 'create3_1'))
    sp = out['/create3_1/slam_toolbox'][p.PARAMS]
    assert all(sp[k] == 1.0 for k in KNOBS)


def test_values_missing_key(base):
    with pytest.raises(p.ParamsError, match='nope'):
        p.values(base, 'slam_toolbox', ['nope'])


def test_cli(base, capsys):
    assert p.main([base, '--namespace', 'r1']) == 0
    assert '/r1/slam_toolbox' in yaml.safe_load(capsys.readouterr().out)
    assert p.main([base, '--mode', 'localization']) == 1
