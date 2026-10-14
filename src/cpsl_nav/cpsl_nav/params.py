r"""
Effective parameter files for console_slam.launch.py and console_nav.launch.py.

A platform's parameters live in one full file per platform kind,
``config/<stem>_<kind>.yaml`` (``slam_create3.yaml``, ``slam_rover.yaml``).
What a launch actually hands its nodes is built from that file in three steps:

1. **load** the platform file;
2. **merge** an optional overrides file over it (same shape as the platform
   file: ``<node>: ros__parameters: {key: value}``). An override may only
   change a key the platform file already has, so a misspelt key fails here
   instead of being ignored by the node;
3. **rewrite** what the launch decides rather than the file: frames, topics,
   the map file, and the root keys, which become fully qualified node names
   (``/<namespace>/<node>``) so the sections apply inside a namespace.

The result is written to a temporary file and passed to the nodes. Plain
Python + PyYAML, no ROS imports, so it can be used (and tested) without a
sourced workspace.

Nav2 (``nav2_<kind>.yaml``) differs in step 3 only: its root keys stay short
node names, because stock ``bringup_launch.py`` nests the whole file under the
namespace itself, and :func:`nav2_rewrite` sets frames and topics wherever
they occur (a costmap's section is one level deeper:
``local_costmap: local_costmap: ros__parameters``).

CLI, to see what a launch would run::

    python3 -m cpsl_nav.params config/slam_create3.yaml \\
        --overrides robot.yaml --namespace robot1 --mode mapping
    python3 -m cpsl_nav.params config/nav2_create3.yaml --namespace robot1 --mode nav2
"""

import argparse
import copy
import os
import re
import sys
import tempfile

import yaml

PARAMS = 'ros__parameters'


class ParamsError(ValueError):
    """A parameter file that cannot be used as asked."""


def platform_file(config_dir, kind, stem='slam'):
    """Path of ``<stem>_<kind>.yaml`` in `config_dir`; ParamsError if absent."""
    if not kind or os.sep in kind or kind.startswith('.'):
        raise ParamsError(f'bad platform kind: {kind!r}')
    path = os.path.join(config_dir, f'{stem}_{kind}.yaml')
    if not os.path.isfile(path):
        raise ParamsError(f'no parameter file for platform {kind!r}: {path}')
    return path


def load(path):
    """Load a parameter file: a mapping of node name -> {ros__parameters: {...}}."""
    with open(path) as f:
        data = yaml.safe_load(f)
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ParamsError(f'{path}: top level is not a mapping')
    for node, section in data.items():
        if not _is_section(section):
            raise ParamsError(f'{path}: section {node!r} has no {PARAMS} mapping')
    return data


def _is_section(section):
    """Tell whether `section` is a node's section or a namespace of them (costmaps nest)."""
    if not isinstance(section, dict):
        return False
    if isinstance(section.get(PARAMS), dict):
        return True
    return bool(section) and all(isinstance(v, dict) and isinstance(v.get(PARAMS), dict)
                                 for v in section.values())


def sections(params, _path=''):
    """Yield (``node`` or ``ns/node``, its ros__parameters dict) for every node."""
    for name, section in params.items():
        path = f'{_path}/{name}' if _path else str(name)
        if isinstance(section.get(PARAMS), dict):
            yield path, section[PARAMS]
        else:
            yield from sections(section, path)


def load_overrides(path):
    """
    Load the overrides file at `path`; {} if `path` is empty or does not exist.

    A missing file is the normal state (no per-robot tuning yet, or reset to
    defaults), so it is not an error.
    """
    if not path or not os.path.isfile(path):
        return {}
    return load(path)


def merge(base, overrides, _where=''):
    """
    Return `base` with `overrides` merged in, recursively; neither is modified.

    Mappings merge key by key; any other value replaces. A key in `overrides`
    that `base` lacks raises ParamsError, as does replacing a mapping with a
    scalar or the reverse.
    """
    out = copy.deepcopy(base)
    for key, value in (overrides or {}).items():
        where = f'{_where}.{key}' if _where else str(key)
        if key not in out:
            raise ParamsError(f'override {where!r} is not a parameter of the platform file')
        if isinstance(out[key], dict) or isinstance(value, dict):
            if not (isinstance(out[key], dict) and isinstance(value, dict)):
                raise ParamsError(f'override {where!r} changes a mapping into a value or back')
            out[key] = merge(out[key], value, where)
        else:
            out[key] = copy.deepcopy(value)
    return out


def node_key(namespace, node):
    """Fully qualified node name used as a root key: ``/ns/node`` or ``/node``."""
    ns = (namespace or '').strip('/')
    node = node.strip('/').split('/')[-1]
    return f'/{ns}/{node}' if ns else f'/{node}'


def rewrite(params, namespace='', sets=None):
    """
    Rewrite root keys to fully qualified names under `namespace`; apply `sets`.

    `sets` maps a node's short name to {key: value} written into its
    ros__parameters, whether or not the file has the key. Sets for a node the
    file has no section for are skipped: a mode only rewrites what it runs.
    """
    out = {}
    for node, section in params.items():
        out[node_key(namespace, node)] = copy.deepcopy(section)
    for node, values in (sets or {}).items():
        key = node_key(namespace, node)
        if key in out:
            out[key][PARAMS].update(values)
    return out


def topic(namespace, name):
    """`name` resolved the way a node in `namespace` would resolve it."""
    if name.startswith('/'):
        return name
    ns = (namespace or '').strip('/')
    return f'/{ns}/{name}' if ns else f'/{name}'


def slam_sets(mode, namespace='', scan_topic='scan', base_frame='base_link',
              odom_frame='odom', map_frame='map', map_yaml='', use_sim_time=False):
    """
    Return what console_slam.launch.py rewrites, per node, for `mode`.

    mapping: slam_toolbox. Its map topic parameter (``map_name``) defaults to
    the absolute ``/map``, so it is set to the namespaced topic explicitly.
    localization: map_server serving `map_yaml` and AMCL on it.
    """
    common = {'use_sim_time': bool(use_sim_time)}
    map_topic = topic(namespace, 'map')
    if mode == 'mapping':
        return {'slam_toolbox': dict(common, mode='mapping', scan_topic=scan_topic,
                                     base_frame=base_frame, odom_frame=odom_frame,
                                     map_frame=map_frame, map_name=map_topic)}
    if mode == 'localization':
        if not map_yaml:
            raise ParamsError('localization needs a map (map_yaml is empty)')
        if not os.path.isfile(map_yaml):
            raise ParamsError(f'map file not found: {map_yaml}')
        return {
            'map_server': dict(common, yaml_filename=map_yaml, frame_id=map_frame,
                               topic_name='map'),
            'amcl': dict(common, scan_topic=scan_topic, map_topic='map',
                         base_frame_id=base_frame, odom_frame_id=odom_frame,
                         global_frame_id=map_frame),
        }
    raise ParamsError(f'unknown mode {mode!r} (mapping | localization)')


def effective(params_file, overrides_file='', namespace='', sets=None):
    """Load, merge and rewrite; returns the parameter dict a launch passes on."""
    return rewrite(merge(load(params_file), load_overrides(overrides_file)),
                   namespace, sets)


# --------------------------------------------------------------------------
# Nav2
# --------------------------------------------------------------------------

# Every key, in a Nav2 file, that names a tf frame.
FRAME_KEYS = frozenset({'global_frame', 'robot_base_frame', 'local_frame', 'base_frame',
                        'base_frame_id', 'odom_frame_id', 'fixed_frame'})
# A frame's last path segment -> which launch frame replaces it. The lab's
# robots carry a base_footprint and tf-prefixed names (``cpsl_ugv_1/odom``).
_FRAME_ROLE = {'map': 'map', 'odom': 'odom', 'base_link': 'base', 'base_footprint': 'base'}
# A tf frame never starts with '/' (an absolute topic such as /robot1/map does).
_PREFIXED = re.compile(r'^[^/]+/(odom|base_link|base_footprint|map)$')
KEEPOUT_PLUGIN = 'nav2_costmap_2d::KeepoutFilter'
STATIC_PLUGIN = 'nav2_costmap_2d::StaticLayer'
FILTER_INFO_TOPIC = 'costmap_filter_info'
MASK_TOPIC = 'keepout_filter_mask'


def _observation_sources(d):
    names = d.get('observation_sources')
    if names is None:
        return []
    return names.split() if isinstance(names, str) else [str(n) for n in names]


def _nav2_walk(d, frames, topics):
    for key, value in list(d.items()):
        if key in FRAME_KEYS and isinstance(value, str):
            role = _FRAME_ROLE.get(value.strip('/').split('/')[-1])
            if role is None:
                raise ParamsError(f'nav2 params: frame {value!r} ({key}) is not a map, '
                                  'odom or base frame')
            d[key] = frames[role]
        elif isinstance(value, dict):
            _nav2_walk(value, frames, topics)
    plugin = str(d.get('plugin', ''))
    if plugin == STATIC_PLUGIN:
        d['map_topic'] = topics['map']
    elif plugin == KEEPOUT_PLUGIN:
        d['filter_info_topic'] = topics['filter_info']
    for name in _observation_sources(d):
        src = d.get(name)
        if not isinstance(src, dict):
            raise ParamsError(f'nav2 params: observation source {name!r} has no settings')
        src['topic'] = topics['scan']


def nav2_rewrite(params, namespace='', scan_topic='scan', map_topic='map', odom_topic='',
                 cmd_vel_topic='', base_frame='base_link', odom_frame='odom',
                 map_frame='map', use_sim_time=False):
    """
    Return a Nav2 parameter dict with what the launch decides set everywhere.

    * every frame key: by its last path segment, ``map`` -> `map_frame`,
      ``odom`` -> `odom_frame`, ``base_link``/``base_footprint`` -> `base_frame`
      (so the lab's ``cpsl_ugv_1/base_footprint`` becomes the launch's base
      frame); any other frame raises ParamsError;
    * every costmap observation source (and the collision monitor's) reads
      `scan_topic`; every static layer and costmap reads `map_topic`; every
      keepout filter reads ``<namespace>/costmap_filter_info`` -- all resolved
      to absolute names, so no layer depends on how Nav2 resolves a relative
      topic inside a costmap's own namespace;
    * `odom_topic` / `cmd_vel_topic`, if given, replace every ``odom_topic`` /
      the collision monitor's ``cmd_vel_out_topic``; empty keeps the file's
      (the platform file knows the robot's odometry and command topics);
    * ``use_sim_time`` everywhere.

    Root keys are left as short node names. Raises ParamsError if a
    tf-prefixed frame survives anywhere.
    """
    frames = {'map': map_frame, 'odom': odom_frame, 'base': base_frame}
    topics = {'scan': topic(namespace, scan_topic or 'scan'),
              'map': topic(namespace, map_topic or 'map'),
              'filter_info': topic(namespace, FILTER_INFO_TOPIC)}
    out = copy.deepcopy(params)
    for path, ros in sections(out):
        ros['use_sim_time'] = bool(use_sim_time)
        if odom_topic and 'odom_topic' in ros:
            ros['odom_topic'] = odom_topic
        if cmd_vel_topic and 'cmd_vel_out_topic' in ros:
            ros['cmd_vel_out_topic'] = cmd_vel_topic
        _nav2_walk(ros, frames, topics)
        if path.endswith('_costmap') and 'plugins' in ros:
            ros['map_topic'] = topics['map']
    bad = leftovers(out, allowed=set(frames.values()))
    if bad:
        raise ParamsError('nav2 params still name a tf-prefixed frame: ' + ', '.join(bad))
    return out


def leftovers(params, allowed=(), _path=''):
    """``path=value`` for every string that looks like a tf-prefixed frame not in `allowed`."""
    found = []
    if isinstance(params, dict):
        for k, v in params.items():
            found += leftovers(v, allowed, f'{_path}.{k}' if _path else str(k))
    elif isinstance(params, list):
        for i, v in enumerate(params):
            found += leftovers(v, allowed, f'{_path}[{i}]')
    elif isinstance(params, str) and _PREFIXED.search(params) and params not in allowed:
        found.append(f'{_path}={params}')
    return found


def nav2_effective(params_file, overrides_file='', **rewrite_args):
    """Load, merge and :func:`nav2_rewrite`; root keys stay short node names."""
    return nav2_rewrite(merge(load(params_file), load_overrides(overrides_file)),
                        **rewrite_args)


def keepout_sets(namespace='', mask_yaml='', map_frame='map', use_sim_time=False):
    """Return what console_nav.launch.py sets in config/keepout_filter.yaml, per node."""
    if mask_yaml and not os.path.isfile(mask_yaml):
        raise ParamsError(f'keepout mask not found: {mask_yaml}')
    mask_topic = topic(namespace, MASK_TOPIC)
    common = {'use_sim_time': bool(use_sim_time)}
    return {
        'costmap_filter_info_server': dict(
            common, filter_info_topic=topic(namespace, FILTER_INFO_TOPIC), mask_topic=mask_topic),
        'filter_mask_server': dict(common, yaml_filename=mask_yaml, frame_id=map_frame,
                                   topic_name=mask_topic),
    }


def write_empty_mask(directory, name='keepout_empty'):
    """
    Write a keepout mask with no keepout cells; return its YAML path.

    One free cell: the keepout filter leaves every cell outside a mask alone,
    so this is "no zones" on any map. A real mask (console's, rasterized on
    the map's grid) replaces it at run time through filter_mask_server's
    ``load_map`` service.
    """
    pgm = os.path.join(directory, name + '.pgm')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n1 1\n255\n\xfe')
    path = os.path.join(directory, name + '.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump({'image': name + '.pgm', 'mode': 'trinary', 'resolution': 0.05,
                        'origin': [0.0, 0.0, 0.0], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.25}, f)
    return path


def write(params, directory=None, prefix='console_params_'):
    """Write `params` to a new YAML file and return its path."""
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.yaml', dir=directory)
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
    return path


def values(params_file, node, keys):
    """
    {key: value} for `keys` of `node` in a platform file (e.g. GUI defaults).

    `node` may be nested (``local_costmap/local_costmap``); a key may be dotted
    (``FollowPath.vx_max``, ``inflation_layer.inflation_radius``).
    """
    found = dict(sections(load(params_file)))
    if node not in found:
        raise ParamsError(f'{params_file}: no {node!r} section')
    out, missing = {}, []
    for key in keys:
        d = found[node]
        for part in key.split('.'):
            if not isinstance(d, dict) or part not in d:
                missing.append(key)
                break
            d = d[part]
        else:
            out[key] = d
    if missing:
        raise ParamsError(f'{params_file}: {node} has no {", ".join(missing)}')
    return out


def main(argv=None):
    """Print the effective parameters; exit status 1 on a ParamsError."""
    ap = argparse.ArgumentParser(description='Print the effective parameters a launch would use.')
    ap.add_argument('params_file')
    ap.add_argument('--overrides', default='')
    ap.add_argument('--namespace', default='')
    ap.add_argument('--mode', default='mapping', choices=('mapping', 'localization', 'nav2'))
    ap.add_argument('--scan-topic', default='scan')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--map-yaml', default='')
    a = ap.parse_args(argv)
    try:
        if a.mode == 'nav2':
            out = nav2_effective(a.params_file, a.overrides, namespace=a.namespace,
                                 scan_topic=a.scan_topic, base_frame=a.base_frame,
                                 odom_frame=a.odom_frame)
        else:
            sets = slam_sets(a.mode, a.namespace, a.scan_topic, a.base_frame, a.odom_frame,
                             map_yaml=a.map_yaml)
            out = effective(a.params_file, a.overrides, a.namespace, sets)
    except (ParamsError, OSError, yaml.YAMLError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 1
    yaml.safe_dump(out, sys.stdout, default_flow_style=False, sort_keys=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
