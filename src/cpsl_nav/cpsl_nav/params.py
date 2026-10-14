r"""
Effective parameter files for console_slam.launch.py (and nav2, later).

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

CLI, to see what a launch would run::

    python3 -m cpsl_nav.params config/slam_create3.yaml \\
        --overrides robot.yaml --namespace robot1 --mode mapping
"""

import argparse
import copy
import os
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
        if not isinstance(section, dict) or not isinstance(section.get(PARAMS), dict):
            raise ParamsError(f'{path}: section {node!r} has no {PARAMS} mapping')
    return data


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


def write(params, directory=None, prefix='console_params_'):
    """Write `params` to a new YAML file and return its path."""
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.yaml', dir=directory)
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
    return path


def values(params_file, node, keys):
    """{key: value} for `keys` of `node` in a platform file (e.g. GUI defaults)."""
    section = load(params_file).get(node)
    if section is None:
        raise ParamsError(f'{params_file}: no {node!r} section')
    p = section[PARAMS]
    missing = [k for k in keys if k not in p]
    if missing:
        raise ParamsError(f'{params_file}: {node} has no {", ".join(missing)}')
    return {k: p[k] for k in keys}


def main(argv=None):
    """Print the effective parameters; exit status 1 on a ParamsError."""
    ap = argparse.ArgumentParser(description='Print the effective parameters a launch would use.')
    ap.add_argument('params_file')
    ap.add_argument('--overrides', default='')
    ap.add_argument('--namespace', default='')
    ap.add_argument('--mode', default='mapping', choices=('mapping', 'localization'))
    ap.add_argument('--scan-topic', default='scan')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--map-yaml', default='')
    a = ap.parse_args(argv)
    try:
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
