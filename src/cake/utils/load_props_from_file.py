# NOTE: this file is symlinked (original file at src/cake/utils/ and relative link
# at src/cake_bundler/) Reason: We don't want to import anything from `cake` into
# `cake_bundler` inside python code. This way we can use bundler without having
# to install all crl dependencies e.g. rclpy.

import json, yaml
from os.path import exists, isdir, join


def load_props_from_file(project_dir):
    _assert_valid_project_dir(project_dir)
    yaml_path = join(project_dir, 'props.yaml')
    if exists(yaml_path):
        return _load_yaml_props(yaml_path)
    json_path = join(project_dir, 'props.json')
    if exists(json_path):
        return _load_json_props(json_path)
    raise Exception(f"Can't find `{yaml_path}` or `{json_path}`")


def _assert_valid_project_dir(project_dir):
    assert exists(project_dir), f"`{project_dir}` doesn't exist."
    assert isdir(project_dir), f"`{project_dir}` isn't a directory."


def _load_yaml_props(yaml_file_path):
    with open(yaml_file_path) as f:
        return yaml.safe_load(f)


def _load_json_props(json_file_path):
    with open(json_file_path) as f:
        return json.load(f)
