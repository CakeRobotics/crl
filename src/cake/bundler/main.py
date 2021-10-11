import argparse
from os.path import dirname, join, realpath
import shutil

from cake.utils.load_props_from_file import load_props_from_file
from .get_dockerfile import get_dockerfile


def main():
    args = parse_args()
    project_dir = args.project_dir
    local_crl = args.local_crl

    props = load_props_from_file(project_dir)
    create_dockerfile(project_dir, props, local_crl)
    print('Dockerfile generated.')
    if local_crl:
        copy_crl_to_project_dir(project_dir)
        print('Copied local CRL to build context.')


def parse_args():
    parser = argparse.ArgumentParser(
        "cake-bundler",
    )
    parser.add_argument('project_dir', metavar='PROJECT_PATH')
    parser.add_argument('--local-crl', action='store_true')
    return parser.parse_args()


def create_dockerfile(project_dir, props, local_crl):
    dockerfile_content = get_dockerfile(props, local_crl)
    dockerfile_target_path = join(project_dir, 'Dockerfile')
    with open(dockerfile_target_path, 'w') as f:
        f.write(dockerfile_content)


def copy_crl_to_project_dir(project_dir):
    local_crl_dir = get_local_crl_dir()
    ignore_patterns = shutil.ignore_patterns(
        '__pycache__',
        '.git',
        'docs',
        'examples',
        'venv',
    )
    shutil.copytree(
        local_crl_dir,
        join(project_dir, 'crl'),
        ignore=ignore_patterns,
        dirs_exist_ok=True
    )


def get_local_crl_dir():
    this_file_dir = realpath(__file__)
    cursor = this_file_dir
    for _ in range(4):
        cursor = dirname(cursor)
    return cursor
