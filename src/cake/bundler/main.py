import argparse
from os.path import join

from cake.utils.load_props import load_props
from .get_dockerfile import get_dockerfile


def main():
    args = parse_args()
    project_dir = args.project_dir
    props = load_props(project_dir)
    dockerfile_content = get_dockerfile(props)
    dockerfile_target_path = join(project_dir, 'Dockerfile')
    with open(dockerfile_target_path, 'w') as f:
        f.write(dockerfile_content)
    print('Dockerfile generated.\n')


def parse_args():
    parser = argparse.ArgumentParser(
        "cake-bundler",
    )
    parser.add_argument('project_dir', metavar='PROJECT_PATH')
    return parser.parse_args()
