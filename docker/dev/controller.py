import json
import logging
import os
import pip
import subprocess

import requests
# import websockets
import yaml


srcdir = '/app'
runner = None


def main():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    ensure_workspace_dir()
    ensure_source()
    ensure_requirements()
    # connect_to_devices_service()
    start_app()


def ensure_workspace_dir():
    if not os.path.isdir(srcdir):
        os.mkdir(srcdir)


def ensure_source():
    if os.path.isfile(f'{srcdir}/main.py'):
        logging.info('Using mounted source...')
        return
    logging.info('Fetching source...')
    fetch_source()


def fetch_source():
    download_source_file('main.py')
    try:
        download_source_file('props.json')
    except requests.exceptions.HTTPError:
        download_source_file('props.yaml')


def download_source_file(filename):
    base_url = os.environ['PROJECT_URL']
    auth_header = os.environ.get('AUTH_HEADER') or ''
    url = f'{base_url}/{filename}'
    response = requests.get(url, headers={'AUTHORIZATION': auth_header})
    response.raise_for_status()
    with open(f'{srcdir}/{filename}', 'wb') as f:
        f.write(response.content)



def ensure_requirements():
    requirements = read_requirements_from_props()
    if len(requirements) > 0:
        pip.main(['install', *requirements])


def read_requirements_from_props():
    json_file_path = f'{srcdir}/props.json'
    yaml_file_path = f'{srcdir}/props.yaml'
    if os.path.isfile(json_file_path):
        with open(json_file_path) as f:
            props = json.load(f)
    elif os.path.isfile(yaml_file_path):
        with open(yaml_file_path) as f:
            props = yaml.safe_load(f)
    else:
        props = {}
    return props.get('pip_requirements') or []


def start_app():
    subprocess.run(['python3', '-u', 'main.py'], cwd='/app')


if __name__ == "__main__":
    main()
