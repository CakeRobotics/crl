import json
import logging
import os
import pip
import subprocess

import requests
import socketio
import yaml


srcdir = '/app'
app_process = None
socket_client = socketio.Client()


def main():
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
    ensure_workspace_dir()
    ensure_source()
    ensure_requirements()
    connect_to_devices_service()
    start_app()
    handle_logs()
    print_exit_code()


def ensure_workspace_dir():
    if not os.path.isdir(srcdir):
        os.mkdir(srcdir)


def ensure_source():
    if os.path.isfile(f'{srcdir}/main.py'):
        logging.info('Using mounted source...')
        return
    fetch_source()


def fetch_source():
    logging.info('Fetching source...')
    download_source_file('main.py')
    try:
        download_source_file('props.json')
    except requests.exceptions.HTTPError:
        download_source_file('props.yaml')


def get_auth_header():
    if os.environ.get('AUTH_HEADER'):
        return os.environ['AUTH_HEADER']
    elif os.environ.get('TOKEN'):
        return f'Device {os.environ["TOKEN"]}'
    return ''


def download_source_file(filename):
    base_url = os.environ.get('PROJECT_URL') or f"https://cloud.cakerobotics.com/api/devices/src"
    url = f'{base_url}/{filename}'
    headers = {'authorization': get_auth_header()}
    response = requests.get(url, headers=headers)
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


def connect_to_devices_service():
    token = os.environ.get('TOKEN')
    if not token:
        logging.info('No device token provided. Skipping socket connection.')
        return
    devices_url = 'https://cloud.cakerobotics.com/'
    socketio_path = '/api/devices/socket'
    logging.info(f'Connecting to socket at {devices_url}...')
    socket_client.connect(
        devices_url,
        auth={'token': token},
        socketio_path=socketio_path,
        wait_timeout=10
    )
    logging.info(f'Connected.')


def start_app():
    global app_process
    logging.info('Starting the application...')
    app_process = subprocess.Popen(
        ['python3', '-u', 'main.py'],
        cwd='/app',
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        encoding='utf-8',
    )


def handle_logs():
    for line in iter(app_process.stdout.readline, ''):
        print(">>> " + line.rstrip())
        if socket_client.connected:
            socket_client.emit('log', line.rstrip())


def print_exit_code():
    logging.info(f'App finished with exit code {app_process.poll()}.')


def stop_app():
    # app_process.send_signal(9)
    app_process.kill()
    app_process.wait()


@socket_client.on('restart')
def on_restart():
    stop_app()
    fetch_source()
    ensure_requirements()
    start_app()
    handle_logs()
    print_exit_code()


@socket_client.on('stop')
def on_stop():
    stop_app()


@socket_client.on('error')
def on_error(msg):
    logging.error(msg)


@socket_client.on('accept')
def on_establish_ok():
    logging.info('Socket established.')


if __name__ == "__main__":
    main()
