#!/bin/bash
set -e
apt-get update
apt-get install -y python3-pip
source /opt/ros/galactic/setup.bash
# Nasty backport (delete after upgrading to python 3.9)
curl https://raw.githubusercontent.com/python/cpython/3.9/Lib/asyncio/events.py \
    --output /usr/lib/python3.8/asyncio/events.py
pip3 install .
python3 -m pytest -x -s -v
