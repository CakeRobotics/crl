#!/bin/bash
set -e
apt-get update
apt-get install -y python3-pip
source /opt/ros/galactic/setup.bash
pip3 install .
python3 -m pytest
