#!/bin/bash
set -e
apt-get update
apt-get install -y python3-pip
pip3 install .
python3 -m pytest
