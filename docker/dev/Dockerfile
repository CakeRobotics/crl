# This dockerfile generates a static image for development use.
# Using this dockerfile, you don't have to rebuild the image every
# time. However, the package size will not be optimal as the package
# comes with some pre-installed packages (e.g. numpy) for faster
# boot. Use cake_bundler to build efficient docker images for
# production.
#
# Usage:
# Either mount project files at /app directory or provide environment
# variables of PROJECT_URL, and AUTH_HEADER if your project
# is in the cloud.
#
# Note: build this image from repo root cwd for correct context.
#

FROM ros:galactic-ros-base-focal
RUN apt-get update
RUN apt-get install -y \
    curl \
    python3-pip \
    ros-galactic-navigation2 \
    ros-galactic-nav2-bringup \
    ros-galactic-slam-toolbox

RUN pip3 install \
    numpy \
    scipy

# Nasty backport (delete after upgrading to python 3.9)
RUN curl https://raw.githubusercontent.com/python/cpython/3.9/Lib/asyncio/events.py \
    --output /usr/lib/python3.8/asyncio/events.py

RUN curl https://raw.githubusercontent.com/m2-farzan/px/main/px --output /px && \
    chmod +x /px

COPY . /crl
WORKDIR /crl
RUN pip3 install .

WORKDIR /app
CMD ["bash", "/crl/docker/dev/main.bash"]
