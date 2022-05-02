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

COPY docker/dev/mock.bash /mock.bash
RUN /mock.bash \
    libqt5widgets5=5.0.2 \
    libqt5gui5 \
    libqt5opengl5 \
    ros-galactic-nav2-rviz-plugins \
    ros-galactic-nav2-waypoint-follower \
    ros-galactic-rviz-common \
    ros-galactic-rviz-default-plugins \
    ros-galactic-rviz-ogre-vendor \
    ros-galactic-rviz-rendering

RUN apt-get install -y --no-install-recommends \
    curl \
    python3-pip \
    ros-galactic-navigation2 \
    ros-galactic-nav2-bringup \
    ros-galactic-nav2-simple-commander \
    ros-galactic-slam-toolbox

# Install ROS 1
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    bash -c 'curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -' && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-ros-base \
        ros-galactic-ros1-bridge

# Nasty backport (delete after upgrading to python 3.9)
RUN curl https://raw.githubusercontent.com/python/cpython/3.9/Lib/asyncio/events.py \
    --output /usr/lib/python3.8/asyncio/events.py

COPY . /crl
WORKDIR /crl
RUN pip3 install .

RUN pip3 install -r /crl/docker/dev/controller-requirements.txt

WORKDIR /app
ENTRYPOINT ["python3", "-u", "/crl/docker/dev/controller.py"]
