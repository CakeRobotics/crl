FROM ros:galactic-ros-base-focal
RUN apt-get update
RUN apt-get install -y python3-pip
COPY . /crl
WORKDIR /crl
RUN pip3 install .
CMD ["python3", "-m", "pytest"]
