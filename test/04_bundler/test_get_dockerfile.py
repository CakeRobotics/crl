import cake.bundler.get_dockerfile

def test_get_dockerfile():
    props = {
        'pip_requirements': ['numpy', 'scipy']
    }
    dockerfile_content = cake.bundler.get_dockerfile.get_dockerfile(props)
    expected_lines = \
"""FROM ros:galactic-ros-base-focal
RUN apt-get update
RUN apt-get install -y python3-pip
RUN pip3 install git+https://github.com/CakeRobotics/crl
RUN pip3 install numpy scipy
COPY . /app
CMD ["python3", "/app/main.py"]"""
    assert expected_lines in dockerfile_content
