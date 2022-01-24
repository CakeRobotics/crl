import cake_bundler.get_dockerfile

def test_get_dockerfile():
    props = {
        'pip_requirements': ['numpy', 'scipy']
    }
    dockerfile_content = cake_bundler.get_dockerfile.get_dockerfile(props)
    expected_lines = \
"""FROM ros:galactic-ros-base-focal
RUN apt-get update
RUN pip3 install git+https://github.com/CakeRobotics/crl
RUN pip3 install numpy scipy
COPY . /app
CMD ["python3", "-u", "/app/main.py"]"""
    for line in expected_lines:
        assert line in dockerfile_content


def test_get_dockerfile_with_local_crl():
    dockerfile_content = cake_bundler.get_dockerfile.get_dockerfile({}, local_crl=True)
    assert 'COPY crl /crl' in dockerfile_content
    assert 'WORKDIR /crl' in dockerfile_content
    assert 'pip3 install .' in dockerfile_content
