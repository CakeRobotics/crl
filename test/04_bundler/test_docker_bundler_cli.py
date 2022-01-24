from subprocess import check_output
import os.path
import os

def test_docker_bundler_cli():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    sample_project_dir = os.path.join(script_dir, 'sample_project')

    expected_dockerfile_path = os.path.join(sample_project_dir, 'Dockerfile')
    if os.path.exists(expected_dockerfile_path):
        os.remove(expected_dockerfile_path)

    response_binary = check_output(['python3', '-m', 'cake_bundler', sample_project_dir])
    response = response_binary.decode('utf-8')
    assert "Dockerfile generated." in response
    assert os.path.exists(expected_dockerfile_path)
