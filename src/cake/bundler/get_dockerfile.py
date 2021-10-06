from datetime import datetime
import jinja2
import os.path


def get_dockerfile(props):
    template = _load_jinja_template()
    args = _get_template_args(props)
    dockerfile_content = template.render(**args)
    return dockerfile_content


def _get_template_args(props):
    return {
        'pip_requirements': props.get('pip_requirements'),
        'date_string': str(datetime.now()),
    }


def _load_jinja_template():
    env = jinja2.Environment()
    loader = jinja2.FileSystemLoader(_get_template_dir())
    template = loader.load(env, 'Dockerfile.jinja2')
    return template


def _get_template_dir():
    bundler_module_dir = os.path.dirname(os.path.realpath(__file__))
    template_dir = os.path.join(bundler_module_dir, 'template')
    return template_dir
