from datetime import datetime
import jinja2
import os.path

# local_crl: Use COPY to copy current CRL library to build the image, instead of
# pulling CRL from GitHub.
#
def get_dockerfile(props, local_crl=False):
    template = _load_jinja_template()
    args = _get_template_args(props)
    dockerfile_content = template.render(local_crl=local_crl, **args)
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
