from cake.modules.hardware.wheels.Wheels import Wheels
from cake.runtime.runtime import Runtime
from cake.utils.load_props_from_file import load_props_from_file
from cake.utils.try_detect_project_dir import try_detect_project_dir

class Robot:
    def __init__(self, props=None):
        self.init_stub()
        self.runtime = Runtime()
        try:
            self.load_props(props)
            self.init_from_props()
        except Exception as exception:
            self.runtime.shutdown()
            raise exception

    def shutdown(self):
        self.runtime.shutdown()

    def init_stub(self):
        self.wheels = Wheels(self)

    def load_props(self, explicitly_provided_props):
        if explicitly_provided_props is not None:
            self.props = explicitly_provided_props
        else:
            project_dir = try_detect_project_dir()
            if project_dir is None:
                raise Exception(
                    "Can't detect project directory. Make sure the robot "
                    "code is inside a file named `main.py`."
                )
            self.props = load_props_from_file(project_dir)

    def init_from_props(self):
        self.wheels.init(self.props)

    def health(self):
        return True
