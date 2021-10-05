from cake.modules.hardware.wheels.Wheels import Wheels
from cake.runtime.runtime import Runtime

class Robot:
    def __init__(self, props=None):
        self.init_stub()
        self.runtime = Runtime()
        try:
            if props is not None:
                self.init_from_props(props)
        except Exception as exception:
            self.runtime.shutdown()
            raise exception

    def shutdown(self):
        self.runtime.shutdown()

    def init_stub(self):
        self.wheels = Wheels(self)

    def init_from_props(self, props):
        hw = props['hardware']
        self.wheels.init({k: v for k, v in hw.items() if v['type'] == 'wheels'})

    def health(self):
        return True
