import asyncio

from cake.modules.hardware.wheels.Wheels import Wheels
from cake.modules.navigation.Navigation import Navigation
from cake.runtime.runtime import Runtime
from cake.utils.block_until_gazebo_runs import block_until_gazebo_runs
from cake.utils.load_props_from_file import load_props_from_file
from cake.utils.try_detect_project_dir import try_detect_project_dir
import cake.ros.ros1_bridge_external_nodes as ros1_bridge_external_nodes

class Robot:
    def __init__(self, props=None):
        self.init_stub()
        self.runtime = Runtime(self)
        try:
            self.load_props(props)
            self.init_from_props()
        except Exception as exception:
            self.runtime.shutdown()
            raise exception
        if self.props.get('sim') != True and self.props.get('ros1_port'):
            self.start_ros1_bridge()
        self.runtime.ros_interface.init_tf(self.props.get('sim'))
        if self.props.get('sim') == True:
            block_until_gazebo_runs(self.runtime)

    def shutdown(self):
        self.shutdown_modules()
        self.runtime.shutdown()

    def init_stub(self):
        self.wheels = Wheels(self)
        self.navigation = Navigation(self)

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
        self.navigation.init(self.props)

    def shutdown_modules(self):
        @self.runtime.run_in_event_loop
        async def body():
            await self.wheels.shutdown()
            await self.navigation.shutdown()
            await asyncio.sleep(0.1)  # Allow rclpy to tick
        body()

    def start_ros1_bridge(self):
        @self.runtime.run_in_event_loop
        async def body():
            ros1_bridge_launch_description = ros1_bridge_external_nodes.generate_launch_description(self.props)
            self.runtime.ros_interface.launch_external_nodes(ros1_bridge_launch_description)
        body()

    def health(self):
        return True
