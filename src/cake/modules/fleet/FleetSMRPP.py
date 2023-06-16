import asyncio
import os
import random
import socketio
from cake.runtime.runtime import run_in_event_loop
from .Fleet import Fleet


def generate_robot_id():
    if os.environ.get('AUTH_HEADER'):
        return os.environ['AUTH_HEADER'][:-5]
    elif os.environ.get('TOKEN'):
        return f'Device {os.environ["TOKEN"]}'[:-5]
    return f'r{random.randint(1000, 9999)}'


class FleetSMRPP(Fleet):
    def __init__(self, robot, props):
        self.robot = robot
        assert self.robot.navigation is not None
        self.fleet_key = props['fleet']['fleet_key']
        self.environment_model = props['fleet']['environment_model']
        self.robot_id = props['fleet']['robot_id']
        if self.robot_id == 'GENERATE':
            self.robot_id = generate_robot_id()
        self.en_route = False


    async def shutdown(self):
        pass


    @run_in_event_loop
    async def join_socket(self):
        self.socket_client = socketio.Client()
        mrpp_server_url = 'http://localhost:5050/'
        self.socket_client.connect(mrpp_server_url)


    @run_in_event_loop
    async def join(self, timeout=None):
        await self.join_socket()
        @self.socket_client.on('path_update')
        def path_update(path):
            self.path_update(path)
        while True:
            x, y, _ = await self.robot.navigation.get_position()
            h = await self.robot.navigation.get_heading()
            h = round(h/(3.14/8)) % 8  # adapt with back-end service
            data = {
                'env_key': self.fleet_key,
                'robot': self.robot_id,
                'state': [x, y, h],
                'reading': None,
            }
            self.socket_client.emit('field_report', data)
            await asyncio.sleep(1)


    @run_in_event_loop
    async def path_update(self, path):
        if path is None or len(path) == 0:
            return

        # Find the next waypoint with a position different to current position
        x_me, y_me, h_me, t_now = path[0]
        for i in range(0, len(path)):
            x, y, h, t = path[i]
            if x_me != x or y_me != y:
                break

        if self.en_route:
            print(f"Skipping {x}, {y}, {h}")
            return
        print(f"Moving to {x}, {y}, {h}")
        h *= 3.14 / 4
        await self.robot.navigation.move_to(x, y, h, wait_to_finish=True)
