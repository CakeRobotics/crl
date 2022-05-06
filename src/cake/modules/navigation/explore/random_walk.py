from asyncio import sleep
from datetime import datetime
import logging

import numpy as np

from cake import Robot
from cake.utils.map_utils import MapUtils


async def random_walk(robot: Robot, timeout):
    body_size = robot.props.get('body_size') or 0.5
    step_size = 2 * body_size
    map_agent = MapUtils(robot, '/map')
    await map_agent.start()

    async def main():
        t_end = now() + timeout
        while not map_agent.ready():
            await robot.wheels.set_speed(0.2)
            await sleep(0.5)
        await robot.wheels.stop()
        while now() < t_end:
            target = await generate_candidate_target()
            target_feasible = (await is_target_in_sight(target)) and map_agent.is_point_inside_map(target)
            if target_feasible:
                logging.debug(f'Target selected: {target}')
                map_agent.visualize_point(target, '/target')
                await robot.navigation.move_to(target[0], target[1])
            else:
                logging.debug('Target rejected.')

    def now():
        return datetime.now().timestamp()

    async def generate_candidate_target():
        current_position = await robot.navigation.get_position()
        current_position = np.asarray(current_position[0:2])
        current_angle = await robot.navigation.get_heading()
        angle_diff = np.random.normal(loc=0, scale=np.pi/3)
        displacement_angle = current_angle + angle_diff
        displacement = step_size * np.asarray([np.cos(displacement_angle), np.sin(displacement_angle)])
        candidate_target = current_position + displacement
        return candidate_target

    async def is_target_in_sight(target):
        # Break line-of-sight into 10 mid-points and assert
        # no occupied cell within body_size/2 radius of each point
        current_position = await robot.navigation.get_position()
        current_position = np.asarray(current_position[:-1])
        N = 10
        for i in range(N):
            checkpoint = i / N * current_position + (N - i) / N * target
            radius = body_size / 2
            distances = await map_agent.get_distmap(checkpoint)
            close_cells = np.where(distances < radius, True, False)
            occupied_cells = np.where(map_agent.get_map() > 0, True, False)
            close_occupied_cells = close_cells * occupied_cells
            if np.any(close_occupied_cells):
                return False
        return True

    await main()
