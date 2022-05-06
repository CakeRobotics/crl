from datetime import datetime
from random import random

from cake.utils.clamped_sleep import clamped_sleep

async def random_walk_open_loop(robot, timeout):
    t_end = datetime.now().timestamp() + timeout
    while datetime.now().timestamp() < t_end:
        await robot.wheels.set_speed(0.2)
        await robot.wheels.set_rotation_rate(2 * (random() - 0.5))
        await clamped_sleep(3, t_end)
        await robot.wheels.set_speed(0.2)
        await robot.wheels.set_rotation_rate(0)
        await clamped_sleep(4, t_end)
    await robot.wheels.set_speed(0.0)
    await robot.wheels.set_rotation_rate(0)
