import asyncio

import rclpy.qos
from rosgraph_msgs.msg._clock import Clock


MIN_NUMBER_OF_CLOCK_MESSAGES = 2
MIN_GAZEBO_UPDATE_RATE_HZ = 10
SAFETY_FACTOR = 2
TIME_TO_SLEEP = SAFETY_FACTOR * MIN_NUMBER_OF_CLOCK_MESSAGES / MIN_GAZEBO_UPDATE_RATE_HZ


async def get_gazebo_state(cake_node):
    if not ('gazebo' in cake_node.get_node_names()):
        return 'NOT_RUNNING'

    global cake_clock_msg_count
    cake_clock_msg_count = 0
    def increment(_):
        global cake_clock_msg_count
        cake_clock_msg_count += 1

    cake_node.create_subscription(Clock, '/clock', increment, rclpy.qos.HistoryPolicy.KEEP_LAST)
    await asyncio.sleep(TIME_TO_SLEEP)

    if cake_clock_msg_count < MIN_NUMBER_OF_CLOCK_MESSAGES:
        return 'PAUSED'

    return 'RUNNING'
