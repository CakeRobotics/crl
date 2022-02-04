import asyncio
from datetime import datetime

# Sleeps for n seconds but bails if break_timestamp is exceeded.
async def clamped_sleep(seconds, break_timestamp):
    time_to_break = break_timestamp - datetime.now().timestamp()
    time_to_sleep = min(seconds, time_to_break)
    await asyncio.sleep(time_to_sleep)
