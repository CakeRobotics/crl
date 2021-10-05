# pyright: reportUnusedCoroutine=false

import time
import cake


def main():
    props = {
        'name': 'name',
        'hardware': {
            'wheels-1': {
                'type': 'wheels',
                'sim': True,
            }
        }
    }

    robot = cake.Robot(props)
    while True:
        robot.wheels.set_speed(2)
        time.sleep(4)
        robot.wheels.set_speed(-2)
        time.sleep(4)


if __name__ == '__main__':
    main()
