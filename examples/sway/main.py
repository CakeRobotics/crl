# pyright: reportUnusedCoroutine=false

import time
import cake


def main():
    robot = cake.Robot()
    while True:
        robot.wheels.set_speed(0.5)
        time.sleep(4)
        robot.wheels.set_speed(-0.5)
        time.sleep(4)


if __name__ == '__main__':
    main()
