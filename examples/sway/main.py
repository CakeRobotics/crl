# pyright: reportUnusedCoroutine=false

import time
import cake


def main():
    robot = cake.Robot()
    while True:
        print("Going forward")
        robot.wheels.set_speed(0.5)
        time.sleep(4)
        print("Going back")
        robot.wheels.set_speed(-0.5)
        time.sleep(4)


if __name__ == '__main__':
    main()
