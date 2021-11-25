# pyright: reportUnusedCoroutine=false

import cake
from getch import Getch

def main():
    robot = cake.Robot()
    getch = Getch()

    print("Controller started...")
    print("Use WASD + Space to control the robot.")
    print("Use Shift+Q to close the program.")

    direction = 0

    while True:
        char = getch()
        if char == 'w':
            robot.wheels.set_speed(0.5)
            robot.wheels.set_rotation_rate(0)
            direction = 1
        elif char == 's':
            robot.wheels.set_speed(-0.5)
            robot.wheels.set_rotation_rate(0)
            direction = -1
        elif char == 'd':
            robot.wheels.set_rotation_rate(-0.5 * direction)
        elif char == 'a':
            robot.wheels.set_rotation_rate(0.5 * direction)
        elif char == 'x' or char == ' ':
            robot.wheels.set_speed(0)
            robot.wheels.set_rotation_rate(0)
        elif char == 'Q':
            break


if __name__ == '__main__':
    main()
