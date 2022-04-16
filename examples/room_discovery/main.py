# pyright: reportUnusedCoroutine=false

import cake
from time import sleep

print("Starting the program...")
robot = cake.Robot()

print("Exploring the room...")
robot.navigation.explore(timeout=20)

print("Moving to the starting point...")
robot.navigation.move_to(0, 0)

print("Program finished.")
robot.shutdown()
