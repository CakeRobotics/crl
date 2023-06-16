# Multi-robot Operation in a Structured Grid Environment
# pyright: reportUnusedCoroutine=false

import cake

print("Starting the program...")
robot = cake.Robot()

print("Joining the fleet...")
robot.fleet.join()

print("Program finished.")
robot.shutdown()
