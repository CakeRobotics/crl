# pyright: reportUnusedCoroutine=false

import cake

print("Starting the program...")
robot = cake.Robot()

print("Exploring the room...")
robot.navigation.explore(method='full_scan', timeout=3000)

print("Moving to the starting point...")
robot.navigation.move_to(0, 0)

print("Program finished.")
robot.shutdown()
