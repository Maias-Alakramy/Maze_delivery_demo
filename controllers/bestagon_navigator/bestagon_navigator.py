"""bestagon_navigator controller."""

import numpy as np
from controller import Robot

from typing import *
from numpy.typing import NDArray

from mecanum_driver import MecanumDriver

robot = Robot()

TIMESTEP = int(robot.getBasicTimeStep())
CONTROL_TIMESTEP = TIMESTEP * 4

# for name, device in robot.devices.items():
#     print(name, type(device))


def sleep(time: float) -> None:
    robot.step(time * 1000)


motors_driver = MecanumDriver(robot)

motors_driver.normalized_motors_velocity = [1, 0, 0, 0]
sleep(1)
motors_driver.normalized_motors_velocity = [0, 1, 0, 0]
sleep(1)

motors_driver.stop()


while robot.step(CONTROL_TIMESTEP) != -1:
    pass

# Enter here exit cleanup code.
