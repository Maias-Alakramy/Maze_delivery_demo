"""bestagon_navigator controller."""

import numpy as np
from controller import Robot

from typing import *
from numpy.typing import NDArray

from mecanum_driver import MecanumDriver

robot = Robot()

TIMESTEP = int(robot.getBasicTimeStep())
CONTROL_TIMESTEP = TIMESTEP * 4

for name, device in robot.devices.items():
    print(name, type(device))

# -- Devices -- #
wheels_driver = MecanumDriver(robot)


def sleep(time: float) -> None:
    robot.step(int(time * 1000))


while robot.step(CONTROL_TIMESTEP) != -1:
    pass

# Enter here exit cleanup code.
