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
    robot.step(int(time * 1000))


wheels_driver = MecanumDriver(robot)

# motors_driver.normalized_motors_velocity = [1, 0, 0, 0]
# sleep(1)
# motors_driver.normalized_motors_velocity = [0, 1, 0, 0]
# sleep(1)
# motors_driver.stop()

wheels_driver.move('left')
sleep(.3)

angle = .5 * np.pi

while robot.step(CONTROL_TIMESTEP) != -1:
    dt = CONTROL_TIMESTEP / 1000
    angle += dt * np.pi * .5
    angle %= np.pi * 2

    wheels_driver.move(angle, turn=np.pi*.1)

# Enter here exit cleanup code.
