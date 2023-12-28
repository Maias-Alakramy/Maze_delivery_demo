"""bestagon_navigator controller."""

import numpy as np
from controller import Robot, Lidar, Camera

from typing import *
from numpy.typing import NDArray

from mecanum_driver import MecanumDriver

robot = Robot()

TIMESTEP = int(robot.getBasicTimeStep())
CONTROL_TIMESTEP = TIMESTEP * 4

# for name, device in robot.devices.items():
#     print(name, type(device))

# -- Devices -- #
wheels_driver = MecanumDriver(robot)
lidar: Lidar = robot.getDevice('front-box-lidar')
camera: Camera = robot.getDevice('front-box-cam')

lidar.enable(100)
camera.enable(100)
#lidar.disable()

print('RES:', lidar.getHorizontalResolution())


def sleep(time: float) -> None:
    robot.step(int(time * 1000))


while robot.step(CONTROL_TIMESTEP) != -1:
    sleep(2)
    # data = lidar.getRangeImage()
    data = np.frombuffer(camera.getImage(), dtype=np.uint8)
    if data is None: continue
    # data = data[::4]

    data = list(map(lambda x: '-' if x > 1e10 else f'{x}', data))
    print(' '.join(data))

# Enter here exit cleanup code.
