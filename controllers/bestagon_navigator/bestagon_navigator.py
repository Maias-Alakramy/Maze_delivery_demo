"""bestagon_navigator controller."""

import numpy as np
from controller import Robot

from typing import *
from numpy.typing import NDArray


from kuka_program import KukaProgram

robot = Robot()

TIMESTEP = int(robot.getBasicTimeStep())
CONTROL_TIMESTEP = int(TIMESTEP * 10)

kuka_program = KukaProgram(robot, CONTROL_TIMESTEP)

kuka_program.enable()
kuka_program.reset()

print('- Aligning box.')
box_angle = kuka_program.align_box('right', 'front')

print('- Capturing box.')
kuka_program.capture_box(box_angle, 'right', 'front')

# Or both steps at once:
# kuka_program.align_and_capture_box()

print('- Exchanging box.')
kuka_program.exchange_box(to='back')
kuka_program.sleep(.5)

print('- Exchanging box back.')
kuka_program.exchange_box(to='front')

print('- Resetting Robot.')
kuka_program.reset()
kuka_program.disable()



# while robot.step(CONTROL_TIMESTEP) != -1:
#     pass