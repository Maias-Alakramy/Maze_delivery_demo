"""bestagon_navigator controller."""

import numpy as np
from controller import Robot

from typing import *
from numpy.typing import NDArray


from kuka_program import KukaProgram

robot = Robot()

TIMESTEP = int(robot.getBasicTimeStep())
CONTROL_TIMESTEP = int(TIMESTEP * 4)

kuka_program = KukaProgram(robot, CONTROL_TIMESTEP)

kuka_program.enable()
kuka_program.reset()

print('- Searching for box.')
kuka_program.wheels.move('forward')

def search_box() -> str:
    while robot.step(CONTROL_TIMESTEP) != -1:
        side = kuka_program.detect_box('all')[0]
        if side is not None: return side

box_side = search_box()
box_arm = 'back' if box_side == 'back' else 'front'

print(f'Found box at side "{box_side}", using arm "{box_arm}" to capture it.')

print('- Aligning box.')
box_angle = kuka_program.align_box(box_side, box_arm)

print('- Capturing box.')
kuka_program.capture_box(box_angle, box_side, box_arm)

# Or both steps at once:
# kuka_program.align_and_capture_box()

if box_arm == 'front':
    print('- Exchanging box.')
    kuka_program.exchange_box(to='back')
    kuka_program.sleep(.5)

print('- Resetting Robot.')
kuka_program.reset()
kuka_program.disable()



# while robot.step(CONTROL_TIMESTEP) != -1:
#     pass