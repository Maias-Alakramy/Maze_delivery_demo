import numpy as np
from controller import Robot

from typing import *
from numpy.typing import NDArray

TO_RADIAN = np.pi / 180
TO_DEGREE = 180 / np.pi

def sleep(robot: Robot, time: float) -> None:
    robot.step(int(time * 1000))


def clamp(value: float | int, min: float | int, max: float | int) -> float | int:
    if value < min: return min
    if value > max: return max
    return value


def get_magnitude(vector: NDArray) -> float:
    return np.sqrt(np.sum(np.square(vector), -1))


def normalize(vector: NDArray) -> NDArray:
    return vector / get_magnitude(vector)