import numpy as np
from enum import StrEnum

from typing import *
from numpy.typing import NDArray

from controller import Robot, Motor


class Direction(StrEnum):
    FORWARD = 'forward'
    LEFT = 'left'
    BACKWARD = 'backward'
    RIGHT = 'right'


direction_2_angle = {
    Direction.FORWARD: .0,
    Direction.BACKWARD: np.pi,
    Direction.LEFT: .5 * np.pi,
    Direction.RIGHT: -.5 * np.pi,

    'u': .0,
    'd': np.pi,
    'l': .5 * np.pi,
    'r': -.5 * np.pi,

    'up': .0,
    'down': np.pi,
    'f': .0,
    'b': np.pi,
    
    'north': .0,
    'south': np.pi,
    'west': .5 * np.pi,
    'east': -.5 * np.pi,

    'n': .0,
    's': np.pi,
    'w': .5 * np.pi,
    'e': -.5 * np.pi,
}


class MecanumDriver:
    def __init__(self, robot: Robot) -> None:
        self.motors = MecanumDriver.__get_and_setup_motors(robot)
        
        self.max_motors_velocity = np.array([
            motor.getMaxVelocity()
            for motor in self.motors
        ])

        self.stop()
    

    def move(self, angle: float | Direction | str=.0, power=1., turn=.0) -> None:
        if type(angle) is str: angle = direction_2_angle[angle.lower()]
        self.normalized_motors_velocity = MecanumDriver.calculate_normalized_velocity(
            angle, power, turn,
        )
    

    @property
    def motors_velocity(self) -> NDArray:
        return np.array([
            motor.getVelocity()
            for motor in self.motors
        ])
    

    @motors_velocity.setter
    def motors_velocity(self, value: NDArray | list[int]):
        for i, motor in enumerate(self.motors):
            motor.setVelocity(value[i])
    

    @property
    def normalized_motors_velocity(self) -> NDArray:
        return self.motors_velocity / self.max_motors_velocity


    @normalized_motors_velocity.setter
    def normalized_motors_velocity(self, value: NDArray | list[int]):
        self.motors_velocity = np.array(value) * self.max_motors_velocity
    

    def stop(self):
        self.motors_velocity = np.zeros(4)
    

    @staticmethod
    def calculate_normalized_velocity(angle: float, power: float, turn: float) -> NDArray:
        sin = np.sin(angle + np.pi * .25)
        cos = np.cos(angle + np.pi * .25)
        max = np.max(np.abs([sin, cos]))

        velocity = np.array([sin, cos, cos, sin])
        velocity = power * velocity / max
        velocity += np.array([-1, 1, -1, 1]) * turn

        abs_turn = np.abs(turn)
        if (power + abs_turn) > 1:
            velocity /= power + abs_turn
        
        return velocity
    

    @staticmethod
    def __get_and_setup_motors(robot: Robot) -> list[Motor]:
        motors = []

        for i in range(4):
            motor: Motor = robot.getDevice(f'wheel{i+1}')
            motor.setPosition(np.Inf)
            motors.append(motor)

        return motors
