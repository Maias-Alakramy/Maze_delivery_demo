import numpy as np
from controller import Robot, Motor

from typing import *
from numpy.typing import NDArray


class MecanumDriver:
    def __init__(self, robot: Robot) -> None:
        self.motors = MecanumDriver.__get_and_setup_motors(robot)
        
        self.max_motors_velocity = np.array([
            motor.getMaxVelocity()
            for motor in self.motors
        ])

        self.stop()


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
    def __get_and_setup_motors(robot: Robot) -> list[Motor]:
        motors = []

        for i in range(4):
            motor: Motor = robot.getDevice(f'wheel{i+1}')
            motor.setPosition(np.Inf)
            motors.append(motor)

        return motors
