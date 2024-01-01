import numpy as np

from typing import *
from numpy.typing import NDArray

from controller import Robot, Motor


class YoubotArmDriver:
    def __init__(self, robot: Robot, timestep: int | None = None, *, prefix=''):
        self.robot = robot
        self.timestep = int(timestep or robot.getBasicTimeStep())

        self.motors = [
            YoubotArmDriver.__get_motor(robot, f'{prefix}arm{i+1}')
            for i in range(5)
        ]

        self.motors.append(YoubotArmDriver.__get_motor(
            robot, f'{prefix}finger::left',
        ))

        for motor in self.motors:
            motor.getPositionSensor().enable(self.timestep)

        self.motors_limits = [
            (m.getMinPosition(), m.getMaxPosition())
            for m in self.motors
        ]

        self.poses = {
            'reset': [.0, .0, .0, .0, .0, self.motors_limits[-1][0]], #[.0, 1.57, -2.635, 1.78, .0, self.motors_limits[-1][0]],
            'packed': [None, 1.57, -2.635, 1.78, .0],
            '3_5_packed': [None, None, -2.635, 1.78, .0],

            # -- Height Presets -- #
            'pre_floor_0': [None, 1, -.8, -.5],
            'pre_floor_1': [None, 1, -2.3, -1.0],
            'pre_floor_2': [None, -.5, -2.3, -.3],
            'pre_floor_3': [None, -.5, -2, -.5],
            'floor': [None, -.97, -1.55, -.61],
            'plate_front': [None, -.62, -.98, -1.53],
            'plate_back_high': [None, .678, .682, 1.74],
            'plate_back_low': [None, .92, .42, 1.78],
            'cardboard': [None, .0, -.77, -1.21],

            # -- Orientation Presets -- #
            'front': [.0],
            'left': [.5 * np.pi],
            'right': [-.5 * np.pi],
            'back_left': [2.949],
            'back_right': [-2.949],

            # -- Gripper Presets -- #
            'grip': [None, None, None, None, None, self.motors_limits[-1][0]],
            'release': [None, None, None, None, None, self.motors_limits[-1][1]],
        }

    def grab(self, block=True) -> None:
        self.pose('grip', False)
        if block: self.wait(slice(-1, None))


    def release(self, block=True) -> None:
        self.pose('release', False)
        if block: self.wait(slice(-1, None))


    def reset(self, block=True) -> None:
        self.pose('reset', block)


    def pose(self, name: str, block=True, threshold: float=1e-3) -> None:
        self[:] = self.poses[name]
        if block: self.wait(threshold=threshold)
    
    
    def wait(
            self, indices=slice(None), *,

            threshold=1e-3,
            timeout:int=5000,

            stagnation_threshold=1e-3,
            stagnation_timeout:int=100,
        ):
        """Wait until movement is complete."""

        previous = None
        stagnation_timer = stagnation_timeout

        while self.robot.step(self.timestep) != -1:
            if timeout > 0:
                timeout -= self.timestep
                if timeout <= 0: return
            
            target = np.array(self.target[indices])
            position = np.array(self[indices])
            distance = np.sum(np.abs(target - position))

            if distance < threshold: return

            if previous is not None:
                diff = np.abs(previous - distance)
                stagnation_timer -= self.timestep
                if diff > stagnation_threshold:
                    stagnation_timer = stagnation_timeout
                elif stagnation_timer <= 0: return
            
            previous = distance
    
    
    @property
    def arm(self) -> list[float]:
        return self[:-2]


    @arm.setter
    def arm(self, value: list[None | int | float]) -> None:
        self[:-2] = value
    
    
    @property
    def target(self) -> list[float]:
        return list(map(
            lambda m: m.getTargetPosition(),
            self.motors,
        ))
    

    def normalize(self, value: list[None | int | float]) -> list[None | float]:
        result = []
        for k, v in enumerate(value):
            if k < len(self) and v is not None:
                v = self.normalize_arm_position(k, v)
            result.append(v)
        return result


    def normalize_arm_position(self, arm: int, position: int | float) -> float:
        return position / np.abs(self.motors_limits[arm][0 if position < 0 else 1])


    def un_normalize_arm_position(self, arm: int, position: int | float) -> float:
        return position * np.abs(self.motors_limits[arm][0 if position < 0 else 1])


    def __getitem__(self, key: int | slice) -> float:
        if type(key) is int:
            return self.motors[key].getPositionSensor().getValue()
        
        return list(map(
            lambda m: m.getPositionSensor().getValue(),
            self.motors[key]
        ))
    

    def __setitem__(self, key: int | slice, value: None | int | float | Sequence[None | int | float]) -> float:
        if type(key) is slice:
            for k, v in zip(
                range(*key.indices(len(self.motors))),
                value,
            ):
                if v is not None:
                    self.motors[k].setPosition(v)
        else:
            if value is not None:    
                self.motors[key].setPosition(value)


    def __len__(self) -> int:
        return len(self.motors)

    @staticmethod
    def __get_motor(robot: Robot, name: str) -> Motor:
        return robot.getDevice(name)