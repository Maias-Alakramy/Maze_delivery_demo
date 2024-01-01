import numpy as np
from controller import Robot

from typing import *
from numpy.typing import NDArray

from utilities import sleep, clamp, get_magnitude, TO_RADIAN

from mecanum_driver import MecanumDriver
from youbot_arm_driver import YoubotArmDriver
from lidar_vision import LiDARVision, LiDARObject



class KukaProgram:
    def __init__(self, robot: Robot, timestep: int | None = None) -> None:
        self.robot = robot
        self.timestep = int(timestep or robot.getBasicTimeStep())

        self.wheels = MecanumDriver(robot)
        
        self.arm = {
            'front': YoubotArmDriver(robot, self.timestep),
            'back': YoubotArmDriver(robot, self.timestep, prefix='front '),
        }

        self.lidar = {
            'front': LiDARVision(robot, 'front-lidar'),
            'back': LiDARVision(robot, 'back-lidar'),
            'left': LiDARVision(robot, 'left-lidar'),
            'right': LiDARVision(robot, 'right-lidar'),
        }

        self.capture_positions = {
            # {arm_side}-{side}
            'front-front': np.array([.0, .445]),
            'back-back': np.array([.0, .445]),
        }
    

    def enable(self) -> None:
        for lidar in self.lidar.values():
            lidar.enable(self.timestep)
    

    def disable(self) -> None:
        for lidar in self.lidar.values():
            lidar.disable()
    
    
    def reset(self, block=True) -> None:
        self.wheels.stop()
        for arm in self.arm.values(): arm.reset(False)
        if not block: return
        for arm in self.arm.values(): arm.wait()


    def sleep(self, duration: float) -> None:
        sleep(self.robot, duration)
    

    def detect_box(self, side='front') -> tuple[str, LiDARObject] | tuple[None, None]:
        lidars = self.lidars if side =='all' else { side: self.lidar[side] }

        for side, lidar in lidars.items():
            lidar.update()

            if len(lidar.objects) == 0: continue

            # print(list(map(lambda o: o.bounding_box[0], lidar.objects)))

            boxes = filter(
                lambda o: .028 <= o.bounding_box[0] <= .043,
                lidar.objects,
            )

            boxes = sorted(boxes, key=lambda o: o.mean[1])
            if len(boxes) == 0: continue

            return side, boxes[0]

        return None, None


    def align_box(
            self, side='front', arm_side='front', *, 
            max_power=.22,
        ) -> float:
        """
        Detects and aligns the box with a location suitable for
        grabbing the box by an arm.

        Returns the angle of the box after alignment in radian.
        """

        capture_position = self.capture_positions[f'{arm_side}-{side}']
        
        while self.robot.step(self.timestep) != -1:
            box = self.detect_box(side)[1]

            if box is None:
                raise Exception('No box detected by LiDAR sensor')

            box_center, box_angle = box.cube_info()

            if np.any(np.isnan(box_center)) or np.isnan(box_angle):
                raise Exception('Invalid box')

            delta_position = box_center - capture_position

            movement = np.array([
                delta_position[0] * 2.5,
                delta_position[1] * 2.5,
            ])

            direction = np.arctan2(movement[1], movement[0]) - .5*np.pi
            power = clamp(get_magnitude(movement), 0, max_power)

            self.wheels.move(direction, power)

            if power < 5e-3: break
        
        self.wheels.stop()
        return box_angle if box_angle is not None else np.nan
    

    def capture_box(self, box_angle: float, side='front', arm_side='front') -> None:
        """
        Captures a Kuka box considering it's aligned.
        """

        arm = self.arm[arm_side]

        arm.release(False) # Prepare the gripper.
        arm.pose(side) # Rotate to the specified side.

        arm.pose('pre_floor_1', threshold=5e-1)
        arm.pose('pre_floor_2', threshold=5e-1)
        arm.pose('pre_floor_3', threshold=5e-1)
        arm.pose('floor') # Get into the floor level.
        
        arm[-2] = -box_angle # Rotate the gripper to suite the box.
        arm.wait()

        arm.grab() # Grab the box


        arm.pose('pre_floor_3', threshold=5e-1)
        arm[-2] = 0 # Reset the gripper to avoid hitting a wall.
        arm.wait(slice(-2, -1))
        arm.pose('pre_floor_2', threshold=5e-1)
        arm.pose('pre_floor_1', threshold=5e-1)
        arm.pose('pre_floor_0', threshold=5e-1)

        arm.reset() # Reset the arm to the default position.
    

    def align_and_capture_box(self, side='front', arm_side='front') -> None:
        box_angle = self.align_box(side, arm_side)
        self.capture_box(box_angle, side, arm_side)


    def exchange_box(self, *, to: Literal['front']|Literal['back']='back') -> None:
        """
        Exchanges the box between the arms.
        """

        primary = self.arm['back' if to == 'front' else 'front']
        secondary = self.arm[to]

        primary.reset()
        secondary.reset()

        exchange_angle = 8 * TO_RADIAN

        secondary.release(False)
        secondary.arm = [.0, -exchange_angle, exchange_angle, .5 * np.pi, .0]
        secondary.wait()

        primary[:] = [.0, -exchange_angle, exchange_angle, .5 * np.pi, .5 * np.pi]
        primary.wait()

        secondary.grab()
        primary.release()

        # Spin both grippers simultaneously.
        primary[-2] = .0
        secondary[-2] = .5 * np.pi
        secondary.wait()
        primary.wait()

        secondary.reset()
        primary.reset()



