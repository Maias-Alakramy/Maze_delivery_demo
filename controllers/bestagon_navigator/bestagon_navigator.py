"""bestagon_navigator controller."""

import numpy as np
from controller import Robot, Lidar

from typing import *
from numpy.typing import NDArray

from mecanum_driver import MecanumDriver
from youbot_arm_driver import YoubotArmDriver

robot = Robot()

TIMESTEP = int(robot.getBasicTimeStep())
CONTROL_TIMESTEP = TIMESTEP * 10


# -- Devices -- #
wheels_driver = MecanumDriver(robot)
arm_driver = YoubotArmDriver(robot, CONTROL_TIMESTEP)
back_arm_driver = YoubotArmDriver(robot, CONTROL_TIMESTEP, prefix='front ')

lidar: Lidar = robot.getDevice('front-box-lidar')
lidar.enablePointCloud()

lidar.enable(int(CONTROL_TIMESTEP))

def sleep(time: float) -> None:
    robot.step(int(time * 1000))



depth_resolution = lidar.getHorizontalResolution()
lidar_x = np.linspace(-1, 1, depth_resolution)

def read_depth() -> NDArray:
    cloud_points = lidar.getPointCloud()
    depth = np.full((len(cloud_points), 2), np.nan)

    # cloud_points.reverse()
    for i, point in enumerate(cloud_points):
        coords = (-point.y, point.x)
        if np.any(np.isnan(coords) | np.isinf(coords)): continue
        depth[i] = coords
    
    return depth


def depth2str(depth: NDArray) -> str:
    return ' '.join(list(map(
        lambda x: '---' if np.isnan(x) else '###' if np.isinf(x) else f'{int(x*100):3d}',
        map(
            lambda x: x if type(x) is not np.ndarray else x[1],
            depth,
        ),
    )))


def print_depth(depth: NDArray, name='') -> None:
    print(name, depth2str(depth))


def objects_segmentation(depth: NDArray, thresh=5e-2) -> list[NDArray]:
    depth_fixed = depth[:, 1].copy()
    depth_fixed[np.isnan(depth_fixed) | np.isinf(depth_fixed)] = -1e2

    objects: list[NDArray] = []
    boundaries = np.abs(np.diff(depth_fixed)) >= thresh

    previous = 0
    for i, boundary in enumerate(np.concatenate([boundaries, [True]])):
        if not boundary: continue
        objects.append(depth[previous:i+1])
        previous = i+1
    
    return objects



def edges_detection(depth: NDArray, thresh=120e-6) -> list[NDArray]:
    depth_cleaned = depth[~np.isnan(depth[:, 1])]
    depth_y = depth_cleaned[:, 1].copy()

    if len(depth_y) < 2: return np.zeros((0, 2, 2), dtype=float)

    filter_size = 5
    filter = np.ones(filter_size) / filter_size

    depth_filtered = np.convolve(depth_y, filter, 'valid')
    diff2 = np.abs(np.diff(depth_filtered, 2))

    edges: list[NDArray] = []
    boundaries = diff2 >= thresh

    # print(''.join(list(map(lambda x: '#' if x else '-', boundaries))), len(depth))
    # print_depth(diff2 * 1e4)

    # hist = np.histogram(diff2, 10)
    # print('Histogram:')
    # print('----------')
    # print(' '.join(map(lambda x: f'{x:7d}', hist[0])))
    # print(' '.join(map(lambda x: f'{x:.5f}', hist[1])))

    previous, skip = 0, False
    for i, boundary in enumerate(np.concatenate([boundaries, [True]])):
        if not boundary: skip = False
        elif skip: previous = i

        if skip or not boundary: continue

        end = clamp(i+2+filter_size, previous, depth_cleaned.shape[0]-1)
        segment = depth_cleaned[previous:end]
        edges.append(segment)
        previous, skip = i+1, True

    return edges


class LiDARObject:
    def __init__(self, depth: NDArray) -> None:
        self.depth = depth
        self.width = np.abs(depth[0, 0] - depth[-1, 0])

        self.mean = np.average(depth, axis=0)
        self.edges = edges_detection(depth)

        self.edges_count = len(self.edges)
        # print('Edges Samples:', list(map(len, self.edges)))
    

    def cube_info(self) -> tuple[NDArray, float]:
        edge = None
        length = -1

        for _edge in self.edges:
            if len(_edge) > length:
                edge, length = _edge, len(_edge)
        
        padding = edge.shape[0] // 3

        angle = np.average(np.arctan2(
            np.diff(edge[padding:-padding, 1]),
            np.diff(edge[padding:-padding, 0]),
        ))

        edge_center = np.average(edge[padding:-padding], 0)

        center = edge_center + np.array([
            np.cos(angle + .5 * np.pi),
            np.sin(angle + .5 * np.pi),
        ]) * .014

        return center, angle


def clamp(value: float, min: float, max: float) -> float:
    if value < min: return min
    if value > max: return max
    return value


def get_magnitude(vector: NDArray) -> float:
    return np.sqrt(np.sum(np.square(vector), -1))

arm_driver.reset()
back_arm_driver.reset()

def exchange_box(reverse = False):
    primary = back_arm_driver if reverse else arm_driver
    secondary = arm_driver if reverse else back_arm_driver

    primary[:] = [.0, .0, .0, .0, .0]
    primary.wait()

    exchange_angle = 8 / 180 * np.pi

    secondary.release(False)
    secondary.arm = [.0, -exchange_angle, exchange_angle, .5 * np.pi, .0]
    secondary.wait()

    primary[:] = [.0, -exchange_angle, exchange_angle, .5 * np.pi, .5 * np.pi]
    primary.wait()

    secondary.grab()
    primary.release()

    primary[-2] = .0
    secondary[-2] = .5 * np.pi

    secondary.wait()
    primary.wait()

    secondary.reset()
    primary.reset()


while robot.step(CONTROL_TIMESTEP) != -1:
    depth = read_depth()

    objects_depth = objects_segmentation(depth)
    objects_depth = list(filter(lambda o: not np.all(np.isnan(o)), objects_depth))
    objects = list(map(LiDARObject, objects_depth))

    if len(objects) == 0:
        # print("I've failed sir..")
        wheels_driver.stop()
        continue
    
    box = objects[0]
    box_center, box_angle = box.cube_info()

    grab_position = np.array([.0, .445])

    delta_position = box_center - grab_position

    direction = np.sign(delta_position)
    direction = direction / get_magnitude(direction)

    speed = np.array([
        delta_position[0] * 2.5,
        delta_position[1] * 2.5,
    ])

    max_speed = .22
    speed_angle = np.arctan2(speed[1], speed[0]) - .5 * np.pi
    magnitude = clamp(get_magnitude(speed), 0, max_speed)

    wheels_driver.move(speed_angle, magnitude)

    # print(f'Cube angle: {box_angle * 180 / np.pi:.1f}° • Objects: {len(objects)}')
    if magnitude < 5e-3: break


# Enter here exit cleanup code.

wheels_driver.stop()
lidar.disablePointCloud()
lidar.disable()

arm_driver.release(False)
arm_driver.pose('floor')

arm_driver[-2] = -box_angle
arm_driver.wait()
arm_driver.grab()

arm_driver.reset()

exchange_box()
sleep(.5)
exchange_box(True)
