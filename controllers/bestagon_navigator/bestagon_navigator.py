"""bestagon_navigator controller."""

import numpy as np
from controller import Robot, Lidar, Camera

from typing import *
from numpy.typing import NDArray

from mecanum_driver import MecanumDriver
from youbot_arm_driver import YoubotArmDriver

robot = Robot()

TIMESTEP = int(robot.getBasicTimeStep())
CONTROL_TIMESTEP = TIMESTEP * 10

# for name, device in robot.devices.items():
#     print(name, type(device))

# -- Devices -- #
wheels_driver = MecanumDriver(robot)
lidar: Lidar = robot.getDevice('front-box-lidar')
lidar.enablePointCloud()
# camera: Camera = robot.getDevice('front-box-cam')

lidar.enable(int(CONTROL_TIMESTEP))
# camera.enable(100)
#lidar.disable()

# print('RES:', lidar.getHorizontalResolution())


def sleep(time: float) -> None:
    robot.step(int(time * 1000))


arm_driver = YoubotArmDriver(robot, CONTROL_TIMESTEP)
# arm_driver.reset(True)
# arm_driver.pose('floor', False)

depth_resolution = lidar.getHorizontalResolution()
depth_x = np.linspace(-1, 1, depth_resolution)

def read_depth() -> NDArray:
    return np.array(list(map(
        lambda x: (x[0], np.nan) if np.isinf(x[1]) or np.isnan(x[1]) else x,
        zip(
            depth_x,
            lidar.getRangeImage(),
        )
    )))


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


def correct_coordinates(depth: NDArray, fov: float) -> NDArray:
    result = depth.copy()
    result[:, 0] *= np.sin(depth[:, 0] * fov * .5) * depth[:, 1]
    return result


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



def edges_detection(depth: NDArray, thresh=180e-6) -> NDArray:
    depth_cleaned = depth[~np.isnan(depth[:, 1])]
    depth_y = depth_cleaned[:, 1].copy()

    if len(depth_y) < 2: return np.zeros((0, 2, 2), dtype=float)

    filter_size = 5
    filter_mid = filter_size // 2
    filter = np.ones(filter_size) / filter_size

    depth_filtered = np.convolve(depth_y, filter, 'valid')
    diff2 = np.abs(np.diff(depth_filtered, 2))

    edges: list[tuple[NDArray, NDArray]] = []
    boundaries = diff2 >= thresh

    print(''.join(list(map(lambda x: 'T' if x else 'F', boundaries))), len(depth))
    print_depth(diff2 * 1e4)

    hist = np.histogram(diff2, 10)
    print('Histogram:')
    print('----------')
    print(' '.join(map(lambda x: f'{x:7d}', hist[0])))
    print(' '.join(map(lambda x: f'{x:.5f}', hist[1])))

    # print('HIST:', ' '.join(map(lambda x: f'{x:.5f}', np.histogram(acc, 10)[0])))
    # print('max:', np.max(acc))

    previous, skip = 0, False
    for i, boundary in enumerate(np.concatenate([boundaries, [True]])):
        if not boundary: skip = False
        if skip or not boundary: continue

        segment = depth_cleaned[previous:i+2+filter_mid]
        edges.append((tuple(segment[0]), tuple(segment[-1])))
        previous, skip = i+1, True

    return np.array(edges)


while robot.step(CONTROL_TIMESTEP) != -1:
    depth = read_depth()
    depth = correct_coordinates(depth, lidar.getFov())
    # print_depth(depth, 'Depth:')
    # diff = np.diff(depth)
    # print_depth(diff * 10, ' Diff:  ')
    # print(' | '.join(list(map(depth2str, objects))))
    # d2 = np.diff(depth[:, 1], 2)
    # print_depth(d2 * 100, 'Diff2:    ')

    # print(depth)
    objects = objects_segmentation(depth)
    objects = list(filter(lambda o: not np.all(np.isnan(o)), objects))
    objects_edges = list(map(edges_detection, objects))
    edges = np.concatenate(objects_edges[:1])

    # edge_detection(depth)

    # print(edge_detection(depth))
    print('Detected Objects:', len(objects))
    print('Edges:')
    print('------')
    for edge in edges:
        center = np.average(edge, axis=0)
        angle = np.arctan2(edge[1, 1] - edge[0, 1], edge[1, 0] - edge[0, 0])
        length = np.sqrt(np.sum(np.square(edge[0] - edge[1])))
        print(f'• Center: ({center[0]}, {center[1]}), Length: {length}, Angle: {angle * 180 / np.pi:.1f}°')
    

    pass

# Enter here exit cleanup code.
