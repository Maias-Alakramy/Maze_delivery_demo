
import numpy as np
from controller import Robot, Lidar

from typing import *
from numpy.typing import NDArray

from utilities import clamp

def get_lidar(robot: Robot, name: str) -> Lidar:
    return robot.getDevice(name)



class LiDARDebug:
    @staticmethod
    def depth2str(depth: NDArray) -> str:
        return ' '.join(list(map(
            lambda x: '---' if np.isnan(x) else '###' if np.isinf(x) else f'{int(x*100):3d}',
            map(
                lambda x: x if type(x) is not np.ndarray else x[1],
                depth,
            ),
        )))


    @staticmethod
    def print_depth(depth: NDArray, name='') -> None:
        print(name, LiDARDebug.depth2str(depth))



class LiDARVision:

    def __init__(self, robot: Robot, name: str) -> None:
        self.sensor = get_lidar(robot, name)
        self.shape = (self.sensor.getHorizontalResolution(), 2)

        self.depth = np.full(self.shape, np.nan)
        self.objects: list[LiDARObject] = []


    def enable(self, sample_rate: int) -> None:
        self.sensor.enable(sample_rate)
        self.sensor.enablePointCloud()


    def disable(self) -> None:
        self.sensor.disablePointCloud()
        self.sensor.disable()
    

    def update_depth(self) -> None:
        cloud_points = self.sensor.getPointCloud()

        for i, point in enumerate(cloud_points):
            coords = (-point.y, point.x)
            if np.any(np.isnan(coords) | np.isinf(coords)):
                self.depth[i] = (np.nan, np.nan)
            else:
                self.depth[i] = coords
    

    def update_objects(self) -> None:
        objects_depth = LiDARVision.objects_segmentation(self.depth)
        objects_depth = filter(
            lambda o: not np.all(np.isnan(o)) and len(o) > 0,
            objects_depth,
        )

        self.objects = list(map(LiDARObject, objects_depth))
    

    def update(self) -> None:
        self.update_depth()
        self.update_objects()


    @staticmethod
    def objects_segmentation(depth: NDArray, thresh=1e-2) -> list[NDArray]:
        depth_safe = depth[:, 1].copy()
        depth_safe[np.isnan(depth_safe) | np.isinf(depth_safe)] = -1e2

        segments: list[NDArray] = []
        boundaries = np.abs(np.diff(depth_safe)) >= thresh

        previous = 0
        for i, boundary in enumerate(np.concatenate([boundaries, [True]])):
            if not boundary: continue
            segments.append(depth[previous:i+1])
            previous = i+1
        
        return segments


    @staticmethod
    def edges_detection(depth: NDArray, thresh=120e-6) -> list[NDArray]:
        depth_cleaned = depth[~np.isnan(depth[:, 1])]
        depth_y = depth_cleaned[:, 1]

        # Too few samples for edges detection.
        if len(depth_y) < 2: return np.zeros((0, 2, 2), dtype=float)

        filter_size = 5
        filter = np.ones(filter_size) / filter_size

        depth_filtered = np.convolve(depth_y, filter, 'valid')
        diff2 = np.abs(np.diff(depth_filtered, 2))

        edges: list[NDArray] = []
        boundaries = diff2 >= thresh

        # print(''.join(list(map(lambda x: '#' if x else '-', boundaries))), len(depth))
        # LiDARDebug.print_depth(diff2 * 1e4)

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
        self.bounding_box = np.max(depth, 0) - np.min(depth, 0)

        self.mean = np.average(depth, axis=0)
        self.edges = LiDARVision.edges_detection(depth)

        self.edges_count = len(self.edges)
        # print('Edges Samples:', list(map(len, self.edges)))
    

    def cube_info(self) -> tuple[NDArray, float]:
        edge = None
        length = -1

        for _edge in self.edges:
            if len(_edge) > length:
                edge, length = _edge, len(_edge)
        
        if edge.shape[0] < 6:
            return np.array([np.nan, np.nan]), np.nan
        
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

