import copy
import logging
import time
from dataclasses import dataclass

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

log = logging.getLogger("collider")
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s', level="DEBUG")

FOCAL = 410.939

Degree = float
d_Degree = float  # delta of Degree
d_d_Degree = float  # delta of delta of Degree


@dataclass
class PixelDegrees:
    x_degree: Degree
    y_degree: Degree


@dataclass
class EulerDegrees:
    roll: Degree
    pitch: Degree  # Down pitch is negative
    yaw: Degree  # Right yaw is positive


Milliseconds = float
Seconds = float


@dataclass
class AttitudeStamped:
    stamp: Milliseconds
    attitude: EulerDegrees


def getDefaultProfile():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class FpsCalculator:
    def __init__(self):
        self._count = 0
        self._begin_time = 0
        self._period = 5

    def update(self):
        if time.time() - self._begin_time > self._period:
            elapsed = time.time() - self._begin_time
            log.info(f"{(self._count / elapsed):.2f} FPS for last {self._period} seconds")
            self._begin_time = time.time()
            self._count = -1
        self._count += 1


@dataclass
class DenormalizedBbox:
    x: int
    y: int
    w: int
    h: int
    frame_w: int
    frame_h: int

    def get_center(self) -> (int, int):
        return self.x + self.w // 2, self.y + self.h // 2

    def distance(self, other: 'DenormalizedBbox') -> float:
        cx0, cy0 = other.get_center()
        cx1, cy1 = self.get_center()
        return np.sqrt((cx1 - cx0) ** 2 + (cy1 - cy0) ** 2)

    def get_pixels_from_center(self) -> (int, int):
        x_from_center = self.x + self.w / 2 - self.frame_w / 2
        y_from_center = self.y + self.h / 2 - self.frame_h / 2
        return x_from_center, y_from_center

    def get_expanded_by(self, pixels: int) -> 'DenormalizedBbox':
        enlarged_bbox = copy.copy(self)
        enlarged_bbox.x -= pixels
        enlarged_bbox.y -= pixels
        enlarged_bbox.w += 2 * pixels
        enlarged_bbox.h += 2 * pixels
        enlarged_bbox._limit_to_frame_size()
        return enlarged_bbox

    def _limit_to_frame_size(self):
        if self.x < 0:
            self.x = 0
        if self.y < 0:
            self.y = 0
        if self.x + self.w > self.frame_w:
            self.w = self.frame_w - self.x
        if self.y + self.h > self.frame_h:
            self.h = self.frame_h - self.y


def continue_when_exception(do):
    try:
        do()
    except NameError as e:
        if "referenced before assignment" in str(e):
            return
        log.warning(f"{e}\nException handled when closing, continue closing")
    except Exception as e:
        log.warning(f"{e}\nException handled when closing, continue closing")
