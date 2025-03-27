import time
from dataclasses import dataclass

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

@dataclass
class PixelDegrees:
    x_degree: float
    y_degree: float

@dataclass
class EulerDegrees:
    roll: float
    pitch: float
    yaw: float

Milliseconds = int

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
            print(f"{(self._count/elapsed):.2f} FPS for last {self._period} seconds")
            self._begin_time = time.time()
            self._count = 0
            return
        self._count += 1
