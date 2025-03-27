import threading
import time

import tf_transformations
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from collider.Types import AttitudeStamped, Milliseconds, EulerDegrees


class PoseHistory(Node):
    def __init__(self, ardupilot_time):
        super().__init__("PoseHistory")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,  # NOTE: UNKNOWN is not a valid QoS setting in ROS 2, using KEEP_LAST instead
            depth=10,  # Default depth, can be set to an appropriate value
            # durability=DurabilityPolicy.VOLATILE,
            # lifespan=rclpy.duration.Duration(seconds=0, nanoseconds=0),  # Infinite
            # deadline=rclpy.duration.Duration(seconds=0, nanoseconds=0),  # Infinite
            # liveliness=LivelinessPolicy.AUTOMATIC,
            # liveliness_lease_duration=rclpy.duration.Duration(seconds=0, nanoseconds=0)  # Infinite
        )
        self._pose_history = [AttitudeStamped(Milliseconds(0), EulerDegrees(0,0,0))]
        self.lock = threading.Lock()
        self._ardupilot_time = ardupilot_time
        self.create_subscription(PoseStamped, '/ap/pose/filtered', self._pose_callback, qos_profile)

    def _pose_callback(self, msg: PoseStamped):
        o = msg.pose.orientation
        stamp = msg.header.stamp.sec*1e3 + msg.header.stamp.nanosec/1e6
        yaw, pitch, roll= tf_transformations.euler_from_quaternion([o.w, o.x, o.y, o.z])
        degree = 180.0 / 3.14159
        roll, pitch, yaw = roll*degree, pitch*degree, yaw*degree
        with self.lock:
            assert stamp > self._pose_history[-1].stamp, "assertion failed, {stamp} > {self._pose_history[-1].stamp}"
            self._pose_history.append(AttitudeStamped(stamp, EulerDegrees(roll, pitch, yaw)))

    def get_closest(self, simulation_timestamp: Milliseconds):
        with self.lock:
            ardupilot_timestamp = self._ardupilot_time.ardupilot_time_from_sim_time(simulation_timestamp)
            margin_ms = 30
            assert ardupilot_timestamp >= self._pose_history[0].stamp - margin_ms, f"assertion failed, {ardupilot_timestamp} >= {self._pose_history[0].stamp - margin_ms}"
            assert ardupilot_timestamp <= self._pose_history[-1].stamp + margin_ms, f"assertion failed, {ardupilot_timestamp} <= {self._pose_history[-1].stamp + margin_ms}"
            time_distance = lambda id_val: abs(id_val[1].stamp - ardupilot_timestamp)
            id, closest = min(enumerate(self._pose_history), key=time_distance)
            self._pose_history = self._pose_history[id:]
        return closest.attitude

    def get_current(self):
        with self.lock:
            last = self._pose_history[-1].attitude
        return last
