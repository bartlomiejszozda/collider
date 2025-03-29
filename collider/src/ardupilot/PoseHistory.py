import threading

import tf_transformations
from geometry_msgs.msg import PoseStamped, Quaternion

from rclpy.node import Node

from collider.src.Helpers import AttitudeStamped, Milliseconds, EulerDegrees, getDefaultProfile


class PoseHistory(Node):
    def __init__(self, ardupilot_time):
        super().__init__("PoseHistory")
        qos_profile = getDefaultProfile()
        self._pose_history = [AttitudeStamped(Milliseconds(0), EulerDegrees(0, 0, 0))]
        self.lock = threading.Lock()
        self._ardupilot_time = ardupilot_time
        self.create_subscription(PoseStamped, '/ap/pose/filtered', self._pose_callback, qos_profile)

    def _pose_callback(self, msg: PoseStamped):
        stamp = Milliseconds(msg.header.stamp.sec * 1e3 + msg.header.stamp.nanosec / 1e6)
        euler_degrees = self._get_attitude_in_degrees(msg.pose.orientation)
        with self.lock:
            assert stamp > self._pose_history[-1].stamp, f"assertion failed, {stamp} > {self._pose_history[-1].stamp}"
            self._pose_history.append(AttitudeStamped(stamp, euler_degrees))

    def get_closest(self, simulation_timestamp: Milliseconds) -> EulerDegrees:
        ardupilot_timestamp = self._ardupilot_time.ardupilot_time_from_sim_time(simulation_timestamp)
        margin = Milliseconds(30)
        with (self.lock):
            assert ardupilot_timestamp >= self._pose_history[
                0].stamp - margin, f"assertion failed, {ardupilot_timestamp} >= {self._pose_history[0].stamp - margin}"
            assert ardupilot_timestamp <= self._pose_history[
                -1].stamp + margin, f"assertion failed, {ardupilot_timestamp} <= {self._pose_history[-1].stamp + margin}"
            time_distance = lambda id_val: abs(id_val[1].stamp - ardupilot_timestamp)
            id, closest = min(enumerate(self._pose_history), key=time_distance)
            self._pose_history = self._pose_history[id:]
        return closest.attitude

    def get_current(self) -> EulerDegrees:
        with self.lock:
            last = self._pose_history[-1].attitude
        return last

    def _get_attitude_in_degrees(self, o: Quaternion) -> EulerDegrees:
        yaw, pitch, roll = tf_transformations.euler_from_quaternion([o.w, o.x, o.y, o.z])
        degree = 180.0 / 3.14159
        roll, pitch, yaw = roll * degree, pitch * degree, yaw * degree
        return EulerDegrees(roll, pitch, yaw)
