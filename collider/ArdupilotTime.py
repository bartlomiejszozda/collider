from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock as ClockClass
from rclpy.node import Node

class ArdupilotTime(Node):
    def __init__(self):
        super().__init__("tracker")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,  # NOTE: UNKNOWN is not a valid QoS setting in ROS 2, using KEEP_LAST instead
            depth=10,  # Default depth, can be set to an appropriate value
        )
        self._difference_ms = None
        self.create_subscription(ClockClass, '/ap/clock', self._ardupilot_clock_callback, qos_profile)

    def _ardupilot_clock_callback(self, msg):
        sim_time_ms = self.get_clock().now().nanoseconds / 1e6
        ardupilot_time_ms = Time.from_msg(msg.clock).nanoseconds / 1e6
        self._difference_ms = ardupilot_time_ms - sim_time_ms

    def sim_time_from_ardupilot_time(self, ardupilot_time_ms):
        if self._difference_ms is None:
            return None
        return int(ardupilot_time_ms - self._difference_ms)

    def ardupilot_time_from_sim_time(self, sim_time_ms):
        if self._difference_ms is None:
            return None
        ardupilot_time_ms = sim_time_ms + self._difference_ms
        return int(ardupilot_time_ms)
