from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockClass
from rclpy.node import Node

from collider.src.Helpers import getDefaultProfile

class ArdupilotTime(Node):
    # Do own time synchronization, because I didn't manage to make ardupilot work on sim_time.
    def __init__(self):
        super().__init__("tracker")
        qos_profile = getDefaultProfile()
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
