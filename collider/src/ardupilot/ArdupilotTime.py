from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockClass

from collider.src.Helpers import getDefaultProfile, Milliseconds


class ArdupilotTime(Node):
    # Do own time synchronization, because didn't manage to make ardupilot work on sim_time.
    def __init__(self):
        super().__init__("tracker")
        qos_profile = getDefaultProfile()
        self._difference = None
        self.create_subscription(ClockClass, '/ap/clock', self._ardupilot_clock_callback, qos_profile)

    def _ardupilot_clock_callback(self, msg: ClockClass):
        sim_time = Milliseconds(self.get_clock().now().nanoseconds / 1e6)
        ardupilot_time = Milliseconds(Time.from_msg(msg.clock).nanoseconds / 1e6)
        self._difference = ardupilot_time - sim_time

    def sim_time_from_ardupilot_time(self, ardupilot_time: Milliseconds) -> Milliseconds:
        if self._difference is None:
            return None
        return Milliseconds(ardupilot_time - self._difference)

    def ardupilot_time_from_sim_time(self, sim_time: Milliseconds) -> Milliseconds:
        if self._difference is None:
            return None
        ardupilot_time = sim_time + self._difference
        return Milliseconds(ardupilot_time)
