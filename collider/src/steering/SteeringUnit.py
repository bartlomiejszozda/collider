import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import collider.src.steering.Stopper as StopperModule
from collider.src.Helpers import Milliseconds, PixelDegrees, getDefaultProfile, Degree, d_Degree, d_d_Degree
from collider.src.ardupilot.PoseHistory import PoseHistory
from collider.src.ardupilot.RcOverride import RcOverride


class SteeringUnit(Node):
    def __init__(self, rc: RcOverride, pose_history: PoseHistory):
        super().__init__("SteeringUnit")
        self.rc = rc
        self._pose_history = pose_history
        self._start_approaching_angle = 10  # don't approach if target pitch to high. Iris barely manage to climb up when flying forward
        self._last_target_yaw = None
        self._last_target_yaw_change = 0
        qos_profile = getDefaultProfile()
        self.create_subscription(Float64MultiArray, '/collider/image_pos', self._image_pos_callback,
                                 qos_profile)

    def _image_pos_callback(self, msg: Float64MultiArray):
        if msg.data[0] == 0:
            print("Stopping app because tracking fails")
            StopperModule.should_stop = True
        if StopperModule.should_stop:
            print("omit steering because should_stop is set")
            return
        self._steer(Milliseconds(msg.data[0]), PixelDegrees(msg.data[1], msg.data[2]))

    def _steer(self, frame_timestamp: Milliseconds, target_angles: PixelDegrees):
        pose_when_image = self._pose_history.get_closest(frame_timestamp)
        if pose_when_image is None:
            print("WARNING omit steering because can't determine pose.")
            return
        current_pose = self._pose_history.get_current()
        # Down pitch is negative
        target_pitch = pose_when_image.pitch - target_angles.y_degree
        self._steer_throttle(target_pitch, 0)

        desired_pitch = self._dont_approach_if_target_pitch_to_high(
            target_pitch)  # pitch is also limited by steer_pitch
        self._steer_pitch(desired_pitch)

        # Right yaw is positive
        target_yaw = pose_when_image.yaw + target_angles.x_degree
        yaw_delta = target_yaw - current_pose.yaw
        self._steer_yaw(yaw_delta)

        if self._last_target_yaw is None:
            self._last_target_yaw = target_yaw
        target_yaw_change = target_yaw - self._last_target_yaw
        change_of_target_yaw_change = target_yaw_change - self._last_target_yaw_change
        self._steer_roll(target_yaw_change, change_of_target_yaw_change)

        self._last_target_yaw = target_yaw
        self._last_target_yaw_change = target_yaw_change

    def _dont_approach_if_target_pitch_to_high(self, target_pitch: Degree) -> Degree:
        desired_pitch = target_pitch - self._start_approaching_angle
        if desired_pitch > 0:
            desired_pitch = 0
        return desired_pitch

    def _steer_pitch(self, desired: Degree):
        pitch_rc_change = int(np.round(8 * desired))
        pitch_rc_change = self._value_limit(val=pitch_rc_change, limit=30)
        self.rc.set_rc("pitch", 1500 + int(pitch_rc_change))

    def _steer_throttle(self, target_pitch: Degree, desired_target_pitch: Degree):
        pitch_delta = target_pitch - desired_target_pitch
        throttle_rc_change = 200 * pitch_delta
        throttle_rc_change = self._value_limit(val=throttle_rc_change, limit=150)
        self.rc.set_rc("throttle", 1500 + int(throttle_rc_change))

    def _steer_yaw(self, yaw_delta: d_Degree):
        yaw_rc_change = np.round(20 * np.sign(yaw_delta) * yaw_delta * yaw_delta)
        yaw_rc_change = self._value_limit(val=yaw_rc_change, limit=80)
        self.rc.set_rc("yaw", 1500 + int(yaw_rc_change))

    def _steer_roll(self, target_yaw_change: d_Degree, change_of_target_yaw_change: d_d_Degree):
        roll_rc_change = np.round(250 * target_yaw_change) + np.round(150 * change_of_target_yaw_change)
        roll_rc_change = self._value_limit(val=roll_rc_change, limit=200)
        self.rc.set_rc("roll", 1500 + int(roll_rc_change))

    @staticmethod
    def _value_limit(val, limit):
        if abs(val) > limit:
            val = np.sign(val) * limit
        return val
