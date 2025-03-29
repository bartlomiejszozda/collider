import time

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import collider.src.steering.Stopper as StopperModule
from collider.src.Helpers import Milliseconds, PixelDegrees, getDefaultProfile


class SteeringUnit(Node):
    def __init__(self, rc, pose_history):
        super().__init__("SteeringUnit")
        self.last_throttles = []
        self._last_target_pitches = []
        self.last_target_yaw = None
        self.last_target_yaw_change = 0
        self._last_target_pitch = 0
        self.rc = rc
        self._pose_history = pose_history
        self.prev_time = time.time()
        self._attack_phase = False
        self._attack_angle = -15
        self._flying_angle = -5
        self._attack_angle_confirmations = 0
        self.subscription = None
        self._attack_phase = False
        self._last_callback_time = None
        self.last_throttle = 1500

        qos_profile = getDefaultProfile()
        self.subscription = self.create_subscription(Float64MultiArray, '/collider/image_pos', self.image_pos_callback,
                                                     qos_profile)

    def image_pos_callback(self, msg: Float64MultiArray):
        if self._last_callback_time is not None and time.time() - self._last_callback_time > 2:
            print("2 seconds elapsed from last image pos callback, stopping")
            StopperModule.should_stop = True
            self._last_callback_time = None
        if msg.data[0] == 0:
            print("msg.data[0] == 0")
            StopperModule.should_stop = True
            self._last_callback_time = None
        if StopperModule.should_stop:
            print("should_stop")
            return
        self._last_callback_time = time.time()
        timestamp = Milliseconds(msg.data[0])
        target_on_image = PixelDegrees(msg.data[1], msg.data[2])
        self.steer(timestamp, target_on_image)

    # def _stop(self):
    # if self.subscribtion is not None:
    # self.destroy_subscription(self.subscription)

    def steer(self, simulation_timestamp: Milliseconds, target_on_image: PixelDegrees):
        pose_when_image = self._pose_history.get_closest(simulation_timestamp)
        if pose_when_image is None:
            print("WARNING omit steering because cant determine pose.")
            return
        current_pose = self._pose_history.get_current()
        # Down pitch is negative
        target_pitch = pose_when_image.pitch - target_on_image.y_degree
        # Right yaw is positive
        target_yaw = pose_when_image.yaw + target_on_image.x_degree

        goal_pitch = target_pitch - 10  # actual pitch is limited by steer_pitch anyway

        """ATTACK
        if not self._attack_phase:
            self._monitor_attack_phase(target_pitch)
        else:
            #print("ATTACK phase activated")
            self._steer_throttle(target_pitch, self._attack_angle)
            goal_pitch = self._attack_angle
        """
        self._steer_throttle(target_pitch, 0)
        self._steer_pitch(current_pose.pitch, goal_pitch)

        if self.last_target_yaw is None:
            self.last_target_yaw = target_yaw
        target_yaw_change = target_yaw - self.last_target_yaw
        change_of_target_yaw_change = target_yaw_change - self.last_target_yaw_change
        yaw_delta = target_yaw - current_pose.yaw

        self._steer_yaw(yaw_delta, target_yaw_change)
        self._steer_roll(target_yaw_change, change_of_target_yaw_change)

        self.last_target_yaw = target_yaw
        self.last_target_yaw_change = target_yaw_change
        self._last_target_pitch = target_pitch

    def _monitor_attack_phase(self, target_pitch):
        if target_pitch < self._attack_angle + 0.5:
            self._attack_angle_confirmations += 1
        elif self._attack_angle_confirmations > 0:
            self._attack_angle_confirmations -= 1
        if self._attack_angle_confirmations > 3:
            self._attack_phase = True

    def _steer_pitch(self, current, desired):
        # delta = desired - current
        # pitch = self.last_pitch_rc + int(np.round(delta))
        pitch_rc = 1500 + int(np.round(8 * desired))
        max_change = 10
        current_rc = self.rc.get_rc("pitch")
        if pitch_rc > current_rc + max_change:
            pitch_rc = current_rc + max_change
        if pitch_rc < current_rc - max_change:
            pitch_rc = current_rc - max_change
        # pitch_rc = 1500 + int(np.round(8*desired)) + int(np.round(10*delta))
        if pitch_rc > 1530:
            pitch_rc = 1530
        if pitch_rc < 1470:
            pitch_rc = 1470
        self.rc.set_rc("pitch", pitch_rc)
        # self.last_pitch_rc = pitch_rc

    def _steer_throttle(self, target_pitch, desired_target_pitch):
        pitch_delta = target_pitch - desired_target_pitch
        target_pitch_change = target_pitch - self._last_target_pitch
        # throttle = self.last_throttle + 1*pitch_delta + 5*target_pitch_change
        throttle = 1500 + 200 * pitch_delta
        print(f"throttle: {throttle}, pitch_delta: {pitch_delta}, target_pitch_change{target_pitch_change}")
        if throttle > 1650:
            throttle = 1650
        if throttle < 1350:
            throttle = 1350
        self.rc.set_rc("throttle", int(throttle))
        # self.last_throttle = throttle

    def _steer_throttle_ATTACK(self, target_pitch, desired_target_pitch):
        backward = 5
        assert len(self.last_throttles) <= backward
        assert len(self._last_target_pitches) <= backward
        # throttle = 1400 #ATTACK
        throttle = 1500
        if len(self.last_throttles) == backward and len(self._last_target_pitches) == backward:
            pitch_delta = target_pitch - desired_target_pitch
            last_throttle = (self.last_throttles[-5] + self.last_throttles[-4] + self.last_throttles[-3]) / 3
            last_target_pitch = (self._last_target_pitches[-5] + self._last_target_pitches[-4] +
                                 self._last_target_pitches[-3]) / 3
            balanced_target_pitch = (target_pitch + self._last_target_pitches[-1]) / 2
            target_pitch_change = balanced_target_pitch - last_target_pitch
            # throttle = last_throttle + 2*pitch_delta + 50*target_pitch_change # ATTACK
            throttle = last_throttle + 10 * pitch_delta + 50 * target_pitch_change
            self.last_throttles.pop(0)
            self._last_target_pitches.pop(0)

        # throttle = optimal_attack_throttle + int(np.round(-50*pitch_delta)) + int(np.round(-50*target_pitch_change))
        if throttle > 1800:
            throttle = 1800
        if throttle < 1200:
            throttle = 1200
        self.rc.set_rc("throttle", int(throttle))
        self.last_throttles.append(int(throttle))
        self._last_target_pitches.append(target_pitch)

    def _steer_yaw(self, yaw_delta, target_yaw_change):
        # deadzone for yaw is 20, add 10 to make deadzone 10
        # smaller_deadzone = np.sign(yaw_delta)*10
        print(f"yaw_delta: {yaw_delta}, target_yaw_change: {target_yaw_change}")
        # yaw_rc_change = np.round(20*yaw_delta) + np.round(40*target_yaw_change)
        yaw_rc_change = np.round(20 * np.sign(yaw_delta) * yaw_delta * yaw_delta)
        limit = 80
        if abs(yaw_rc_change) > limit:
            yaw_rc_change = np.sign(yaw_rc_change) * limit
        yaw_rc = 1500 + int(yaw_rc_change)
        self.rc.set_rc("yaw", yaw_rc)

    def _steer_roll(self, target_yaw_change, change_of_target_yaw_change):
        # self.target_yaw_deltas.append(target_yaw_change))
        # if len(self.target_yaw_deltas) < 2:
        # return
        # sum_deltas = sum(self.target_yaw_deltas)
        # self.target_yaw_deltas.pop(0)
        # roll = 1500 + int(np.round(80*sum_deltas))
        k = 5
        roll_rc_change = np.round(k * 50 * target_yaw_change) + np.round(k * 30 * (change_of_target_yaw_change))
        if abs(roll_rc_change) > 2 * 100:
            roll_rc_change = np.sign(roll_rc_change) * 2 * 100
        roll_rc = 1500 + int(roll_rc_change)
        max_change = 25
        current_rc = self.rc.get_rc("roll")
        if roll_rc > current_rc + max_change:
            roll_rc = current_rc + max_change
        if roll_rc < current_rc - max_change:
            roll_rc = current_rc - max_change
        self.rc.set_rc("roll", roll_rc)
