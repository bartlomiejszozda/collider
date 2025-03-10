#!/usr/bin/env python3
from functools import partial
import time
import threading
from copy import deepcopy
import signal
import sys 
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import ArmMotors
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
# from sensor_msgs.msg import Imu

from mavros_msgs.srv import CommandTOL
from pymavlink import mavutil

import tf_transformations
from pyquaternion import Quaternion
import cv2
import numpy as np


class RcOverride:
    def __init__(self, mavlink_connection):
        self._rc_channels = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
        self._mapping = {
                "roll":0,
                "pitch":1,
                "throttle":2,
                "yaw":3,
                "gimbal_roll":5,
                "gimbal_pitch":6,
                "gimbal_yaw":7,
                }
        self._connection = mavlink_connection
        self._start_rc_overriding_thread()
        self.lock = threading.Lock()

    def set_drone_rc_neutral(self):
        neutral = 1500
        self.set_rc("roll", neutral)
        self.set_rc("pitch", neutral)
        self.set_rc("throttle", neutral)
        self.set_rc("yaw", neutral)

    def set_rc(self, name, val):
        with self.lock:
            try:
                channel = self._mapping[name]
                self._rc_channels[channel] = val
            except KeyError as e:
                print(f"Error while setting rc value: {e}")

    def get_rc(self, name):
        with self.lock:
            try:
                channel = self._mapping[name]
                return self._rc_channels[channel]
            except KeyError as e:
                print(f"Error while getting rc value: {e}")

    def _start_rc_overriding_thread(self):
        self._rc_thread = threading.Thread(target=self._send_rcs_infinitely)
        self._rc_thread.deamon = True
        self._rc_thread.start()

    def _send_rcs_infinitely(self):
        # You need to send RC channels all the time otherwise ardupilot will switch back to STABILIZE mode all the time
        prev_rc = None
        while True:
            if self._rc_channels != prev_rc:
                print(f"Sending RC override {self._rc_channels}")
            prev_rc = deepcopy(self._rc_channels)
            self._send_rc_override(self._rc_channels)
            time.sleep(0.1)

    def _send_rc_override(self, channels):
        self._connection.mav.rc_channels_override_send(
            self._connection.target_system,  # target_system
            self._connection.target_component,  # target_component
            channels[0],  # chan1_raw
            channels[1],  # chan2_raw
            channels[2],  # chan3_raw
            channels[3],  # chan4_raw
            channels[4],  # chan5_raw
            channels[5],  # chan6_raw
            channels[6],  # chan7_raw
            channels[7]   # chan8_raw
        )


class StartStop(Node):
    def __init__(self, mavlink_connection, rc):
        super().__init__("StartStop")
        self.stop_service = self.create_service(Trigger, 'stop_node', self.stop_callback)
        self._stopping = False

        self.connection = mavlink_connection
        self.rc = rc

    def start(self):

        #self._call_pre_arm_check()
        #time.sleep(10)

        #START
        self.get_logger().info("starting a drone")
        self.rc.set_rc("throttle", 1000)
        self._call_mode("guided")
        time.sleep(1)

        self._call_arm_throttle()

        #ASCENDING
        self._call_mode("stabilize")
        time.sleep(0.1)
        self.rc.set_rc("throttle", 2000)
        time.sleep(24)
        self.rc.set_drone_rc_neutral()
        time.sleep(0.1)
        self._call_mode("alt_hold")
        time.sleep(0.1)
        #self._call_takeoff_5()

    def _call_arm_throttle(self):
        client = self.create_client(ArmMotors, "/ap/arm_motors")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for arm_motors service ...")
        request = ArmMotors.Request(arm=True)
        future = client.call_async(request)
        future.result()

    def stop_callback(self, request, response):
        self.get_logger().info("Stop request received. Shutting down...")
        response.success = True
        response.message = "Node is shutting down."
        self.stop(signal_received, frame)
        return response

    def stop(self):
        if not self._stopping:
            self._stopping = True
            self.get_logger().info("landing ...")
            print("landing")
            self.rtl(7)
            self.land()
            rclpy.shutdown()

    def land(self):
        #TODO fix only landing can steer when it starts
        self.rc.set_drone_rc_neutral()
        self.rc.set_rc("throttle", 1000) # cannot enter land when throttle
        self._call_mode("land")
        time.sleep(5)

    def _call_mode(self, mode):
        mode_map = {
                    "stabilize": 0,
                    "acro": 1,
                    "alt_hold": 2,
                    "guided": 4,
                    "rtl": 6,
                    "land": 9,
                    }
        mode_num = mode_map[mode]
        client = self.create_client(ModeSwitch, "/ap/mode_switch")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for mode_switch service ...")
        print(f"enable mode {mode} {mode_num}")
        request = ModeSwitch.Request(mode=mode_num)
        future = client.call_async(request)
        future.result()

    def rtl(self, period=20):
        self._call_mode("rtl")
        self.rc.set_drone_rc_neutral()
        time.sleep(period)

    def acro_wait(self):
        self.rc.set_drone_rc_neutral()
        self._call_mode("acro")
        self._ready = True

    def acro_flip(self):
        #UPSIDE DOWN
        self._call_mode("acro")
        self.rc.set_drone_rc_neutral()
        self.rc.set_rc("roll", 2000)
        time.sleep(0.8)
        self.rc.set_drone_rc_neutral()
        time.sleep(2)

        #BACK TO NORMAL
        self.rc.set_rc("roll", 2000)
        time.sleep(0.8)

        self._call_mode("stabilize")
        self.rc.set_drone_rc_neutral()
        self.rc.set_rc("throttle", 2000)
        time.sleep(2)

    def _call_pre_arm_check(self):
        client = self.create_client(Trigger, "/ap/pre_arm_check")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for pre_arm_check service ...")
            request = Trigger.Request()
            future = client.call_async(request)

    """
    def _call_takeoff_5(self):
        altitude = 5
        # Send takeoff command
        print(f"Taking off to {altitude} meters")
        time.sleep(3)
        self.connection.mav.command_long_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
            0,  # confirmation
            0, 0, 0, 0, 0, 0,  # param1 ~ param6 (unused)
            altitude  # param7 (altitude in meters)
        )

        # Wait until the vehicle reaches a safe height before processing the next step
        while True:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
            print(f"Current altitude: {current_altitude:.2f} meters")
            if current_altitude >= altitude - 1:  # Check if within 1 meters of target altitude
                print(f"Reached target altitude: {altitude} meters")
                break
            time.sleep(0.1)
    """
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


class PoseHistory(Node):
    def __init__(self):
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
        self.create_subscription(PoseStamped, '/ap/pose/filtered', self.pose_callback, qos_profile)

    def pose_callback(self, msg: PoseStamped):
        o = msg.pose.orientation
        stamp = int(time.time() * 1000)#TODO PoseStamped time is ~700sec before time.time()
        #stamp = Milliseconds(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec/1000000)
        yaw, pitch, roll= tf_transformations.euler_from_quaternion([o.w, o.x, o.y, o.z])
        degree = 180.0 / 3.14159
        roll, pitch, yaw = roll*degree, pitch*degree, yaw*degree
        with self.lock:
            assert stamp > self._pose_history[-1].stamp, "assertion failed, {stamp} > {self._pose_history[-1].stamp}"
            self._pose_history.append(AttitudeStamped(stamp, EulerDegrees(roll, pitch, yaw)))
        
    def get_closest(self, exact_stamp: Milliseconds):
        with self.lock:
            assert exact_stamp >= self._pose_history[0].stamp, f"assertion failed, {exact_stamp} <= {self._pose_history[0].stamp}"
            #assert exact_stamp <= self._pose_history[-1].stamp, f"assertion failed, {exact_stamp} <= {self._pose_history[-1].stamp}"
            if exact_stamp <= self._pose_history[-1].stamp: 
                print("assertion failed, {exact_stamp} <= {self._pose_history[-1].stamp}")
            time_distance = lambda id_val: abs(id_val[1].stamp - exact_stamp)
            id, closest = min(enumerate(self._pose_history), key=time_distance)
            print(f"remove up to {id} id")
            self._pose_history = self._pose_history[id:]
        print(f"given pose for {closest.stamp} when requested for {exact_stamp}")
        return closest.attitude

    def get_current(self):
        with self.lock:
            last = self._pose_history[-1].attitude
        return last
    """
    def get_closest(self, exact_stamp: Milliseconds):
        assert(exact_stamp > self._pose_history[0].stamp)
        assert(exact_stamp < self._pose_history[-1].stamp)
        start_id = len(self._pose_history)/2
        start_candidate = self._pose_history[start_id]
        search_right_part = exact_stamp > start_candidate.stamp
        if search_right_part:
            poses_to_search = self._pose_history[start_id+1:]
        else:
            poses_to_search = reversed(self._pose_history[:start_id])
        best_candidate = self._get_closest_pose(poses_to_search, exact_stamp, start_candidate)
        return best_candidate.attitude
        

    def _get_closest_pose(candidates, exact_stamp, best_candidate):
        best_delta = best_candidate.stamp - exact_stamp
        for candidate in candidates:
            delta = candidate.stamp - exact_stamp
            if abs(delta) > abs(best_delta):
                return best_candidate
            best_delta = delta
            best_candidate = candidate
    """


class SteeringUnit(Node):
    def __init__(self, rc, pose_history):
        super().__init__("SteeringUnit")
        self.last_pitch_rc = 1500
        #self.last_throttle = 1500
        self.last_roll = 1500
        self.last_target_yaw = None
        self.last_target_yaw_delta = 0
        self.rc = rc
        self._pose_history = pose_history
        self.prev_time = time.time()
        #self.prev_diff_y = 0.0
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
        self._attack_phase = False
        self._attack_angle = -20
        self._flying_angle = -5
        self._attack_angle_confirmations = 0
        self.create_subscription(Float64MultiArray, '/collider/image_pos', self.image_pos_callback, qos_profile)


    def image_pos_callback(self, msg: Float64MultiArray):
        if msg.data[0] == 0:
            self.rc.set_drone_rc_neutral()
            self.rc.set_rc("pitch", 1510)
            return
        timestamp = Milliseconds(msg.data[0])
        target_on_image = PixelDegrees(msg.data[1], msg.data[2])
        self.steer(timestamp, target_on_image)

    def steer(self, timestamp: Milliseconds, target_on_image: PixelDegrees):
        pose_when_image = self._pose_history.get_closest(timestamp)
        current_pose = self._pose_history.get_current()
        print(f"Pose when image: {pose_when_image}")
        print(f"Current pose   : {current_pose}")
        print(f"target on image: {target_on_image}")
        # Down pitch is negative
        target_pitch = pose_when_image.pitch - target_on_image.y_degree
        # Right yaw is positive
        target_yaw = pose_when_image.yaw + target_on_image.x_degree
        print(f"target    pitch: {target_pitch} yaw: {target_yaw}")
        

        goal_pitch = self._flying_angle
        if self._monitor_attack_phase(target_pitch):
            print("ATTACK phase activated")
            goal_pitch = target_pitch
            self._steer_throttle(goal_pitch, self._attack_angle)
        self._steer_pitch(current_pose.pitch, goal_pitch)

        self._steer_yaw(current_pose.yaw, target_yaw)
        self._steer_roll(target_yaw)

    def _monitor_attack_phase(self, target_pitch):
        if target_pitch < self._attack_angle:
            self._attack_angle_confirmations += 1
        elif self._attack_angle_confirmations > 0:
            self._attack_angle_confirmations -= 1
        attack_phase = self._attack_angle_confirmations > 5
        return attack_phase

    def _steer_pitch(self, current, desired):
        #delta = desired - current
        #pitch = self.last_pitch_rc + int(np.round(delta))
        pitch_rc = 1500 + int(np.round(8*desired))
        max_change = 30
        current_rc = self.rc.get_rc("pitch")
        if pitch_rc > current_rc + max_change:
            pitch_rc = current_rc + max_change
        if pitch_rc < current_rc - max_change:
            pitch_rc = current_rc - max_change
        #pitch_rc = 1500 + int(np.round(8*desired)) + int(np.round(10*delta))
        if pitch_rc > 2000:
            pitch_rc = 2000
        if pitch_rc < 1000:
            pitch_rc = 1000
        self.rc.set_rc("pitch", pitch_rc)
        #self.last_pitch_rc = pitch_rc

    def _steer_throttle(self, goal_pitch, desired_pitch):
        delta = desired_pitch - goal_pitch
        #throttle = self.last_throttle + int(np.round(delta))
        optimal_attack_throttle = 1350
        throttle = optimal_attack_throttle + int(np.round(-20*delta))
        if throttle > 2000:
            throttle = 2000
        if throttle < 1000:
            throttle = 1000
        self.rc.set_rc("throttle", throttle)
        #self.last_throttle = throttle

    def _steer_yaw(self, current, desired):
        print(f"desired yaw: {desired}")
        delta = desired - current
        yaw_rc_change = int(np.round(20*delta))
        if abs(yaw_rc_change) > 100:
            yaw_rc_change = np.sign(yaw_rc_change)*100
        if abs(yaw_rc_change) < 21:
            yaw_rc_change = np.sign(yaw_rc_change)*21
        yaw_rc = 1500 + yaw_rc_change
        self.rc.set_rc("yaw", yaw_rc)

    def _steer_roll(self, target_yaw):
        if self.last_target_yaw is None:
            self.last_target_yaw = target_yaw
        target_yaw_delta = target_yaw - self.last_target_yaw
        print(f"target_yaw_delta: {target_yaw_delta}")
        delta_delta = target_yaw_delta - self.last_target_yaw_delta

        #self.target_yaw_deltas.append(target_yaw_delta)
        #if len(self.target_yaw_deltas) < 2:
            #return
        #sum_deltas = sum(self.target_yaw_deltas)
        #self.target_yaw_deltas.pop(0)
        #roll = 1500 + int(np.round(80*sum_deltas))
        roll_rc_change = int(np.round(50*target_yaw_delta)) + int(np.round(50*(delta_delta)))
        if abs(roll_rc_change) > 150:
            roll_rc_change = np.sign(roll_rc_change)*150
        roll = 1500 + roll_rc_change
        self.rc.set_rc("roll", roll)
        self.last_target_yaw = target_yaw
        self.last_target_yaw_delta = target_yaw_delta


def connect_mavlink():
    connection = mavutil.mavlink_connection('udp:localhost:14550')
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print("Heartbeat received")
    return connection

def set_gimbal_position(rc, connection):
    rc.set_rc("gimbal_pitch", 1700)
    rc.set_rc("gimbal_yaw", 1500)
    rc.set_rc("gimbal_roll", 1500)
    """
    connection.mav.command_long_send(
        self.connection.target_system,  # target_system
        self.connection.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,  # confirmation
        0, 0, 0, 0, 0, 0,  # param1 ~ param6 (unused)
        altitude  # param7 (altitude in meters)
    )
    """


def set_param(connection, param_name, param_val):
    connection.mav.param_set_send(
        connection.target_system, connection.target_component,
        param_name.encode('utf-8'),
        float(param_val),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    print(f"Set {param_name} to {param_val}")

def start(start_stop):
    start_stop.start()


def main(args=None):
    try:
        rclpy.init(args=args)

        mavlink_connection = connect_mavlink()
        set_param(mavlink_connection, "ANGLE_MAX", 5000)
        set_param(mavlink_connection, "PILOT_SPEED_DN", 3000)
        set_param(mavlink_connection, "THR_DZ", 0)
        set_param(mavlink_connection, "PILOT_ACCEL_Z", 500)
        set_param(mavlink_connection, "ATC_ACCEL_P_MAX", 5000)
        #set_param(mavlink_connection, "ATC_ACCEL_R_MAX", 10000)
        rc = RcOverride(mavlink_connection)
        set_gimbal_position(rc, mavlink_connection)
        start_stop = StartStop(mavlink_connection, rc)
        pose_history = PoseHistory()
        steering_unit = SteeringUnit(rc, pose_history)
        signal.signal(signal.SIGINT, lambda signal, frame: start_stop.stop())

        start_thread = threading.Thread(target=start, args=(start_stop,))
        start_thread.start()
        executor = MultiThreadedExecutor()
        executor.add_node(start_stop)
        executor.add_node(pose_history)
        executor.add_node(steering_unit)

        executor.spin()
    finally:
        start_stop.stop()
        executor.shutdown()

if __name__ == '__main__':
    main()

