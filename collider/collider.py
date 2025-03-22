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
import functools

should_stop = False

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


class ModeChanger(Node):
    def __init__(self):
        super().__init__("ModeChanger")

    def call_mode(self, mode):
        mode_map = {
                    "stabilize": 0,
                    "acro": 1,
                    "alt_hold": 2,
                    "guided": 4,
                    "rtl": 6,
                    "land": 9,
                    "brake": 17,
                    }
        mode_num = mode_map[mode]
        client = self.create_client(ModeSwitch, "/ap/mode_switch")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for mode_switch service ...")
        print(f"enable mode {mode}")
        request = ModeSwitch.Request(mode=mode_num)
        future = client.call_async(request)
        future.result()
#TODO add a proxy pattern to access ModeChanger and RCOverride


class Stopper(Node):
    def __init__(self, rc, mode_changer):
        super().__init__("Stopper")
        self.stop_service = self.create_service(Trigger, 'stop_node', self.stop_callback)
        self._stopping = False

        self.rc = rc
        self.mode_changer = mode_changer

    def stop_monitor(self):
        global should_stop
        while True:
            if should_stop:
                self.stop()
            time.sleep(1)

    def stop_callback(self, request, response):
        self.get_logger().info("Stop request received. Shutting down...")
        response.success = True
        response.message = "Node is shutting down."
        self.stop(signal_received, frame)
        return response

    def stop(self):
        global should_stop
        if not self._stopping:
            self._stopping = True

            self.get_logger().info("landing ...")
            print("landing")
            self.rc.set_drone_rc_neutral()
            time.sleep(5)
            self.rtl()
            time.sleep(27)
            self.mode_changer.call_mode("alt_hold")
            self.rc.set_rc("yaw", 1520)
            should_stop = False
            self._stopping = False

    def land(self):
        self.rc.set_drone_rc_neutral()
        self.rc.set_rc("throttle", 1000) # cannot enter land when throttle
        self.mode_changer.call_mode("land")
        time.sleep(5)

    def rtl(self):
        self.rc.set_drone_rc_neutral()
        self.mode_changer.call_mode("rtl")

    def brake(self):
        self.mode_changer.call_mode("brake")


def sleep_and_check(sleep_time):
    global should_stop
    period = 0.1
    limit = int(np.round(sleep_time / period))
    for count in range(limit):
        time.sleep(0.1)
        if should_stop:
            raise STOP


class STOP(Exception):
    pass


class Starter(Node):
    def __init__(self, rc, mode_changer):
        super().__init__("Starter")

        self.rc = rc
        self.mode_changer = mode_changer

    def takeoff(self):

        try:
            #START
            self.get_logger().info("starting a drone")
            self.rc.set_rc("throttle", 1000)
            self.mode_changer.call_mode("guided")
            sleep_and_check(1)

            self._call_arm_throttle()

            #ASCENDING
            self.mode_changer.call_mode("stabilize")
            sleep_and_check(0.1)
            self.rc.set_rc("throttle", 2000)
            sleep_and_check(10)
            self.rc.set_drone_rc_neutral()
            sleep_and_check(0.5)
            self.mode_changer.call_mode("alt_hold")
            sleep_and_check(0.5)
            #self.mode_changer._call_takeoff_5()
        except STOP:
            return

    def _call_arm_throttle(self):
        client = self.create_client(ArmMotors, "/ap/arm_motors")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for arm_motors service ...")
        request = ArmMotors.Request(arm=True)
        future = client.call_async(request)
        future.result()

    def acro_flip(self):
        #UPSIDE DOWN
        self.mode_changer.call_mode("acro")
        self.rc.set_drone_rc_neutral()
        self.rc.set_rc("roll", 2000)
        sleep_and_check(0.8)
        self.rc.set_drone_rc_neutral()
        sleep_and_check(2)

        #BACK TO NORMAL
        self.rc.set_rc("roll", 2000)
        sleep_and_check(0.8)

        self.mode_changer.call_mode("stabilize")
        self.rc.set_drone_rc_neutral()
        self.rc.set_rc("throttle", 2000)
        sleep_and_check(2)

    def _call_pre_arm_check(self):
        client = self.create_client(Trigger, "/ap/pre_arm_check")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for pre_arm_check service ...")
            request = Trigger.Request()
            future = client.call_async(request)

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
        yaw, pitch, roll= tf_transformations.euler_from_quaternion([o.w, o.x, o.y, o.z])
        degree = 180.0 / 3.14159
        roll, pitch, yaw = roll*degree, pitch*degree, yaw*degree
        with self.lock:
            assert stamp > self._pose_history[-1].stamp, "assertion failed, {stamp} > {self._pose_history[-1].stamp}"
            self._pose_history.append(AttitudeStamped(stamp, EulerDegrees(roll, pitch, yaw)))
        
    def get_closest(self, exact_stamp: Milliseconds):
        with self.lock:
            assert exact_stamp >= self._pose_history[0].stamp, f"assertion failed, {exact_stamp} >= {self._pose_history[0].stamp}"
            #assert exact_stamp <= self._pose_history[-1].stamp, f"assertion failed, {exact_stamp} <= {self._pose_history[-1].stamp}"
            if exact_stamp <= self._pose_history[-1].stamp: 
                print(f"WARNING: {exact_stamp} <= {self._pose_history[-1].stamp}")
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
        #self.last_pitch_rc = 1500
        self.last_throttles = []
        self._last_target_pitches = []
        #self.last_roll = 1500
        self.last_target_yaw = None
        self.last_target_yaw_change = 0
        self._last_target_pitch = 0
        self.rc = rc
        self._pose_history = pose_history
        self.prev_time = time.time()
        #self.prev_diff_y = 0.0
        self._attack_phase = False
        self._attack_angle = -15
        self._flying_angle = -5
        self._attack_angle_confirmations = 0
        self.subscription = None
        self._attack_phase = False
        self._last_callback_time = None
        self.last_throttle = 1500

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
        self.subscription = self.create_subscription(Float64MultiArray, '/collider/image_pos', self.image_pos_callback, qos_profile)

    def test_pitch_angle(self):
        pitch_rc = 1500
        for count in range(50):
            for sign in (-1, 1):
                pitch_rc = 1500 + sign*count*10
                self.rc.set_rc("pitch", int(pitch_rc))
                sleep_and_check(10)
                timestamp = int(time.time() * 1000)
                closest_pose = self._pose_history.get_closest(timestamp)
                print(f"pitch angle {closest_pose.pitch} for {pitch_rc}")


    def image_pos_callback(self, msg: Float64MultiArray):
        global should_stop
        if self._last_callback_time is not None and time.time() - self._last_callback_time > 2:
            print("self._last_callback_time is not None and time.time() - self._last_callback_time > 2")
            should_stop = True
            self._last_callback_time = None
        if msg.data[0] == 0:
            print("msg.data[0] == 0")
            should_stop = True
            self._last_callback_time = None
        if should_stop:
            print("should_stop")
            return
        self._last_callback_time = time.time()
        timestamp = Milliseconds(msg.data[0])
        target_on_image = PixelDegrees(msg.data[1], msg.data[2])
        self.steer(timestamp, target_on_image)

    #def _stop(self):
        #if self.subscribtion is not None:
            #self.destroy_subscription(self.subscription)

    def steer(self, timestamp: Milliseconds, target_on_image: PixelDegrees):
        pose_when_image = self._pose_history.get_closest(timestamp)
        current_pose = self._pose_history.get_current()
        print(f"Pose when image: {pose_when_image}")
        #print(f"Current pose   : {current_pose}")
        print(f"target on image: {target_on_image}")
        # Down pitch is negative
        target_pitch = pose_when_image.pitch - target_on_image.y_degree
        # Right yaw is positive
        target_yaw = pose_when_image.yaw + target_on_image.x_degree
        #print(f"target pitch: {target_pitch} yaw: {target_yaw}")
        
        goal_pitch = target_pitch - 10 #actual pitch is limited steer_pitch anyway

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
        #print(f"target yaw: {target_yaw}")
        target_yaw_change = target_yaw - self.last_target_yaw
        #print(f"target_yaw_change: {target_yaw_change}")
        change_of_target_yaw_change = target_yaw_change - self.last_target_yaw_change
        #print(f"change_of_target_yaw_change: {change_of_target_yaw_change}")
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
        #delta = desired - current
        #pitch = self.last_pitch_rc + int(np.round(delta))
        pitch_rc = 1500 + int(np.round(8*desired))
        max_change = 10
        current_rc = self.rc.get_rc("pitch")
        if pitch_rc > current_rc + max_change:
            pitch_rc = current_rc + max_change
        if pitch_rc < current_rc - max_change:
            pitch_rc = current_rc - max_change
        #pitch_rc = 1500 + int(np.round(8*desired)) + int(np.round(10*delta))
        if pitch_rc > 1530:
            pitch_rc = 1530
        if pitch_rc < 1470:
            pitch_rc = 1470
        self.rc.set_rc("pitch", pitch_rc)
        #self.last_pitch_rc = pitch_rc

    def _steer_throttle(self, target_pitch, desired_target_pitch):
        pitch_delta = target_pitch - desired_target_pitch
        target_pitch_change = target_pitch - self._last_target_pitch
        #throttle = self.last_throttle + 1*pitch_delta + 5*target_pitch_change
        throttle = 1500 + 200*pitch_delta
        print(f"throttle: {throttle}, pitch_delta: {pitch_delta}, target_pitch_change{target_pitch_change}")
        if throttle > 1650:
            throttle = 1650
        if throttle < 1350:
            throttle = 1350
        self.rc.set_rc("throttle", int(throttle))
        #self.last_throttle = throttle

    def _steer_throttle_ATTACK(self, target_pitch, desired_target_pitch):
        backward = 5
        assert len(self.last_throttles) <= backward
        assert len(self._last_target_pitches) <= backward
        #throttle = 1400 #ATTACK
        throttle = 1500
        if len(self.last_throttles) == backward and len(self._last_target_pitches) == backward:
            pitch_delta = target_pitch - desired_target_pitch
            last_throttle = (self.last_throttles[-5] + self.last_throttles[-4] + self.last_throttles[-3]) / 3
            last_target_pitch = (self._last_target_pitches[-5] + self._last_target_pitches[-4] + self._last_target_pitches[-3]) / 3
            balanced_target_pitch = (target_pitch + self._last_target_pitches[-1]) / 2
            target_pitch_change = balanced_target_pitch - last_target_pitch
            #throttle = last_throttle + 2*pitch_delta + 50*target_pitch_change # ATTACK
            throttle = last_throttle + 10*pitch_delta + 50*target_pitch_change
            print(f"steer_throttle. pitch_delta {pitch_delta}, target_pitch_change {target_pitch_change}, throttle {throttle}, last_throttle {last_throttle}")
            self.last_throttles.pop(0)
            self._last_target_pitches.pop(0)


        #throttle = optimal_attack_throttle + int(np.round(-50*pitch_delta)) + int(np.round(-50*target_pitch_change))
        if throttle > 1800:
            throttle = 1800
        if throttle < 1200:
            throttle = 1200
        self.rc.set_rc("throttle", int(throttle))
        self.last_throttles.append(int(throttle))
        self._last_target_pitches.append(target_pitch)

    def _steer_yaw(self, yaw_delta, target_yaw_change):
        # deadzone for yaw is 20, add 10 to make deadzone 10
        #smaller_deadzone = np.sign(yaw_delta)*10
        print(f"yaw_delta: {yaw_delta}, target_yaw_change: {target_yaw_change}")
        #yaw_rc_change = np.round(20*yaw_delta) + np.round(40*target_yaw_change)
        yaw_rc_change = np.round(20*np.sign(yaw_delta)*yaw_delta*yaw_delta)
        limit = 80
        if abs(yaw_rc_change) > limit:
            yaw_rc_change = np.sign(yaw_rc_change)*limit
        yaw_rc = 1500 + int(yaw_rc_change)
        self.rc.set_rc("yaw", yaw_rc)

    def _steer_roll(self, target_yaw_change, change_of_target_yaw_change):
        #self.target_yaw_deltas.append(target_yaw_change))
        #if len(self.target_yaw_deltas) < 2:
            #return
        #sum_deltas = sum(self.target_yaw_deltas)
        #self.target_yaw_deltas.pop(0)
        #roll = 1500 + int(np.round(80*sum_deltas))
        k=5
        roll_rc_change = np.round(k*50*target_yaw_change) + np.round(k*30*(change_of_target_yaw_change))
        if abs(roll_rc_change) > 2*100:
            roll_rc_change = np.sign(roll_rc_change)*2*100
        roll_rc = 1500 + int(roll_rc_change)
        max_change = 25
        current_rc = self.rc.get_rc("roll")
        if roll_rc > current_rc + max_change:
            roll_rc = current_rc + max_change
        if roll_rc < current_rc - max_change:
            roll_rc = current_rc - max_change
        self.rc.set_rc("roll", roll_rc)


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

class Exit(Exception):
    pass

#sigint_called_before = False
def handle_sigint(signal, frame, executor):
    global should_stop
    print("handle sigint")
    should_stop = True

    #global sigint_called_before
    #if sigint_called_before:
        #print("sigint called before, executor shutdown")
        #executor.shutdown()
    #sigint_called_before = True

def main(args=None):
    try:
        print("starting collider")
        rclpy.init(args=args)

        mavlink_connection = connect_mavlink()
        set_param(mavlink_connection, "ANGLE_MAX", 5000)
        set_param(mavlink_connection, "PILOT_SPEED_DN", 3000)
        set_param(mavlink_connection, "THR_DZ", 0)
        set_param(mavlink_connection, "RC1_DZ", 0)
        set_param(mavlink_connection, "RC2_DZ", 0)
        set_param(mavlink_connection, "RC3_DZ", 0)
        set_param(mavlink_connection, "RC4_DZ", 0)
        set_param(mavlink_connection, "PILOT_ACCEL_Z", 500)
        set_param(mavlink_connection, "ATC_ACCEL_P_MAX", 5000)
        #set_param(mavlink_connection, "ATC_ACCEL_R_MAX", 10000)
        rc = RcOverride(mavlink_connection)
        mode_changer = ModeChanger()
        #set_gimbal_position(rc, mavlink_connection)

        executor = MultiThreadedExecutor()
        stopper = Stopper(rc, mode_changer)
        global should_stop
        signal.signal(signal.SIGINT, functools.partial(handle_sigint, executor=executor))

        starter = Starter(rc, mode_changer)
        starter.takeoff()

        pose_history = PoseHistory()
        steering_unit = SteeringUnit(rc, pose_history)
        #sleep_and_check(5)
        #test_pitch = threading.Thread(target=steering_unit.test_pitch_angle, args=())
        #test_pitch.start()

        executor.add_node(mode_changer)
        executor.add_node(starter)
        executor.add_node(stopper)
        executor.add_node(pose_history)
        executor.add_node(steering_unit)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        stopper.stop_monitor()
    finally:
        print("finally")
        print("shutdown")
        executor.shutdown()
        mode_changer.destroy_node()
        starter.destroy_node()
        stopper.destroy_node()
        pose_history.destroy_node()
        steering_unit.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()

