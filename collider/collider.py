#!/usr/bin/env python3
from functools import partial

import rclpy
from rclpy.node import Node
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import ArmMotors
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
# from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandTOL
from pymavlink import mavutil
import time
import threading
from pyquaternion import Quaternion
from sensor_msgs.msg import Image
import cv2
import numpy as np

class Collider(Node):
    def __init__(self):
        super().__init__("collider")
        self.create_timer(1.0, self.timer_callback)
        self.counter_ = 0
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
        #self.twist_subscriber = self.create_subscription(TwistStamped, "/ap/twist/filtered", self.twist_callback, qos_profile)
        #self.pose_topic = self.create_subscription(PoseStamped, '/ap/pose/filtered', self.pose_callback, qos_profile)
        self.pose_topic = self.create_subscription(Image, 'camera/image', self.image_callback, qos_profile)
        self.connect_mavlink()

    def start_rc_overriding_thread(self):
        self._rc_channels = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
        self._rc_thread = threading.Thread(target=self._send_rcs_infinitely)
        self._rc_thread.deamon = True
        self._rc_thread.start()

    def connect_mavlink(self):
        # Connect to SITL on UDP port 14550
        self.connection = mavutil.mavlink_connection('udp:localhost:14550')

        # Wait for a heartbeat from the vehicle to ensure the connection is established
        print("Waiting for heartbeat...")
        self.connection.wait_heartbeat()
        print("Heartbeat received")

    def start(self):
        stabilize = 0
        acro = 1
        guided = 4
        rtl = 6
        land = 9

        #self._call_pre_arm_check()
        #time.sleep(10)

        #START
        self.get_logger().info("starting a drone")
        self.start_rc_overriding_thread()
        self._rc_channels = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
        self._call_mode(guided)

        self._call_arm_throttle()

        #ASCENDING
        self._call_mode(stabilize)
        self._rc_channels = [1500, 1500, 2000, 1500, 1500, 1500, 1500, 1500]
        time.sleep(14)
        self._rc_channels = [1500, 1500, 1500, 1550, 1500, 1500, 1500, 1500]
        #self._call_takeoff_5()

    def acro(self):
        #UPSIDE DOWN
        self._call_mode(acro)
        self._rc_channels = [2000, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        time.sleep(0.8)
        self._rc_channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        time.sleep(2)

        #BACK TO NORMAL
        self._rc_channels = [2000, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        time.sleep(0.8)

        self._call_mode(stabilize)
        self._rc_channels = [1500, 1500, 2000, 1500, 1500, 1500, 1500, 1500]
        time.sleep(2)

    def rtl_land():
        #BACK TO HOME
        self._call_mode(rtl)
        self._rc_channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        time.sleep(20)
        self._call_mode(stabilize)
        self._rc_channels = [1500, 1500, 1400, 1500, 1500, 1500, 1500, 1500]
        time.sleep(5)

        self._rc_channels = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
        self._call_mode(land)

    def _call_pre_arm_check(self):
        client = self.create_client(Trigger, "/ap/pre_arm_check")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for pre_arm_check service ...")
            request = Trigger.Request()
            future = client.call_async(request)

    def _call_arm_throttle(self):
        client = self.create_client(ArmMotors, "/ap/arm_motors")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for arm_motors service ...")
        request = ArmMotors.Request(arm=True)
        future = client.call_async(request)
        future.result()


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


    def _call_mode(self, mode):
        client = self.create_client(ModeSwitch, "/ap/mode_switch")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for mode_switch service ...")
        print(f"enable mode {mode}")
        request = ModeSwitch.Request(mode=mode)
        future = client.call_async(request)
        future.result()

    def _send_rcs_infinitely(self):
        #you need to send RC channels all the time otherwise ardupilot will switch back to STABILIZE mode all the time
        prev_rc = None
        while True:
            if self._rc_channels != prev_rc:
                print(f"Sending RC override {self._rc_channels}")
            prev_rc = self._rc_channels
            self._send_rc_override(self._rc_channels)
            time.sleep(0.1)
    """
    def _send_rcs(self):
        for i in range(30):
            print("throttle 2000")
            rc_channels = [1500, 1500, 2000, 1500, 1500, 1500, 1500, 1500]
            self._send_rc_override(rc_channels)
            # self._send_set_servo(3,2000)
            time.sleep(1)
            self._neutral_throttle()
        for i in range(100):
            print("throttle 2000. pitch 2000")
            rc_channels = [1500, 2000, 2000, 1500, 1500, 1500, 1500, 1500]
            self._send_rc_override(rc_channels)
            # self._send_set_servo(3,1600)
            # self._send_set_servo(2,2000)
            time.sleep(0.1)
        for i in range(30):
            print("throttle 1600. pitch 1500")
            rc_channels = [1500, 1500, 1600, 1500, 1500, 1500, 1500, 1500]
            self._send_rc_override(rc_channels)
            # self._send_set_servo(2,1500)
            time.sleep(1)
        """

    def _send_rc_override(self, channels):
        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            channels[0],  # chan1_raw
            channels[1],  # chan2_raw
            channels[2],  # chan3_raw
            channels[3],  # chan4_raw
            channels[4],  # chan5_raw
            channels[5],  # chan6_raw
            channels[6],  # chan7_raw
            channels[7]   # chan8_raw
        )

    def _send_set_servo(self, servo_num, val):
        print("Sending set servo")
        self.connection.mav.command_long_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,
            servo_num, val,
            0, 0, 0, 0, 0,  # param1 ~ param6 (unused)
        )



    """
    def twist_callback(self, msg: TwistStamped):
        self.get_logger().info(f'message is: {msg.twist.angular.x}, {msg.twist.angular.y}, {msg.twist.angular.z}')
        # self.get_logger().info(f'message time is: {msg.header.stamp.sec}, {msg.header.stamp.nanosec}')
    """
    """
    def pose_callback(self, msg: PoseStamped):
        #self.get_logger().info(f'message is: {msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z}, {msg.pose.orientation.w}')
        o = msg.pose.orientation
        q = Quaternion(o.w, o.x, o.y, o.z)
        my_vector = [1, 0, 0]
        rotated_vector = q.rotate(my_vector)
        print(rotated_vector)
    """
    def image_callback(self, msg: Image):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        cv2.imshow("Camera Image", image)
        cv2.waitKey(1)  # Required for OpenCV to update the window

    def timer_callback(self):
        self.get_logger().info(f"Hello {self.counter_} from collider")
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = Collider()
    node.start()
    rclpy.spin(node) #enables callbacks
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# def listen_messages():
#     while True:
#         message = connection.recv_match(blocking=True)
#         if not message:
#             continue
#         print(f"Received message: {message}")
