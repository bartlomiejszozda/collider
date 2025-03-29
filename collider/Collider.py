#!/usr/bin/env python3
import threading

import rclpy
from pymavlink import mavutil
from rclpy.executors import MultiThreadedExecutor

from collider.src.ardupilot.ArdupilotClock import ArdupilotClock
from collider.src.ardupilot.ModeChanger import ModeChanger
from collider.src.ardupilot.PoseHistory import PoseHistory
from collider.src.ardupilot.RcOverride import RcOverride
from collider.src.steering.Starter import Starter
from collider.src.steering.SteeringUnit import SteeringUnit
from collider.src.steering.Stopper import Stopper
from collider.src.tracker.BlackSpotTracker import BlackSpotTracker
from collider.src.tracker.TrackerManager import TrackerManager


def connect_mavlink():
    connection = mavutil.mavlink_connection('udp:localhost:14550')
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print("Heartbeat received")
    return connection


def set_gimbal_position(rc):
    rc.set_rc("gimbal_pitch", 1700)
    rc.set_rc("gimbal_yaw", 1500)
    rc.set_rc("gimbal_roll", 1500)


def set_param(connection, param_name, param_val):
    connection.mav.param_set_send(
        connection.target_system, connection.target_component,
        param_name.encode('utf-8'),
        float(param_val),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    print(f"Set {param_name} to {param_val}")


def main(args=None):
    try:
        print("starting collider")
        rclpy.init(args=['--ros-args', '--param', 'use_sim_time:=true'])

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

        executor = MultiThreadedExecutor()

        rc = RcOverride(mavlink_connection)
        mode_changer = ModeChanger()
        stopper = Stopper(rc, mode_changer)

        starter = Starter(rc, mode_changer)
        starter.takeoff()

        ardupilot_clock = ArdupilotClock()
        pose_history = PoseHistory(ardupilot_clock)
        steering_unit = SteeringUnit(rc, pose_history)
        tracker = TrackerManager(BlackSpotTracker())

        executor.add_node(mode_changer)
        executor.add_node(starter)
        executor.add_node(stopper)
        executor.add_node(ardupilot_clock)
        executor.add_node(pose_history)
        executor.add_node(steering_unit)
        executor.add_node(tracker)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        stopper.stop_monitor()
    finally:
        print("Enters finally, shutdown")
        mode_changer.destroy_node()
        starter.destroy_node()
        stopper.destroy_node()
        pose_history.destroy_node()
        steering_unit.destroy_node()
        tracker.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    main()

# TODO BASICS
# do backflip at the start?
# add continue_if_exception
# change logs to ros logs? Or add logging system
# Tracker using should_stop signal
# add c++ node?
# Fill readme
# Fill readme with all changes I made after ardupilot instructions (add camera, ros bridge, )
# add unit tests

# TODO IN THE END
# check if _ before private methods
# auto code clean up
# check type hints

# TODO ADDITIONAL
# add a proxy pattern to access ModeChanger and RCOverride (do we really need it?)
# allow to kill app / RTL and restart properly
# log useful for analysis should be saved in excel

# TODO DONE
# clean up TrackerManager
# add parent class for Trackers?
# clean up BlackSpotTracker
# Milliseconds everywhere and as float
# typehints everywhere
# _ before private everywhere
# Clean Up Steering unit
# clean up Collider.py file
