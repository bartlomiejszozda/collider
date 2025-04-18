#!/usr/bin/env python3
import threading

import rclpy
from pymavlink import mavutil
from rclpy.executors import MultiThreadedExecutor

from collider.src.ardupilot_related.ArdupilotClock import ArdupilotClock
from collider.src.ardupilot_related.ModeChanger import ModeChanger
from collider.src.ardupilot_related.PoseHistory import PoseHistory
from collider.src.ardupilot_related.RcOverrider import RcOverrider
from collider.src.steering.Starter import Starter
from collider.src.steering.SteeringUnit import SteeringUnit
from collider.src.steering.Stopper import Stopper
from collider.src.tracker.BlackSpotTracker import BlackSpotTracker
from collider.src.tracker.TrackerManager import TrackerManager
from collider.src.Helpers import log, continue_when_exception


def connect_mavlink():
    connection = mavutil.mavlink_connection('udp:localhost:14550')
    log.info("Waiting for heartbeat...")
    connection.wait_heartbeat()
    log.info("Heartbeat received")
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
    log.info(f"Set {param_name} to {param_val}")


def main(args=None):
    try:
        log.info("starting collider")
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

        rc = RcOverrider(mavlink_connection)
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
        log.info("Enters finally, shutdown")
        continue_when_exception(mode_changer.destroy_node)
        continue_when_exception(starter.destroy_node)
        continue_when_exception(stopper.destroy_node)
        continue_when_exception(pose_history.destroy_node)
        continue_when_exception(steering_unit.destroy_node)
        continue_when_exception(tracker.destroy_node)
        continue_when_exception(rclpy.shutdown)
        continue_when_exception(executor_thread.join)


if __name__ == '__main__':
    main()
