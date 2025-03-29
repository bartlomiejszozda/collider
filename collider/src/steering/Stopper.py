import time

import numpy as np
from rclpy.node import Node
from std_srvs.srv import Trigger

should_stop = False


def handle_sigint(signal, frame, executor):
    global should_stop
    print("handle sigint")
    should_stop = True


class STOP(Exception):
    pass


class Exit(Exception):
    pass


def sleep_and_check(sleep_time):
    global should_stop
    period = 0.1
    limit = int(np.round(sleep_time / period))
    for count in range(limit):
        time.sleep(0.1)
        if should_stop:
            raise STOP


class Stopper(Node):
    def __init__(self, rc, mode_changer):
        super().__init__("Stopper")
        self.stop_service = self.create_service(Trigger, 'stop_node', self._stop_callback)
        self._stopping = False

        self.rc = rc
        self.mode_changer = mode_changer

    def _stop_callback(self, request, response):
        self.get_logger().info("Stop request received. Shutting down...")
        response.success = True
        response.message = "Node is shutting down."
        self._stop()
        return response

    def stop_monitor(self):
        global should_stop
        while True:
            if should_stop:
                self._stop()
                return
            time.sleep(1)

    def land(self):
        self.rc.set_drone_rc_neutral()
        self.rc.set_rc("throttle", 1000)# cannot enter land when throttle
        self.mode_changer.call_mode("land")
        time.sleep(5)

    def rtl(self):
        self.rc.set_drone_rc_neutral()
        self.mode_changer.call_mode("rtl")

    def brake(self):
        self.mode_changer.call_mode("brake")

    def _stop(self):
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
            should_stop = False
            self._stopping = False
