from ardupilot_msgs.srv import ArmMotors
from rclpy.node import Node

from collider.src.steering.Stopper import sleep_and_check, STOP


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
            sleep_and_check(5)
            self.rc.set_drone_rc_neutral()
            sleep_and_check(0.5)
            self.mode_changer.call_mode("alt_hold")
            sleep_and_check(2)
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
