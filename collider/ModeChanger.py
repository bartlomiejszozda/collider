from ardupilot_msgs.srv import ModeSwitch
from rclpy.node import Node


class ModeChanger(Node):
    #TODO add a proxy pattern to access ModeChanger and RCOverride
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
