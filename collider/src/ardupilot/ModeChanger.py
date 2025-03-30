from ardupilot_msgs.srv import ModeSwitch
from rclpy.node import Node

from collider.src.Helpers import log

class ModeChanger(Node):
    def __init__(self):
        super().__init__("ModeChanger")

    mode_map = {
        "stabilize": 0,
        "acro": 1,
        "alt_hold": 2,
        "guided": 4,
        "rtl": 6,
        "land": 9,
        "brake": 17,
    }

    def call_mode(self, mode: str):
        mode_num = self.mode_map[mode]
        client = self.create_client(ModeSwitch, "/ap/mode_switch")
        while not client.wait_for_service(timeout_sec=1.0):
            log.warn("Waiting for mode_switch service ... (may take a while)")
        log.info(f"enable mode {mode}")
        request = ModeSwitch.Request(mode=mode_num)
        future = client.call_async(request)
        future.result()
