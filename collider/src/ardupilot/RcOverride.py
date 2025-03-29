import threading
import time
from copy import deepcopy


class RcOverride:
    def __init__(self, mavlink_connection):
        self._rc_channels = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
        self._mapping = {
            "roll": 0,
            "pitch": 1,
            "throttle": 2,
            "yaw": 3,
            "gimbal_roll": 5,
            "gimbal_pitch": 6,
            "gimbal_yaw": 7,
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

    def set_rc(self, name: str, val: int):
        with self.lock:
            try:
                channel = self._mapping[name]
                self._rc_channels[channel] = val
            except KeyError as e:
                print(f"Error while setting rc value: {e}")

    def get_rc(self, name: str):
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
        # Need to send RC channels all the time otherwise ardupilot will switch back to STABILIZE mode
        prev_rc = None
        while True:
            if self._rc_channels != prev_rc:
                print(f"Sending RC override {self._rc_channels}")
            prev_rc = deepcopy(self._rc_channels)
            self._send_rc_override(self._rc_channels)
            time.sleep(0.1)

    def _send_rc_override(self, channels: list[int]):
        self._connection.mav.rc_channels_override_send(
            self._connection.target_system,
            self._connection.target_component,
            channels[0],
            channels[1],
            channels[2],
            channels[3],
            channels[4],
            channels[5],
            channels[6],
            channels[7]
        )
