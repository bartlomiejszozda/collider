from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np

from collider.src.Helpers import getDefaultProfile, FpsCalculator, DenormalizedBbox, FOCAL, Milliseconds, PixelDegrees


class TrackerManager(Node):
    def __init__(self, tracker):
        super().__init__("tracker")
        qos_profile = getDefaultProfile()
        self._tracker = tracker
        self._fps = FpsCalculator()
        self._publisher = self.create_publisher(Float64MultiArray, '/collider/image_pos', 10)
        self.create_subscription(ImageMsg, 'static_camera_sensor/image', self._image_callback, qos_profile)

    def _image_callback(self, msg: ImageMsg):
        frame, timestamp, frame_height, frame_width = self._unpack_msg(msg)
        bbox = self._tracker.track(frame)
        if bbox is not None:
            self._draw_bbox(frame, bbox)
            angles = self._calc_target_angles(bbox)

            self._send_target_angles(timestamp, angles)
            self._fps.update()
        else:
            cv2.putText(frame, "Tracking Failure", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self._send_target_angles(Milliseconds(0), PixelDegrees(0.0, 0.0))

        cv2.imshow("Tracking", frame)
        cv2.waitKey(1)

    def _unpack_msg(self, msg: ImageMsg) -> (np.ndarray, Milliseconds, int, int):
        timestamp = msg.header.stamp.sec * 1e3 + msg.header.stamp.nanosec / 1e6
        frame_height, frame_width = msg.height, msg.width
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(frame_height, frame_width, -1)
        return frame, timestamp, frame_height, frame_width

    def _draw_bbox(self, frame, bbox: DenormalizedBbox):
        cv2.rectangle(frame, (bbox.x, bbox.y), (bbox.x + bbox.w, bbox.y + bbox.h), (0, 255, 0), 2, 1)
        cv2.putText(frame, self._tracker.name(), (bbox.x, bbox.y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def _calc_target_angles(self, bbox: DenormalizedBbox) -> PixelDegrees:
        target_from_center = bbox.get_pixels_from_center()
        degrees = lambda pixels: np.degrees(np.arctan(pixels / FOCAL))
        return PixelDegrees(x_degree = degrees(target_from_center[0]), y_degree = degrees(target_from_center[1]))

    def _send_target_angles(self, timestamp: Milliseconds, angles: PixelDegrees):
        msg = Float64MultiArray()
        msg.data = [float(timestamp), angles.x_degree, angles.y_degree]
        self._publisher.publish(msg)
