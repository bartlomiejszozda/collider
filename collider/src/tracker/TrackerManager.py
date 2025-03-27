#!/usr/bin/env python3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np

from collider.src.Helpers import getDefaultProfile, FpsCalculator


class TrackerManager(Node):
    def __init__(self, tracker):
        super().__init__("tracker")
        qos_profile = getDefaultProfile()
        self._failure = False
        self._tracker = tracker
        self._fps = FpsCalculator()
        self.publisher = self.create_publisher(Float64MultiArray, '/collider/image_pos', 10)
        self.camera_topic = self.create_subscription(Image, 'static_camera_sensor/image', self.image_callback, qos_profile)

    def image_callback(self, msg: Image):
        if self._failure:
            return
        timestamp_ms = msg.header.stamp.sec * 1e3 + msg.header.stamp.nanosec / 1e6

        frame_height, frame_width = msg.height, msg.width
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        success, bbox = self._tracker.track(frame)
        if success:
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
            target_from_center = x + w/2 - frame_width/2, y + h/2 - frame_height/2
            focal = 410.939
            degrees = lambda pixels: np.degrees(np.arctan(pixels/focal))
            x_angle = degrees(target_from_center[0])
            y_angle = degrees(target_from_center[1])
            
            self.send_target_angles(float(timestamp_ms), x_angle, y_angle)
            cv2.putText(frame, self._tracker.name(), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            self._fps.update()
        else:
            self._failure = True
            cv2.putText(frame, "Tracking Failure", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.send_target_angles(0.0, 0.0, 0.0)

        cv2.imshow("Tracking", frame)
        cv2.waitKey(1)

    def send_target_angles(self, timestamp, x_angle, y_angle):
        msg = Float64MultiArray()
        msg.data = [timestamp, x_angle, y_angle]
        self.publisher.publish(msg)
        #self.get_logger().info(f"Publishing: {msg.data}")
