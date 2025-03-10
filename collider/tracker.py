#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
import time


class Tracker(Node):
    def __init__(self):
        super().__init__("tracker")
        self.counter_ = 0
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,  # NOTE: UNKNOWN is not a valid QoS setting in ROS 2, using KEEP_LAST instead
            depth=10,  # Default depth, can be set to an appropriate value
        )
        self.publisher = self.create_publisher(Float64MultiArray, '/collider/image_pos', 10)
        self.camera_topic = self.create_subscription(Image, 'static_camera_sensor/image', self.image_callback, qos_profile)
        # "KCF": cv2.TrackerKCF_create,
        # "MOSSE": cv2.legacy.TrackerMOSSE_create,
        self.tracker_pair = ("csrt", cv2.TrackerCSRT_create)

        """
        params = cv2.TrackerCSRT_Params()
        params.use_hog = True               # Enable HOG features
        params.use_color_names = True       # Enable color-based tracking
        params.use_gray = False             # Use grayscale tracking
        params.use_rgb = True               # Use RGB for tracking
        params.filter_lr = 0.02             # Learning rate (lower = more stable)
        params.gsl_sigma = 1.0              # Gaussian Sigma for motion estimation
        params.use_channel_weights = True   # Improve robustness
        params.use_segmentation = True      # Enable object segmentation
        #params.wrap_kernel = False          # Kernel wrapping for better robustness
        params.psr_threshold = 0.05         # Peak-to-Sidelobe Ratio threshold (lower = more sensitive)
        self.tracker = cv2.TrackerCSRT_create(params)
        """
        self.tracker = cv2.TrackerCSRT_create()
        self.started = 0
        self._failure = 0

    def image_callback(self, msg: Image):
        if self._failure > 3:
            return
        timestamp = int(time.time() * 1000)# Image stamp is time from start of the simulation
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if not self.started:
            bbox = cv2.selectROI("Select Object", frame, fromCenter=False, showCrosshair=True)
            self.tracker.init(frame, bbox)
        self.started = True

        #timestamp = msg.header.stamp.sec*1000 + msg.header.stamp.nanosec/1000000
        success, bbox = self.tracker.update(frame)
        if success:
            if self._failure > 0:
                self._failure -= 1
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
            H, W = 960, 1280
            target_from_center = x + w/2 - W/2, y + h/2 - H/2
            focal = 410.939
            degrees = lambda pixels: np.degrees(np.arctan(pixels/focal))
            x_angle = degrees(target_from_center[0])
            y_angle = degrees(target_from_center[1])
            
            self.send_target_angles(float(timestamp), x_angle, y_angle)
            cv2.putText(frame, self.tracker_pair[0], (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            self._failure += 1
            cv2.putText(frame, "Tracking Failure", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.send_target_angles(0.0, 0.0, 0.0)

        cv2.imshow("Tracking", frame)
        cv2.waitKey(1)

    def send_target_angles(self, timestamp, x_angle, y_angle):
        msg = Float64MultiArray()
        msg.data = [timestamp, x_angle, y_angle]
        self.publisher.publish(msg)
        #self.get_logger().info(f"Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node) #enables callbacks
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


