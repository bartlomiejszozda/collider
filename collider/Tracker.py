#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
import time

#H, W = 960, 1280
H, W = 480, 640


class BlackSpotFinder:
    def __init__(self):
        self._started = False
        self._prev_bbox = None

    def track(self, frame):
        if not self._started:
            self._prev_bbox = cv2.selectROI("Select Object", frame, fromCenter=False, showCrosshair=True)
            cv2.destroyWindow("Select Object")
            self._started = True
        threshold_value = 30
        roi_x, roi_y, roi_w, roi_h = self._enlarge_by_pixels(self._prev_bbox, 100)
        roi = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        cv2.imshow("ROI", roi_gray)
        cv2.waitKey(1)
        _, roi_thresholded = cv2.threshold(roi_gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(roi_thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            print("BlackSpotFidner didn't find any tracking candidate")
            return False, (0,0,0,0)
        bbox = self._get_closest_to_previous(contours, roi_x, roi_y)
        self._prev_bbox = bbox
        return True, bbox

    def _enlarge_by_pixels(self, bbox, pixels):
        assert isinstance(pixels, int)
        x = self._prev_bbox[0] - pixels
        y = self._prev_bbox[1] - pixels
        w = self._prev_bbox[2] + 2*pixels
        h = self._prev_bbox[3] + 2*pixels
        global W, H
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        if w > W:
            w = W
        if h > H:
            h = H
        return x, y, w, h

    def _get_closest_to_previous(self, contours, roi_x, roi_y):
        min_distance = float("inf")
        for cnt in contours:
            x1_in_roi, y1_in_roi, w1, h1 = cv2.boundingRect(cnt)
            x1 = roi_x + x1_in_roi
            y1 = roi_y + y1_in_roi

            cx1, cy1 = x1 + w1 // 2, y1 + h1 // 2

            x0, y0, w0, h0 = self._prev_bbox
            cx0, cy0 = x0 + w0 // 2, y0 + h0 // 2
            distance = np.sqrt((cx1 - cx0) ** 2 + (cy1 - cy0) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_bbox = (x1, y1, w1, h1)
        return closest_bbox

    def name(self):
        return "Black Spot Finder"


class OpenCvTracker:
    def __init__(self):
        # "KCF": cv2.TrackerKCF_create,
        # "MOSSE": cv2.legacy.TrackerMOSSE_create,
        self.tracker_pair = ("CSRT", cv2.TrackerCSRT_create)

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
        self.tracker = self.tracker_pair[1]()
        self.started = False

    def track(self, frame):
        if not self.started:
            bbox = cv2.selectROI("Select Object", frame, fromCenter=False, showCrosshair=True)
            cv2.destroyWindow("Select Object")
            self.tracker.init(frame, bbox)
            self.started = True
        return self.tracker.update(frame)
    
    def name(self):
        return self.tracker_pair[0]

class FPS:
    def __init__(self):
        self._count = 0
        self._begin_time = 0
        self._period = 5

    def update(self):
        if time.time() - self._begin_time > self._period:
            elapsed = time.time() - self._begin_time
            #print(f"{(self._count/elapsed):.2f} FPS for last {self._period} seconds")
            self._begin_time = time.time()
            self._count = 0
            return
        self._count += 1

class Tracker(Node):
    def __init__(self):
        super().__init__("tracker")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,  # NOTE: UNKNOWN is not a valid QoS setting in ROS 2, using KEEP_LAST instead
            depth=10,  # Default depth, can be set to an appropriate value
        )
        self._failure = False
        self._tracker = BlackSpotFinder()
        self._fps = FPS()
        self.publisher = self.create_publisher(Float64MultiArray, '/collider/image_pos', 10)
        self.camera_topic = self.create_subscription(Image, 'static_camera_sensor/image', self.image_callback, qos_profile)

    def image_callback(self, msg: Image):
        if self._failure:
            return
        timestamp_ms = msg.header.stamp.sec * 1e3 + msg.header.stamp.nanosec / 1e6

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        tracking_frame = frame
        #tracking_frame = frame[(H//2-120):(H//2+120), (W//2-160):(W//2+160)]
        success, bbox = self._tracker.track(tracking_frame)
        if success:
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(tracking_frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
            global W, H
            target_from_center = x + w/2 - W/2, y + h/2 - H/2
            focal = 410.939
            degrees = lambda pixels: np.degrees(np.arctan(pixels/focal))
            x_angle = degrees(target_from_center[0])
            y_angle = degrees(target_from_center[1])
            
            self.send_target_angles(float(timestamp_ms), x_angle, y_angle)
            cv2.putText(tracking_frame, self._tracker.name(), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            self._fps.update()
        else:
            self._failure = True
            cv2.putText(tracking_frame, "Tracking Failure", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.send_target_angles(0.0, 0.0, 0.0)

        cv2.imshow("Tracking", tracking_frame)
        cv2.waitKey(1)

    def send_target_angles(self, timestamp, x_angle, y_angle):
        msg = Float64MultiArray()
        msg.data = [timestamp, x_angle, y_angle]
        self.publisher.publish(msg)
        #self.get_logger().info(f"Publishing: {msg.data}")
