#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np


class Tracker(Node):
    def __init__(self):
        super().__init__("tracker")
        self.create_timer(1.0, self.timer_callback)
        self.counter_ = 0
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,  # NOTE: UNKNOWN is not a valid QoS setting in ROS 2, using KEEP_LAST instead
            depth=10,  # Default depth, can be set to an appropriate value
        )
        self.pose_topic = self.create_subscription(Image, 'camera/image', self.image_callback, qos_profile)
        # "KCF": cv2.TrackerKCF_create,
        # "MOSSE": cv2.legacy.TrackerMOSSE_create,
        self.tracker_pair = ("CSRT", cv2.TrackerCSRT_create)
        self.tracker = cv2.TrackerCSRT_create()
        self.started = 0

    def image_callback(self, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if not self.started:
            bbox = cv2.selectROI("Select Object", frame, fromCenter=False, showCrosshair=True)
            self.tracker.init(frame, bbox)
        self.started = True

        success, bbox = self.tracker.update(frame)
        if success:
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
            cv2.putText(frame, self.tracker_pair[0], (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Tracking Failure", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Tracking", frame)
        cv2.waitKey(1)

    def timer_callback(self):
        self.get_logger().info(f"Hello {self.counter_} from collider")
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node) #enables callbacks
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


