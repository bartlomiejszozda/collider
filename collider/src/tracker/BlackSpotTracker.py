import cv2
import numpy as np

from collider.src.Helpers import DenormalizedBbox
from collider.src.tracker.Tracker import Tracker

class BlackSpotTracker(Tracker):
    def __init__(self):
        self._started = False
        self._prev_bbox = None

    def track(self, frame: np.ndarray) -> (bool, DenormalizedBbox):
        if not self._started:
            self._prev_bbox = cv2.selectROI("Select Object", frame, fromCenter=False, showCrosshair=True)
            cv2.destroyWindow("Select Object")
            self._started = True
        threshold_value = 30
        roi_x, roi_y, roi_w, roi_h = self._enlarge_prev_bbox_by_pixels(frame.shape[0], frame.shape[1], 100)
        roi = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        cv2.imshow("ROI", roi_gray)
        cv2.waitKey(1)
        _, roi_thresholded = cv2.threshold(roi_gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(roi_thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            print("BlackSpotFidner found no tracking candidate")
            return False, (0,0,0,0)
        bbox = self._get_closest_to_previous(contours, roi_x, roi_y)
        self._prev_bbox = bbox
        x, y, w, h = [int(v) for v in bbox]
        return True, DenormalizedBbox(x=x, y=y, w=w, h=h, frame_w=frame.shape[1], frame_h=frame.shape[0])

    def _enlarge_prev_bbox_by_pixels(self, frame_height, frame_width, pixels):
        assert isinstance(pixels, int)
        x = self._prev_bbox[0] - pixels
        y = self._prev_bbox[1] - pixels
        w = self._prev_bbox[2] + 2*pixels
        h = self._prev_bbox[3] + 2*pixels
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        if w > frame_width:
            w = frame_width
        if h > frame_height:
            h = frame_height
        return x, y, w, h

    def _get_closest_to_previous(self, contours, roi_x, roi_y):
        min_distance = float("inf")
        closest_bbox = None
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
        return "Black Spot Tracker"
