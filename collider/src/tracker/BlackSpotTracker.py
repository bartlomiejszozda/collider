import cv2
import numpy as np

from collider.src.Helpers import DenormalizedBbox
from collider.src.tracker.Tracker import Tracker

class BlackSpotTracker(Tracker):
    def __init__(self):
        self._started = False
        self._prev_bbox: DenormalizedBbox = None

    THRESHOLD = 30
    TRACKING_REGION_EXPANDED_BY = 100

    def track(self, frame: np.ndarray) -> DenormalizedBbox:
        if not self._started:
            self._start(frame)
        tracking_region = self._prev_bbox.get_expanded_by(self.TRACKING_REGION_EXPANDED_BY)
        candidates = self._find_tracking_candidates(tracking_region, frame)
        if len(candidates) == 0:
            print("BlackSpotFidner found no tracking candidate")
            return None
        new_bbox = min(candidates, key=lambda new: new.distance(self._prev_bbox))
        self._prev_bbox = new_bbox
        return new_bbox

    def _start(self, frame: np.ndarray):
        x, y, h, w = cv2.selectROI("Select Object", frame, fromCenter=False, showCrosshair=True)
        frame_height, frame_width = frame.shape[0], frame.shape[1]
        self._prev_bbox = DenormalizedBbox(x=x, y=y, w=w, h=h, frame_w=frame_width,
                         frame_h=frame_height)
        cv2.destroyWindow("Select Object")
        self._started = True

    def _find_tracking_candidates(self, tracking_region: DenormalizedBbox, frame: np.ndarray) -> list[DenormalizedBbox]:
        tracking_frame = frame[tracking_region.y: tracking_region.y + tracking_region.h,
                                tracking_region.x: tracking_region.x + tracking_region.w]
        frame_height, frame_width = frame.shape[0], frame.shape[1]
        tracking_frame_gray = cv2.cvtColor(tracking_frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow("ROI", tracking_frame_gray)
        cv2.waitKey(1)
        _, roi_thresholded = cv2.threshold(tracking_frame_gray, self.THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        countours, _ = cv2.findContours(roi_thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for cnt in countours:
            x, y, w, h = cv2.boundingRect(cnt)
            candidates.append(
                DenormalizedBbox(x=tracking_region.x + x, y=tracking_region.y + y, w=w, h=h, frame_w=frame_width,
                                 frame_h=frame_height))
        return candidates

    def name(self):
        return "Black Spot Tracker"
