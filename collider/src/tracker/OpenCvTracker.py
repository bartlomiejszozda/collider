import cv2
import numpy as np

from collider.src.Helpers import DenormalizedBbox
from collider.src.tracker.Tracker import Tracker


class OpenCvTracker(Tracker):
    def __init__(self):
        # "KCF": cv2.TrackerKCF_create,
        # "MOSSE": cv2.legacy.TrackerMOSSE_create,
        self.tracker_pair = ("CSRT", cv2.TrackerCSRT_create)
        self.tracker = self.tracker_pair[1]()
        self.started = False

    def track(self, frame: np.ndarray) -> DenormalizedBbox:
        if not self.started:
            bbox = cv2.selectROI("Select Object", frame, fromCenter=False, showCrosshair=True)
            cv2.destroyWindow("Select Object")
            self.tracker.init(frame, bbox)
            self.started = True
        success, bbox = self.tracker.update(frame)
        x, y, w, h = [int(v) for v in bbox]
        if success:
            return DenormalizedBbox(x=x, y=y, w=w, h=h, frame_w=frame.shape[1], frame_h=frame.shape[0])
        return None

    def name(self):
        return self.tracker_pair[0]
