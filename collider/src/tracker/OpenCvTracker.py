import cv2


class OpenCvTracker:
    def __init__(self):
        # "KCF": cv2.TrackerKCF_create,
        # "MOSSE": cv2.legacy.TrackerMOSSE_create,
        self.tracker_pair = ("CSRT", cv2.TrackerCSRT_create)
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
