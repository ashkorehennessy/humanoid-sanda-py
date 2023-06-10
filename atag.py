import apriltag
import cv2
import numpy as np

class Atag:
    def __init__(self):
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def get_id(self, results):
        result = results[0]
        return result.tag_id

    def get_center(self, results):
        result = results[0]
        return np.array(result.center).astype(int)
    
    def get_size(self, results):
        result = results[0]
        return int(((result.corners[0][1] - result.corners[1][1]) + (result.corners[3][1] - result.corners[2][1])) / 2)

