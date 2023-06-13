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
        size_list=[]
        for result in results:
            line_list=[]
            (ptA, ptB, ptC, ptD) = result.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            line_list.append(np.linalg.norm(np.array(ptA)-np.array(ptB)))
            line_list.append(np.linalg.norm(np.array(ptB)-np.array(ptC)))
            line_list.append(np.linalg.norm(np.array(ptC)-np.array(ptD)))
            line_list.append(np.linalg.norm(np.array(ptD)-np.array(ptA)))
            size_list.append(max(line_list))
        return int(max(size_list))

