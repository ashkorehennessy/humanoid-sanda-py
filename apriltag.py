import apriltag
import cv2
import numpy as np

class Detector:
    def __init__(self):
        # Create the AprilTag detector
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

    def detect(self, image_path):
        # Load the image
        image = cv2.imread(image_path)

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the image
        results = self.detector.detect(gray)

        # Process the results
        tag = []
        result = results[0]

        # Append tag id
        tag.append(result.tag_id)

        # Append tag center pos
        tag.append(np.array(result.center).astype(int))

        return tag
