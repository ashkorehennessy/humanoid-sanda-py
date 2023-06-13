import cv2
import numpy as np
from atag import Atag

cap = cv2.VideoCapture('/dev/video0')
atag = Atag()

while True:
    ret, frame = cap.read()
    results = atag.detect(frame)
    if results is not None:
        #draw the bounding box
        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
        #show the frame
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF    
        
