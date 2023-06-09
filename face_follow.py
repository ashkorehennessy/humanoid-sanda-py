# -*- coding: utf-8 -*-

import cv2
import time
import numpy as np
import uptech


cap = cv2.VideoCapture(0)
ret = cap.set(3, 640)  # 设置帧宽
ret = cap.set(4, 480)  # 设置帧高
font = cv2.FONT_HERSHEY_SIMPLEX  # 设置字体样式
kernel = np.ones((5, 5), np.uint8)  # 卷积
face_cascade = cv2.CascadeClassifier('/home/pi/uptench_star/cascades/haarcascade_frontalface_default.xml')

class FaceDetector():
    def __init__(self):
        self.up = uptech.UpTech()
        self.up.CDS_Open()
        self.up.CDS_SetMode(1,1)
        self.up.CDS_SetMode(2,1)
        self.up.CDS_SetMode(3,1)
        self.up.CDS_SetMode(4,1)
        self.up.CDS_SetMode(5,1)
        self.up.CDS_SetMode(6,1)

        self.speed = 240
        self.move_stop()
        # self.move_yaw()
        time.sleep(1)
        self.start_video()


    def api_init(self):
        print("process start")

    def move_yaw(self):
        print("yaw")
        self.up.CDS_SetSpeed(1,self.speed)
        self.up.CDS_SetSpeed(2,self.speed)
        self.up.CDS_SetSpeed(3,self.speed)
        self.up.CDS_SetSpeed(4,400)
        self.up.CDS_SetSpeed(4,400)

    def move_stop(self):
        self.up.CDS_SetSpeed(1,0)
        self.up.CDS_SetSpeed(2,0)
        self.up.CDS_SetSpeed(3,0)
        self.up.CDS_SetSpeed(4,0)

    def start_video(self):
        while cap.isOpened():
            ret, frame = cap.read()
            src = frame.copy()
            result = frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)

            for (x, y, w, h) in faces:
                result = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            if faces is not None:
                self.move_stop()
                if len(faces) == 1:
                    (fx, fy, fw, fh) = faces[0]
                    target_face_x = fx + fw / 2
                    offset_x = target_face_x - 640 / 2
                    offset_x = -offset_x

                    target_face_y = fy + fh / 2
                    offset_y = target_face_y - 480 / 2
                    offset_y = -offset_y

                    if offset_x < -20:

                        self.up.CDS_SetSpeed(5,-120)
                    if offset_x > 20:
                        self.up.CDS_SetSpeed(5, 120)
                elif len(faces) < 1:
                    self.move_yaw()
                    self.up.CDS_SetSpeed(5,0)
                else:
                    self.up.CDS_SetSpeed(5,0)
            else:
                self.move_yaw()
                self.up.CDS_SetSpeed(5,0)        

            cv2.imshow("result", result)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    face_detector = FaceDetector()
