import cv2
import numpy as np



def edge_detect(gray):
    width = 640
    height = 200
    frame = gray[200: 400, :]
    ret, frame = cv2.threshold(frame, 135, 1, cv2.THRESH_BINARY)
    left_white_pixel = np.sum(frame[:, :width // 2])
    right_white_pixel = np.sum(frame[:, width // 2:])
    up_white_pixel = np.sum(frame[:height // 2, :])
    down_white_pixel = np.sum(frame[height // 2:, :])
    diff = int(left_white_pixel) - right_white_pixel
    # 拐角处特征
    if up_white_pixel < 28000:
        print("diff",diff,end="")
        print("拐角")
        return 1
    # 直向边缘处特征
    elif up_white_pixel < 35000:
        if diff > 0: 
            print("diff",diff,end="")
            print("直向边缘，左转")
            return 2
        else:
            print("diff",diff,end="")
            print("直向边缘，右转")
            return 3
    # 侧向边缘处特征
    elif up_white_pixel < 45000:
        if diff > 0: 
            print("diff",diff,end="")
            print("侧向边缘，左转")
            return 4
        else:
            print("diff",diff,end="")
            print("侧向边缘，右转")
            return 5
    else:
        return 0
