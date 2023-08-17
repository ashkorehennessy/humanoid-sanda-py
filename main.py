#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import subprocess
import multiprocessing
from pid import pid
from up_controller import UpController
import apriltag
tag_distance = 0
tag_counter = 3
attack_counter = 0
turning_counter = 0
front_hit_counter = 0

RWHEEL = 1
LWHEEL = 2
LFOOT = 3
LSHOULDER = 4
LELBOW = 5
LHAND = 6
RFOOT = 7
RSHOULDER = 8
RELBOW = 9
RHAND = 10

DISTANCE_FRONT = 5
DISTANCE_HEAD = 3
DISTANCE_RIGHT = 7
DISTANCE_LEFT = 8
DISTANCE_BACK = 6
DISTANCE_BOTTOM = 2
DISTANCE_LF = 4
DISTANCE_RF = 5

ANGLE = 4

up_controller = UpController()
pid = pid()
flag_stand = 0
video_width = 640
video_height = 480
print("head init finished")


class Atag:
    def __init__(self):
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

    def detect(self, gray):
        return self.detector.detect(gray)

    def get_distance(self, H, t):
        """
        :param H: homography matrix
        :param t: ???
        :return: distance
        """
        ss = 0.5
        src = np.array([[-ss, -ss, 0],
                        [ss, -ss, 0],
                        [ss, ss, 0],
                        [-ss, ss, 0]])
        Kmat = np.array([[700, 0, 0],
                         [0, 700, 0],
                         [0, 0, 1]]) * 1.0
        disCoeffs = np.zeros([4, 1]) * 1.0
        ipoints = np.array([[-1, -1],
                            [1, -1],
                            [1, 1],
                            [-1, 1]])
        for point in ipoints:
            x = point[0]
            y = point[1]
            z = H[2, 0] * x + H[2, 1] * y + H[2, 2]
            point[0] = (H[0, 0] * x + H[0, 1] * y + H[0, 2]) / z * 1.0
            point[1] = (H[1, 0] * x + H[1, 1] * y + H[1, 2]) / z * 1.0
        campoint = ipoints * 1.0
        opoints = np.array([[-1.0, -1.0, 0.0],
                            [1.0, -1.0, 0.0],
                            [1.0, 1.0, 0.0],
                            [-1.0, 1.0, 0.0]])
        opoints = opoints * 0.5
        rate, rvec, tvec = cv2.solvePnP(opoints, campoint, Kmat, disCoeffs)
        point, jac = cv2.projectPoints(src, np.zeros(rvec.shape), tvec, Kmat, disCoeffs)
        points = np.int32(np.reshape(point, [4, 2]))
        distance = np.abs(t / np.linalg.norm(points[0] - points[1]))
        return distance

def edge_detect(gray):
    width = 640
    height = 200
    frame = gray[150: 350, :]
    ret, frame = cv2.threshold(frame, 135, 1, cv2.THRESH_BINARY)
    left_white_pixel = np.sum(frame[:, :width // 2])
    right_white_pixel = np.sum(frame[:, width // 2:])
    up_white_pixel = np.sum(frame[:height // 2, :])
    down_white_pixel = np.sum(frame[height // 2:, :])
    diff = int(left_white_pixel) - right_white_pixel
    # 拐角处特征
    if up_white_pixel < 32000:
        print("diff" + str(diff) + "white:" + str(up_white_pixel) + "拐角")
        return 1
    # 直向边缘处特征
    elif up_white_pixel < 38000:
        if diff > 0: 
            print("diff:" + str(diff) + "white:" + str(up_white_pixel) + "直向边缘，左转")
            return 2
        else:
            print("diff:" + str(diff) + "white:" + str(up_white_pixel) + "直向边缘，右转")
            return 3
    # 侧向边缘处特征
    elif up_white_pixel < 49000:
        if diff > 0: 
            print("diff:" + str(diff) + "white:" + str(up_white_pixel) + "侧向边缘，左转")
            return 4
        else:
            print("diff:" + str(diff) + "white:" + str(up_white_pixel) + "侧向边缘，右转")
            return 5
    else:
        return 0


def robot_detect(image):
    # 转换颜色空间
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 设定颜色范围（在HSV颜色空间中）
    lower_orange = np.array([14, 65, 190])  # 橙色的最低HSV值
    upper_orange = np.array([35, 255, 255])  # 橙色的最高HSV值

    orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

    # 查找橙色连接件的轮廓
    contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 计算每个轮廓的外接矩形和中心点
    bounding_boxes = [cv2.boundingRect(contour) for contour in contours]
    center_points = [(box[0] + box[2] // 2, box[1] + box[3] // 2) for box in bounding_boxes]

    # 检查是否有轮廓
    if len(center_points) == 0:
        return None

    # 第一次筛选
    # 计算所有中心点的平均坐标
    avg_x = sum(point[0] for point in center_points) // len(center_points)
    avg_y = sum(point[1] for point in center_points) // len(center_points)

    # 筛选掉距离平均坐标较远的
    filtered_bounding_boxes = []
    filtered_center_points = []
    distance_threshold = 100

    for box, center in zip(bounding_boxes, center_points):
        distance = np.sqrt((center[0] - avg_x) ** 2 + (center[1] - avg_y) ** 2)
        if distance <= distance_threshold:
            filtered_bounding_boxes.append(box)
            filtered_center_points.append(center)

    if len(filtered_center_points) == 0:
        return None

    # 第二次筛选
    # 计算第一次筛选的中心点的平均坐标
    avg_x = sum(point[0] for point in filtered_center_points) // len(filtered_center_points)
    avg_y = sum(point[1] for point in filtered_center_points) // len(filtered_center_points)

    # 筛选掉距离平均坐标较远的小球
    filtered_bounding_boxes = []
    filtered_center_points = []

    distance_threshold = 150
    for box, center in zip(bounding_boxes, center_points):
        distance = np.sqrt((center[0] - avg_x) ** 2 + (center[1] - avg_y) ** 2)
        if distance <= distance_threshold:
            filtered_bounding_boxes.append(box)
            filtered_center_points.append(center)

    # 计算大矩形的位置
    if len(filtered_bounding_boxes) > 0:
        x_min = min(box[0] for box in filtered_bounding_boxes)
        y_min = min(box[1] for box in filtered_bounding_boxes)
        x_max = max(box[0] + box[2] for box in filtered_bounding_boxes)
        y_max = max(box[1] + box[3] for box in filtered_bounding_boxes)

        area = (x_max - x_min) * (y_max - y_min)

        print("area:", area, end="")

        if 3000 < area < 100000:
            return (x_min + x_max) // 2
        else:
            return None
    else:
        return None

def signal_handler(signal, frame):
    stop()
    exit(0)

def test():
    while True:
        time.sleep(1)
        up.CDS_SetAngle(3, 800, 512)
        time.sleep(1)
        up.CDS_SetAngle(3, 200, 512)

def reset_to_512():
    up.CDS_SetAngle(3, 512, 256)
    up.CDS_SetAngle(4, 512, 256)
    up.CDS_SetAngle(5, 512, 256)
    up.CDS_SetAngle(6, 512, 256)
    up.CDS_SetAngle(7, 512, 256)
    up.CDS_SetAngle(8, 512, 256)
    up.CDS_SetAngle(9, 512, 256)
    up.CDS_SetAngle(10, 512, 256)

def init():
    up.CDS_SetAngle(LFOOT, 552, 384)
    up.CDS_SetAngle(RFOOT, 552, 384)
    up.CDS_SetAngle(LSHOULDER, 512, 384)
    up.CDS_SetAngle(RSHOULDER, 512, 384)
    time.sleep(0.5)
    up.CDS_SetAngle(LHAND, 512, 384)
    up.CDS_SetAngle(RHAND, 512, 384)
    time.sleep(0.5)
    up.CDS_SetAngle(LELBOW, 512, 384)
    up.CDS_SetAngle(RELBOW, 512, 384)
    time.sleep(0.5)
    up.CDS_SetAngle(LSHOULDER, 818, 384)
    up.CDS_SetAngle(RSHOULDER, 206, 384)
    time.sleep(0.5)
    up.CDS_SetAngle(LELBOW, 206, 384)
    up.CDS_SetAngle(RELBOW, 206, 384)
    time.sleep(0.5)

def forward(ms=100):
    up.CDS_SetSpeed(RWHEEL, -330)
    up.CDS_SetSpeed(LWHEEL, 340)
    time.sleep(ms / 1000)

    
def slow(ms=100):
    up.CDS_SetSpeed(RWHEEL, -290)
    up.CDS_SetSpeed(LWHEEL, 300)
    time.sleep(ms / 1000)

def boost():
    up.CDS_SetSpeed(RWHEEL, -480)
    up.CDS_SetSpeed(LWHEEL, 520)
    time.sleep(0.3)

def boost_back():
    up.CDS_SetSpeed(RWHEEL, 620)
    up.CDS_SetSpeed(LWHEEL, -660)
    time.sleep(0.3)

def go_back():
    up.CDS_SetSpeed(RWHEEL, 460)
    up.CDS_SetSpeed(LWHEEL, -470)
    time.sleep(0.4)

def turn_left():
    up.CDS_SetSpeed(RWHEEL, -370)  # 亏电390，满电370
    up.CDS_SetSpeed(LWHEEL, -350)  # 亏电370，满电350
    time.sleep(0.4)

def turn_right():
    up.CDS_SetSpeed(RWHEEL, 350)  # 亏电370，满电350
    up.CDS_SetSpeed(LWHEEL, 370)  # 亏电390，满电370
    time.sleep(0.4)

def turn_left_back():
    up.CDS_SetSpeed(RWHEEL, -160)
    up.CDS_SetSpeed(LWHEEL, -390)
    time.sleep(0.4)

def turn_right_back():
    up.CDS_SetSpeed(RWHEEL, 390)
    up.CDS_SetSpeed(LWHEEL, 160)
    time.sleep(0.4)

def stop():
    up.CDS_SetSpeed(RWHEEL, 0)
    up.CDS_SetSpeed(LWHEEL, 0)

def back_1():
    up.CDS_SetAngle(LHAND, 858, 512)
    up.CDS_SetAngle(RHAND, 166, 512)
    up.CDS_SetAngle(LELBOW, 818, 768)
    up.CDS_SetAngle(RELBOW, 818, 768)
    up.CDS_SetAngle(LSHOULDER, 512, 768)
    up.CDS_SetAngle(RSHOULDER, 512, 768)

def back_2():
    up.CDS_SetAngle(LSHOULDER, 206, 768)
    up.CDS_SetAngle(RSHOULDER, 818, 768)

def back_3():
    up.CDS_SetAngle(LELBOW, 512, 768)
    up.CDS_SetAngle(RELBOW, 512, 768)
    up.CDS_SetAngle(LHAND, 730, 768)
    up.CDS_SetAngle(RHAND, 294, 768)

def back_4():
    up.CDS_SetAngle(LHAND, 512, 768)
    up.CDS_SetAngle(RHAND, 512, 768)
    up.CDS_SetAngle(LELBOW, 512, 768)
    up.CDS_SetAngle(RELBOW, 512, 768)
    up.CDS_SetAngle(LFOOT, 740, 768)
    up.CDS_SetAngle(RFOOT, 740, 768)
    time.sleep(0.4)
    up.CDS_SetAngle(LHAND, 730, 768)
    up.CDS_SetAngle(RHAND, 294, 768)
    up.CDS_SetAngle(LELBOW, 430, 768)
    up.CDS_SetAngle(RELBOW, 430, 768)

def back_5():
    up.CDS_SetAngle(LHAND, 512, 768)
    up.CDS_SetAngle(RHAND, 512, 768)
    up.CDS_SetAngle(LELBOW, 270, 768)
    up.CDS_SetAngle(RELBOW, 270, 768)
    up.CDS_SetAngle(LFOOT, 512, 768)
    up.CDS_SetAngle(RFOOT, 512, 768)
    time.sleep(0.4)
    up.CDS_SetAngle(LELBOW, 206, 768)
    up.CDS_SetAngle(RELBOW, 206, 768)
    time.sleep(0.4)
    up.CDS_SetAngle(LSHOULDER, 818, 768)
    up.CDS_SetAngle(RSHOULDER, 206, 768)

def front_1():
    up.CDS_SetAngle(LSHOULDER, 512, 768)
    up.CDS_SetAngle(RSHOULDER, 512, 768)
    time.sleep(0.3)
    up.CDS_SetAngle(LHAND, 858, 512)
    up.CDS_SetAngle(RHAND, 166, 512)
    up.CDS_SetAngle(LELBOW, 818, 768)
    up.CDS_SetAngle(RELBOW, 818, 768)

def front_2():
    up.CDS_SetAngle(LSHOULDER, 838, 768)
    up.CDS_SetAngle(RSHOULDER, 193, 768)

def front_3():
    up.CDS_SetAngle(LELBOW, 512, 768)
    up.CDS_SetAngle(RELBOW, 512, 768)
    up.CDS_SetAngle(LHAND, 730, 768)
    up.CDS_SetAngle(RHAND, 294, 768)

def front_4():
    up.CDS_SetAngle(LHAND, 512, 768)
    up.CDS_SetAngle(RHAND, 512, 768)
    up.CDS_SetAngle(LELBOW, 512, 768)
    up.CDS_SetAngle(RELBOW, 512, 768)
    up.CDS_SetAngle(LFOOT, 284, 768)
    up.CDS_SetAngle(RFOOT, 284, 768)
    time.sleep(0.4)
    up.CDS_SetAngle(LHAND, 730, 768)
    up.CDS_SetAngle(RHAND, 294, 768)
    up.CDS_SetAngle(LELBOW, 430, 768)
    up.CDS_SetAngle(RELBOW, 430, 768)

def front_5():
    up.CDS_SetAngle(LHAND, 512, 768)
    up.CDS_SetAngle(RHAND, 512, 768)
    up.CDS_SetAngle(LELBOW, 270, 768)
    up.CDS_SetAngle(RELBOW, 270, 768)
    up.CDS_SetAngle(LFOOT, 532, 768)
    up.CDS_SetAngle(RFOOT, 532, 768)
    time.sleep(0.4)
    up.CDS_SetAngle(LELBOW, 206, 768)
    up.CDS_SetAngle(RELBOW, 206, 768)
    

def hit_2L():
    # 前方攻击，左手先
    # 前倾
    up.CDS_SetAngle(LFOOT, 520, 512)
    up.CDS_SetAngle(RFOOT, 520, 512)

    # 左手
    up.CDS_SetAngle(LSHOULDER, 880, 712)
    # 伸肘
    up.CDS_SetAngle(LELBOW, 425, 968)
    time.sleep(0.05)
    # 伸手
    up.CDS_SetAngle(LHAND, 592, 968)
    time.sleep(0.1)
    # 转臂
    up.CDS_SetAngle(LSHOULDER, 920, 712)
    time.sleep(0.3)
    # 复位
    alert()
    time.sleep(0.5)

    
def hit_2R():
    # 前方攻击，右手
    # 前倾
    up.CDS_SetAngle(LFOOT, 520, 512)
    up.CDS_SetAngle(RFOOT, 520, 512)

    # 右手
    up.CDS_SetAngle(RSHOULDER, 144, 712)
    # 伸肘
    up.CDS_SetAngle(RELBOW, 425, 968)
    time.sleep(0.05)
    # 伸手
    up.CDS_SetAngle(RHAND, 432, 968)
    time.sleep(0.1)
    # 转臂
    up.CDS_SetAngle(RSHOULDER, 104, 712)
    time.sleep(0.3)
    # 复位
    alert()
    time.sleep(0.5)

def hit_3_L():
    # 左侧旋转攻击
    # 前倾
    up.CDS_SetAngle(LFOOT, 540, 512)
    up.CDS_SetAngle(RFOOT, 540, 512)
    # 伸肘
    up.CDS_SetAngle(LELBOW, 425, 968)
    time.sleep(0.05)
    # 伸手
    up.CDS_SetAngle(LHAND, 622, 968)
    # 摆臂
    up.CDS_SetAngle(LSHOULDER, 328, 912)
    time.sleep(0.5)
    # 复位
    up.CDS_SetAngle(LELBOW, 76, 512)
    up.CDS_SetAngle(LHAND, 958, 512)
    time.sleep(0.4)
    alert()

def hit_3_LF():
    # 左前方攻击
    # 前倾
    up.CDS_SetAngle(LFOOT, 525, 512)
    up.CDS_SetAngle(RFOOT, 525, 512)
    up.CDS_SetAngle(LSHOULDER, 736, 912)
    time.sleep(0.1)
    # 伸肘
    up.CDS_SetAngle(LELBOW, 425, 968)
    time.sleep(0.05)
    # 伸手
    up.CDS_SetAngle(LHAND, 622, 968)
    time.sleep(0.4)
    # 复位
    up.CDS_SetAngle(LELBOW, 76, 512)
    up.CDS_SetAngle(LHAND, 958, 512)
    time.sleep(0.4)
    alert()

def hit_3_LF_S():
    # 左前方攻击，向中间扫
    # 前倾
    up.CDS_SetAngle(LFOOT, 530, 512)
    up.CDS_SetAngle(RFOOT, 530, 512)
    up.CDS_SetAngle(LSHOULDER, 676, 912)
    time.sleep(0.1)
    # 伸肘
    up.CDS_SetAngle(LELBOW, 425, 968)
    time.sleep(0.05)
    # 伸手
    up.CDS_SetAngle(LHAND, 622, 968)
    time.sleep(0.2)
    # 摆臂
    up.CDS_SetAngle(LSHOULDER, 900, 912)
    time.sleep(0.4)
    # 复位
    up.CDS_SetAngle(LELBOW, 76, 512)
    up.CDS_SetAngle(LHAND, 958, 512)
    time.sleep(0.4)
    alert()


def hit_3_R():
    # 右侧旋转攻击
    # 前倾
    up.CDS_SetAngle(LFOOT, 540, 512)
    up.CDS_SetAngle(RFOOT, 540, 512)
    # 伸肘
    up.CDS_SetAngle(RELBOW, 425, 968)
    time.sleep(0.05)
    # 伸手
    up.CDS_SetAngle(RHAND, 382, 968)
    # 摆臂
    up.CDS_SetAngle(RSHOULDER, 696, 912)
    time.sleep(0.5)
    # 复位
    up.CDS_SetAngle(RELBOW, 76, 512)
    up.CDS_SetAngle(RHAND, 66, 512)
    time.sleep(0.4)
    alert()

def hit_3_RF():
    # 右前方攻击
    # 前倾
    up.CDS_SetAngle(LFOOT, 525, 512)
    up.CDS_SetAngle(RFOOT, 525, 512)
    up.CDS_SetAngle(RSHOULDER, 288, 912)
    time.sleep(0.1)
    # 伸肘
    up.CDS_SetAngle(RELBOW, 425, 968)
    time.sleep(0.05)
    # 伸手
    up.CDS_SetAngle(RHAND, 382, 968)
    time.sleep(0.4)
    # 复位
    up.CDS_SetAngle(RELBOW, 76, 512)
    up.CDS_SetAngle(RHAND, 66, 512)
    time.sleep(0.4)
    alert()

def hit_3_RF_S():
    # 右前方攻击，向中间扫
    # 前倾
    up.CDS_SetAngle(LFOOT, 530, 512)
    up.CDS_SetAngle(RFOOT, 530, 512)
    up.CDS_SetAngle(RSHOULDER, 348, 912)
    time.sleep(0.1)
    # 伸肘
    up.CDS_SetAngle(RELBOW, 425, 968)
    time.sleep(0.05)
    # 伸手
    up.CDS_SetAngle(RHAND, 382, 968)
    time.sleep(0.2)
    # 摆臂
    up.CDS_SetAngle(RSHOULDER, 124, 912)
    time.sleep(0.4)
    # 复位
    up.CDS_SetAngle(RELBOW, 76, 512)
    up.CDS_SetAngle(RHAND, 66, 512)
    time.sleep(0.4)
    alert()

def alert():
    up.CDS_SetAngle(LELBOW, 76, 512)
    up.CDS_SetAngle(RELBOW, 76, 512)
    up.CDS_SetAngle(LHAND, 958, 512)
    up.CDS_SetAngle(RHAND, 66, 512)
    up.CDS_SetAngle(LSHOULDER, 838, 712)
    up.CDS_SetAngle(RSHOULDER, 186, 712)
    up.CDS_SetAngle(LFOOT, 535, 512)
    up.CDS_SetAngle(RFOOT, 535, 512)

def release():
    time.sleep(0.2)
    up.CDS_SetAngle(LSHOULDER, 808, 512)
    up.CDS_SetAngle(RSHOULDER, 216, 512)
    up.CDS_SetAngle(LELBOW, 206, 256)
    up.CDS_SetAngle(RELBOW, 206, 256)
    up.CDS_SetAngle(LHAND, 512, 512)
    up.CDS_SetAngle(RHAND, 512, 512)

def autopilot(flag_stop_autopilot,datas,flag_attack,stand_event,start_event,flag_tracking,flag_turning,image,tracking_counter):
    global flag_stand
    global turning_counter
    while True:
        auto_pilot_index = 0
        iodata = datas[0]
        adcdata = datas[1]
        if len(iodata) != 8:
            iodata = [0,0,0,0,1,1,0,0]
            print("reset iodata")
        if len(adcdata) != 10:
            adcdata = [0,0,0,0,2000,0,0,0,0,0]
            print("reset adcdata")

        if flag_stop_autopilot.value == 1:
            time.sleep(0.1)
            continue

        if flag_stand == 0:
            if adcdata[ANGLE] < 700:
                flag_stand = 1
                stand_event.set()
                time.sleep(0.1)
            elif adcdata[ANGLE] > 2900:
                flag_stand = 4
                stand_event.set()
                time.sleep(0.1)
        else:
            # front stand
            if flag_stand == 1:
                if adcdata[ANGLE] > 600:
                    stand_event.clear()
                    flag_stand = 0
                    continue
                stop()
                front_1()
                time.sleep(0.8)
                if adcdata[DISTANCE_BOTTOM] > 250 or adcdata[DISTANCE_LEFT] > 500 or adcdata[DISTANCE_RIGHT] > 500:
                    flag_stand = 3
                else:
                    flag_stand = 2
                front_2()
                time.sleep(0.4)
                front_3()
                time.sleep(0.4)
                front_4()
            elif flag_stand == 2:
                flag_stand = 0
                time.sleep(0.5)
                front_5()
                time.sleep(0.6)
                stand_event.clear()
            elif flag_stand == 3:
                flag_stand = 0
                time.sleep(0.2)
                up.CDS_SetSpeed(RWHEEL, 480)
                up.CDS_SetSpeed(LWHEEL, -500)
                time.sleep(0.5)
                up.CDS_SetSpeed(RWHEEL, 0)
                up.CDS_SetSpeed(LWHEEL, 0)
                time.sleep(0.5)
                front_5()
                time.sleep(0.6)
                stand_event.clear()
            # back stand
            elif flag_stand == 4:
                if adcdata[ANGLE] < 3300:
                    stand_event.clear()
                    flag_stand = 0
                    continue
                stop()
                back_1()
                time.sleep(0.8)
                if adcdata[DISTANCE_BOTTOM] > 250 or adcdata[DISTANCE_LEFT] > 500 or adcdata[DISTANCE_RIGHT] > 500:
                    flag_stand = 6
                else:
                    flag_stand = 5
                back_2()
                time.sleep(0.4)
                back_3()
                time.sleep(0.4)
                back_4()
            elif flag_stand == 5:
                flag_stand = 0
                time.sleep(0.5)
                back_5()
                time.sleep(0.6)
                stand_event.clear()
            elif flag_stand == 6:
                flag_stand = 0
                time.sleep(0.2)
                up.CDS_SetSpeed(RWHEEL, -480)
                up.CDS_SetSpeed(LWHEEL, 500)
                time.sleep(0.5)
                up.CDS_SetSpeed(RWHEEL, 0)
                up.CDS_SetSpeed(LWHEEL, 0)
                time.sleep(0.5)
                back_5()
                time.sleep(0.6)
                stand_event.clear()
            continue

        if flag_attack.value == 0:
            alert()

        if adcdata[DISTANCE_BACK] > 200:
            turn_right()
            turn_right()
            turn_right()

        if iodata[0] == 1:
            auto_pilot_index += 1
        if iodata[1] == 1:
            auto_pilot_index += 4

        if auto_pilot_index != 0:
            print(f"index:{auto_pilot_index} iodata:{iodata} adcdata{adcdata}")
        if auto_pilot_index == 1:
            flag_turning.value = 1
            turning_counter = 3
            if up.CDS_GetCurPos(LFOOT) > 515:
                turn_right_back()
                turn_right()
                turn_right()
                slow(500)
                forward(500)
            else:
                stop()
            continue
        elif auto_pilot_index == 4:
            flag_turning.value = 1
            turning_counter = 3
            if up.CDS_GetCurPos(LFOOT) > 515:
                turn_left_back()
                turn_left()
                turn_left()
                forward(500)
            else:
                stop()
            continue
        elif auto_pilot_index == 5:
            flag_turning.value = 1
            turning_counter = 3
            if up.CDS_GetCurPos(LFOOT) > 515:
                turn_left_back()
                turn_left()
                turn_left()
                turn_left()
                forward(500)
            else:
                stop()
            continue
        else:
            if flag_tracking.value == 0:
                if up.CDS_GetCurPos(LFOOT) < 515:
                    up.CDS_SetSpeed(RWHEEL, -290)
                    up.CDS_SetSpeed(LWHEEL, 300)
                else:
                    up.CDS_SetSpeed(RWHEEL, -330)
                    up.CDS_SetSpeed(LWHEEL, 340)

        gray = cv2.cvtColor(image.value, cv2.COLOR_BGR2GRAY)
        turn = edge_detect(gray)
        if turn != 0:
            if up.CDS_GetCurPos(LFOOT) < 530 and flag_attack.value == 1:
                continue
            else:
                print(up.CDS_GetCurPos(LFOOT))
                if flag_stop_autopilot.value == 0:
                    flag_turning.value = 1
                    turning_counter = 3
                    if turn == 1:
                        turn_left()
                        turn_left()
                        turn_left()
                        slow(500)
                        forward(1000)
                    elif turn == 2:
                        turn_left()
                        turn_left()
                        turn_left()
                        slow(500)
                        forward(1000)
                    elif turn == 3:
                        turn_right()
                        turn_right()
                        turn_right()
                        slow(500)
                        forward(1000)
                    elif turn == 4:
                        turn_left()
                        turn_left()
                        turn_left()
                        forward(1000)
                    elif turn == 5:
                        turn_right()
                        turn_right()
                        turn_right()
                        forward(1000)
        turning_counter -= 1
        if turning_counter < 1:
            flag_turning.value = 0


def detect_tag(image, flag_stop_autopilot, flag_video_ok, flag_tracking, flag_turning, tracking_counter):
    global tag_counter
    while True:
        pid_output = 0
        tag_distance = 0
        speed = 340
        if flag_stop_autopilot.value == 0:
            robot_x = robot_detect(image.value)
            if not robot_x is None:
                flag_tracking.value = 1
                tracking_counter.value = 3
                input_value = robot_x - video_width / 2
                pid_output = pid.update(input_value, 0)
                if flag_turning.value == 0:
                    up.CDS_SetSpeed(RWHEEL, -speed+10 - pid_output)
                    up.CDS_SetSpeed(LWHEEL, speed - pid_output)
                print("robot x:" + str(robot_x) + " pid_output:" + str(pid_output))
            else:
                tracking_counter.value -= 1
                if tracking_counter.value < 1:
                    flag_tracking.value = 0
        gray = cv2.cvtColor(image.value, cv2.COLOR_BGR2GRAY)
        results = atag.detect(gray)
        results_len = len(results)
        if results_len == 0:
            print("tag not found:",tag_counter)
            tag_counter -= 1
            if tag_counter < 1:
                flag_stop_autopilot.value = 0
            continue
        # 确认标签ID
        flag_found = False
        for i in range(results_len):
            tag_id = results[i].tag_id
            if tag_id == 0:
                flag_found = True
                break
        if flag_found == False:
            print("tag not match id:",tag_counter)
            continue
        # 判断距离，获取距离最近的标签的下标
        index = 0
        print(results_len,end='')
        if results_len > 1:
            for i in range(1,results_len):
                if atag.get_distance(results[index].homography, 4300) > atag.get_distance(results[i].homography, 4300):
                    index = i
        tag_distance = atag.get_distance(results[index].homography, 4300)
        print("id:" + str(tag_id) + " distance:" + str(tag_distance), end="")
        flag_stop_autopilot.value = 1
        tag_counter = 3
        if tag_distance < 100:
            speed = 300
            release()
        if tag_distance < 48:
            print("push")
            push_tag(flag_stop_autopilot)
            continue
        # 获取标签x坐标，与屏幕中心作差值进行pid运算
        tag_x = np.array(results[index].center).astype(int)[0]
        input_value = tag_x - video_width / 2
        pid_output = pid.update(input_value, 0)
        if flag_turning.value == 0:
            up.CDS_SetSpeed(RWHEEL, -speed+10 - pid_output)
            up.CDS_SetSpeed(LWHEEL, speed - pid_output)
        print("tag x:" + str(tag_x) + " pid_output:" + str(pid_output))


def push_tag(flag_stop_autopilot):
    # 准备推动
    stop()
    up.CDS_SetAngle(LFOOT, 483, 256)
    up.CDS_SetAngle(RFOOT, 483, 256)
    up.CDS_SetAngle(LELBOW, 240, 256)
    up.CDS_SetAngle(RELBOW, 240, 256)
    up.CDS_SetAngle(LSHOULDER, 908, 256)
    up.CDS_SetAngle(RSHOULDER, 116, 256)
    up.CDS_SetAngle(LHAND, 594, 256)
    up.CDS_SetAngle(RHAND, 430, 256)
    time.sleep(0.4)
    # 抬起手臂
    up.CDS_SetAngle(LELBOW, 355, 900)
    up.CDS_SetAngle(RELBOW, 355, 900)
    up.CDS_SetAngle(LHAND, 594, 512)
    up.CDS_SetAngle(RHAND, 430, 512)
    time.sleep(0.3)
    # 挥动肩膀推下标签
    up.CDS_SetAngle(LELBOW, 385, 900)
    up.CDS_SetAngle(RELBOW, 385, 900)
    up.CDS_SetAngle(LHAND, 410, 512)
    up.CDS_SetAngle(RHAND, 624, 512)
    up.CDS_SetAngle(LSHOULDER, 512, 1000)
    up.CDS_SetAngle(RSHOULDER, 512, 1000)
    time.sleep(0.5)
    # 推下完毕，恢复姿态，后退
    up.CDS_SetAngle(LSHOULDER, 808, 256)
    up.CDS_SetAngle(RSHOULDER, 216, 256)
    up.CDS_SetAngle(LELBOW, 206, 256)
    up.CDS_SetAngle(RELBOW, 206, 256)
    up.CDS_SetAngle(LFOOT, 552, 256)
    up.CDS_SetAngle(RFOOT, 552, 256)
    up.CDS_SetAngle(LHAND, 512, 256)
    up.CDS_SetAngle(RHAND, 512, 256)
    time.sleep(1.0)
    go_back()
    time.sleep(0.5)
    turn_left()
    turn_left()
    flag_stop_autopilot.value = 0
    slow(500)
    forward(1000)

def videocap(image, flag_video_ok):
    cap = cv2.VideoCapture("/dev/video0")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)
    flag_video_ok.value, image.value = cap.read()
    while True:
        flag_video_ok.value, image.value = cap.read()
        if flag_video_ok.value == 0:
            image.value = np.zeros((video_height, video_width, 3), np.uint8)
            print("video error, try reload")
            cap.release()
            subprocess.call("sudo modprobe -r uvcvideo", shell=True)
            time.sleep(0.5)
            subprocess.call("sudo modprobe uvcvideo", shell=True)
            time.sleep(0.3)
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)
            flag_video_ok.value, image.value = cap.read()
            print("video reloaded")
            

def dataget(datas):
    up_controller = UpController()
    while True:
        time.sleep(0.05)
        datas[0] = up_controller.io_data
        datas[1] = up_controller.adc_data

def main(flag_stop_autopilot,datas,flag_attack,stand_event,flag_tracking,tracking_counter):
    # reset_to_512()
    # time.sleep(2)
    # exit(0)
    global attack_counter
    global front_hit_counter
    release()
    while not start_event.is_set():
        datas[0] = up_controller.io_data
        datas[1] = up_controller.adc_data
        print(datas[1][DISTANCE_FRONT])
        time.sleep(0.1)
        if datas[1][DISTANCE_HEAD] > 350:
            start_event.set()
    up.CDS_SetSpeed(RWHEEL, -430)
    up.CDS_SetSpeed(LWHEEL, 490)
    up.CDS_SetAngle(LFOOT, 489, 256)
    up.CDS_SetAngle(RFOOT, 489, 256)
    up.CDS_SetSpeed(RWHEEL, -430)
    up.CDS_SetSpeed(LWHEEL, 490)
    print(482)
    time.sleep(1.3)
    up.CDS_SetAngle(LFOOT, 542, 512)
    up.CDS_SetAngle(RFOOT, 542, 512)
    time.sleep(0.2)
    up.CDS_SetSpeed(RWHEEL, -320)
    up.CDS_SetSpeed(LWHEEL, 460)
    print(542)
    time.sleep(2)
    forward(2000)
    while True:
        if not stand_event.is_set():
            iodata = datas[0]
            adcdata = datas[1]
            if len(iodata) != 8:
                iodata = [0,0,0,0,1,1,0,0]
                print("reset iodata")
            if len(adcdata) != 10:
                adcdata = [0,0,0,0,2000,0,0,0,0,0]
                print("reset adcdata")
            if adcdata[ANGLE] < 800 or adcdata[ANGLE] > 2900:
                continue
            if flag_stop_autopilot.value == 1:
                time.sleep(0.2)
                continue
            if adcdata[DISTANCE_LEFT] > 350:
                print("left hit")
                flag_attack.value = 1
                attack_counter = 3
                flag_tracking.value = 1
                tracking_counter.value = 3
                up.CDS_SetSpeed(RWHEEL, -390)
                up.CDS_SetSpeed(LWHEEL, -370)
                hit_3_L()
                up.CDS_SetSpeed(RWHEEL, 0)
                up.CDS_SetSpeed(LWHEEL, 0)
                print(datas[0],datas[1])
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value) +
                    " turning:" + str(flag_turning.value))
                continue
            if adcdata[DISTANCE_RIGHT] > 350:
                print("right hit")
                flag_attack.value = 1
                attack_counter = 3
                flag_tracking.value = 1
                tracking_counter.value = 3
                up.CDS_SetSpeed(RWHEEL, 390)
                up.CDS_SetSpeed(LWHEEL, 370)
                hit_3_R()
                up.CDS_SetSpeed(RWHEEL, 0)
                up.CDS_SetSpeed(LWHEEL, 0)
                print(datas[0],datas[1])
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value) +
                    " turning:" + str(flag_turning.value))
            if iodata[DISTANCE_RF] == 0:
                flag_attack.value = 1
                attack_counter = 3
                flag_tracking.value = 1
                tracking_counter.value = 3
                if adcdata[DISTANCE_FRONT] > 350 or adcdata[DISTANCE_HEAD] > 350:
                    hit_3_RF()
                    print("right front hit")
                else:
                    hit_3_RF_S()
                    print("right front hit with swing")
                print(datas[0],datas[1])
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value) +
                    " turning:" + str(flag_turning.value))
                continue
            if iodata[DISTANCE_LF] == 0:
                flag_attack.value = 1
                attack_counter = 3
                flag_tracking.value = 1
                tracking_counter.value = 3
                if adcdata[DISTANCE_FRONT] > 350 or adcdata[DISTANCE_HEAD] > 350:
                    hit_3_LF()
                    print("left front hit")
                else:
                    hit_3_LF_S()
                    print("left front hit with swing")
                print(datas[0],datas[1])
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value) +
                    " turning:" + str(flag_turning.value))
                continue
            if (adcdata[DISTANCE_FRONT] > 350 or adcdata[DISTANCE_HEAD] > 350):
                print("front hit")
                flag_attack.value = 1
                attack_counter = 3
                flag_tracking.value = 1
                tracking_counter.value = 3
                if iodata[DISTANCE_RF] == 0:
                    up.CDS_SetSpeed(RWHEEL, -350)
                    up.CDS_SetSpeed(LWHEEL, -270)
                    hit_2R()
                    up.CDS_SetSpeed(RWHEEL, 0)
                    up.CDS_SetSpeed(LWHEEL, 0)
                elif iodata[DISTANCE_RF] == 0:
                    up.CDS_SetSpeed(RWHEEL, 290)
                    up.CDS_SetSpeed(LWHEEL, 330)
                    hit_2L()
                    up.CDS_SetSpeed(RWHEEL, 0)
                    up.CDS_SetSpeed(LWHEEL, 0)
                else:
                    if front_hit_counter % 2 == 1:
                        up.CDS_SetSpeed(RWHEEL, -350)
                        up.CDS_SetSpeed(LWHEEL, -270)
                        hit_2R()
                        up.CDS_SetSpeed(RWHEEL, 0)
                        up.CDS_SetSpeed(LWHEEL, 0)
                    else:
                        up.CDS_SetSpeed(RWHEEL, 290)
                        up.CDS_SetSpeed(LWHEEL, 330)
                        hit_2L()
                        up.CDS_SetSpeed(RWHEEL, 0)
                        up.CDS_SetSpeed(LWHEEL, 0)
                    front_hit_counter += 1
                print(iodata,adcdata)
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value) +
                    " turning:" + str(flag_turning.value))
                continue
            attack_counter -= 1
            if attack_counter < 1:
                flag_attack.value = 0
            tracking_counter.value -= 1
            if tracking_counter.value < 1:
                flag_tracking.value = 0
        

if __name__ == '__main__':
    atag = Atag()
    stand_event = multiprocessing.Event()
    start_event = multiprocessing.Event()
    flag_stop_autopilot = multiprocessing.Manager().Value('i',0)
    flag_attack = multiprocessing.Manager().Value('i',0)
    flag_video_ok = multiprocessing.Manager().Value('i',0)
    flag_tracking = multiprocessing.Manager().Value('i',0)
    flag_turning = multiprocessing.Manager().Value('i',0)
    hit = multiprocessing.Manager().Value('i',0)
    tracking_counter = multiprocessing.Manager().Value('i',0)
    datas = multiprocessing.Manager().list([[],[]])
    dataget_proc = multiprocessing.Process(target=dataget, args=(datas,))
    dataget_proc.start()
    print("dataget start")
    import uptech
    up = uptech.UpTech()
    up.ADC_IO_Open()
    # up.ADC_Led_SetColor(0, 0x07E0)
    # up.ADC_Led_SetColor(1, 0x07E0)
    up.CDS_Open()
    servo_ids = [LFOOT, RFOOT, LSHOULDER, RSHOULDER, LELBOW, LHAND, RELBOW, RHAND]
    motor_ids = [RWHEEL, LWHEEL]
    up_controller.set_cds_mode(servo_ids, 0)
    up_controller.set_cds_mode(motor_ids, 1)
    main_proc = multiprocessing.Process(target=main, args=(flag_stop_autopilot,datas,flag_attack,stand_event,flag_tracking,tracking_counter,))
    main_proc.start()
    print("main start")
    import cv2
    image = multiprocessing.Manager().Value(cv2.CV_8UC3, None)
    import numpy as np
    while not start_event.is_set():
        print("wait for start")
    videocap_proc = multiprocessing.Process(target=videocap, args=(image, flag_video_ok,))
    videocap_proc.start()
    print("videocap start")
    while flag_video_ok.value == 0:
        pass
    print("video ok")

    detect_tag_proc = multiprocessing.Process(target=detect_tag, args=(image,flag_stop_autopilot,flag_video_ok,flag_tracking,flag_turning,tracking_counter))
    detect_tag_proc.start()
    print("detect_tag start")

    autopilot_proc = multiprocessing.Process(target=autopilot, args=(flag_stop_autopilot,datas,flag_attack,stand_event,start_event,flag_tracking,flag_turning,image,tracking_counter,))
    autopilot_proc.start()
    print("autopilot start")

    import signal
    signal.signal(signal.SIGINT, signal_handler)

    subprocess.call("v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=50", shell=True)
    print("exposure set")
    videocap_proc.join()
    main_proc.join()
    autopilot_proc.join()
