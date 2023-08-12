#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time
import signal
import cv2
import multiprocessing
import numpy as np
import sys
import subprocess
from robot_detect import robot_detect
from edge_detect import edge_detect
from pid import pid
from atag import Atag
from up_controller import UpController
tag_distance = 0

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

up = uptech.UpTech()
up_controller = UpController()
atag = Atag()
pid = pid()
up.ADC_IO_Open()
# up.ADC_Led_SetColor(0, 0x07E0)
# up.ADC_Led_SetColor(1, 0x07E0)
up.LCD_Open(2)
up.LCD_PutString(10, 0, '666')
up.LCD_Refresh()
up.CDS_Open()
servo_ids = [LFOOT, RFOOT, LSHOULDER, RSHOULDER, LELBOW, LHAND, RELBOW, RHAND]
motor_ids = [RWHEEL, LWHEEL]
up_controller.set_cds_mode(servo_ids, 0)
up_controller.set_cds_mode(motor_ids, 1)
flag_stand = 0
video_width = 640
video_height = 480

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
    up.CDS_SetSpeed(RWHEEL, -413)
    up.CDS_SetSpeed(LWHEEL, -393)
    time.sleep(0.4)

def turn_right():
    up.CDS_SetSpeed(RWHEEL, 393)
    up.CDS_SetSpeed(LWHEEL, 413)
    time.sleep(0.4)

def turn_left_back():
    up.CDS_SetSpeed(RWHEEL, -120)
    up.CDS_SetSpeed(LWHEEL, -550)
    time.sleep(0.4)

def turn_right_back():
    up.CDS_SetSpeed(RWHEEL, 550)
    up.CDS_SetSpeed(LWHEEL, 120)
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

def hit_left():
    up.CDS_SetAngle(LFOOT, 462, 650)
    up.CDS_SetAngle(RFOOT, 462, 650)
    up.CDS_SetAngle(LELBOW, 630, 968)
    up.CDS_SetAngle(LHAND, 580, 968)
    up.CDS_SetSpeed(RWHEEL, -580)
    up.CDS_SetSpeed(LWHEEL, -605)
    time.sleep(0.8)
    up.CDS_SetAngle(LELBOW, 680, 612)
    up.CDS_SetAngle(LHAND, 630, 612)

def hit_right():
    up.CDS_SetAngle(LFOOT, 462, 650)
    up.CDS_SetAngle(RFOOT, 462, 650)
    up.CDS_SetAngle(RELBOW, 430, 968)
    up.CDS_SetAngle(RHAND, 470, 968)
    up.CDS_SetSpeed(RWHEEL, 580)
    up.CDS_SetSpeed(LWHEEL, 605)

    time.sleep(0.8)
    up.CDS_SetAngle(RELBOW, 740, 612)
    up.CDS_SetAngle(RHAND, 780, 612)

def hit_1():
    up.CDS_SetAngle(LFOOT, 522, 256)
    up.CDS_SetAngle(RFOOT, 522, 256)
    up.CDS_SetAngle(LELBOW, 230, 512)
    up.CDS_SetAngle(RELBOW, 280, 512)
    up.CDS_SetAngle(LSHOULDER, 908, 512)
    up.CDS_SetAngle(RSHOULDER, 116, 512)
    time.sleep(0.1)
    up.CDS_SetAngle(LHAND, 818, 512)
    up.CDS_SetAngle(RHAND, 206, 512)
    time.sleep(0.4)
    up.CDS_SetAngle(LELBOW, 450, 512)
    up.CDS_SetAngle(RELBOW, 480, 512)
    time.sleep(0.5)
    up.CDS_SetAngle(LHAND, 512, 512)
    up.CDS_SetAngle(RHAND, 512, 512)
    time.sleep(0.6)
    up.CDS_SetAngle(LSHOULDER, 700, 768)
    up.CDS_SetAngle(RSHOULDER, 312, 768)
    time.sleep(0.6)
    up.CDS_SetAngle(LSHOULDER, 808, 256)
    up.CDS_SetAngle(RSHOULDER, 216, 256)
    up.CDS_SetAngle(LELBOW, 206, 256)
    up.CDS_SetAngle(RELBOW, 206, 256)
    up.CDS_SetAngle(LFOOT, 532, 256)
    up.CDS_SetAngle(RFOOT, 532, 256)
    time.sleep(0.5)
    go_back()
    time.sleep(0.5)
    turn_left()
    turn_left()
    slow(500)
    forward(500)

def hit_2():
    up.CDS_SetAngle(LSHOULDER, 880, 712)
    up.CDS_SetAngle(RSHOULDER, 144, 712)
    # 前倾
    up.CDS_SetAngle(LFOOT, 500, 512)
    up.CDS_SetAngle(RFOOT, 500, 512)
    time.sleep(0.2)
    # 伸手
    up.CDS_SetAngle(LELBOW, 425, 968)
    up.CDS_SetAngle(RELBOW, 425, 968)
    # 伸肘
    up.CDS_SetAngle(LHAND, 592, 968)
    up.CDS_SetAngle(RHAND, 432, 968)
    time.sleep(0.5)
    alert()
    time.sleep(0.3)

def hit_3_L():
    # 伸手
    up.CDS_SetAngle(LELBOW, 425, 968)
    # 伸肘
    up.CDS_SetAngle(LHAND, 622, 968)
    time.sleep(0.1)
    # 摆臂
    up.CDS_SetAngle(LSHOULDER, 328, 912)
    time.sleep(0.5)
    # 复位
    up.CDS_SetAngle(LELBOW, 76, 512)
    up.CDS_SetAngle(LHAND, 958, 512)
    time.sleep(0.4)
    alert()

def hit_3_LF():
    up.CDS_SetAngle(LSHOULDER, 696, 912)
    time.sleep(0.1)
    # 伸手
    up.CDS_SetAngle(LELBOW, 425, 968)
    # 伸肘
    up.CDS_SetAngle(LHAND, 622, 968)
    time.sleep(0.4)
    # 复位
    up.CDS_SetAngle(LELBOW, 76, 512)
    up.CDS_SetAngle(LHAND, 958, 512)
    time.sleep(0.4)
    alert()


def hit_3_R():
    # 伸手
    up.CDS_SetAngle(RELBOW, 425, 968)
    # 伸肘
    up.CDS_SetAngle(RHAND, 382, 968)
    time.sleep(0.1)
    # 摆臂
    up.CDS_SetAngle(RSHOULDER, 696, 912)
    time.sleep(0.5)
    # 复位
    up.CDS_SetAngle(RELBOW, 76, 512)
    up.CDS_SetAngle(RHAND, 66, 512)
    time.sleep(0.4)
    alert()

def hit_3_RF():
    up.CDS_SetAngle(RSHOULDER, 328, 912)
    time.sleep(0.1)
    # 伸手
    up.CDS_SetAngle(RELBOW, 425, 968)
    # 伸肘
    up.CDS_SetAngle(RHAND, 382, 968)
    time.sleep(0.4)
    # 复位
    up.CDS_SetAngle(RELBOW, 76, 512)
    up.CDS_SetAngle(RHAND, 66, 512)
    time.sleep(0.4)
    alert()


def hit_4():
    up.CDS_SetAngle(LFOOT, 462, 650)
    up.CDS_SetAngle(RFOOT, 462, 650)
    up.CDS_SetAngle(LELBOW, 380, 512)
    up.CDS_SetAngle(LHAND, 330, 512)
    up.CDS_SetAngle(RELBOW, 340, 512)
    up.CDS_SetAngle(RHAND, 380, 512)

    time.sleep(0.7)
    up.CDS_SetAngle(LSHOULDER, 170, 768)
    up.CDS_SetAngle(RSHOULDER, 885, 768)
    up.CDS_SetSpeed(RWHEEL, -500)
    up.CDS_SetSpeed(LWHEEL, -515)

    time.sleep(1.8)
    up.CDS_SetAngle(LFOOT, 462, 512)
    up.CDS_SetAngle(RFOOT, 462, 512)


def alert():
    up.CDS_SetAngle(LELBOW, 76, 512)
    up.CDS_SetAngle(RELBOW, 76, 512)
    up.CDS_SetAngle(LHAND, 958, 512)
    up.CDS_SetAngle(RHAND, 66, 512)
    up.CDS_SetAngle(LSHOULDER, 838, 712)
    up.CDS_SetAngle(RSHOULDER, 186, 712)
    up.CDS_SetAngle(LFOOT, 542, 512)
    up.CDS_SetAngle(RFOOT, 542, 512)

def release():
    time.sleep(0.2)
    up.CDS_SetAngle(LSHOULDER, 808, 512)
    up.CDS_SetAngle(RSHOULDER, 216, 512)
    up.CDS_SetAngle(LELBOW, 206, 256)
    up.CDS_SetAngle(RELBOW, 206, 256)
    up.CDS_SetAngle(LHAND, 512, 512)
    up.CDS_SetAngle(RHAND, 512, 512)

def autopilot(flag_stop_autopilot,datas,flag_attack,stand_event,start_event,flag_tracking,flag_turning,image):
    global flag_stand
    while True:
        auto_pilot_index = 0
        iodata = datas[0]
        adcdata = datas[1]

        if flag_stop_autopilot.value == 1:
            time.sleep(0.1)
            continue

        if flag_stand == 0:
            if adcdata[ANGLE] < 600:
                flag_stand = 1
                stand_event.set()
                time.sleep(0.5)
            elif adcdata[ANGLE] > 3300:
                flag_stand = 4
                stand_event.set()
                time.sleep(0.5)
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
                up.CDS_SetSpeed(RWHEEL, -480)
                up.CDS_SetSpeed(LWHEEL, 500)
                time.sleep(0.75)
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
                up.CDS_SetSpeed(RWHEEL, 480)
                up.CDS_SetSpeed(LWHEEL, -500)
                time.sleep(0.75)
                up.CDS_SetSpeed(RWHEEL, 0)
                up.CDS_SetSpeed(LWHEEL, 0)
                time.sleep(0.5)
                back_5()
                time.sleep(0.6)
                stand_event.clear()
            continue

        if flag_attack.value == 0:
            alert()

        # if adcdata[DISTANCE_BACK] > 200:
        #     turn_right()
        #     turn_right()

        if iodata[0] == 1:
            auto_pilot_index += 1
        if iodata[3] == 1:
            auto_pilot_index += 4

        if auto_pilot_index != 0:
            print("index:",auto_pilot_index)
        if auto_pilot_index == 1:
            flag_turning.value = 1
            if flag_attack.value == 1:
                turn_right_back()
                turn_right()
                slow(500)
                forward(500)
            else:
                stop()
            flag_turning.value = 0
        elif auto_pilot_index == 4:
            flag_turning.value = 1
            if flag_attack.value == 1:
                turn_left_back()
                turn_left()
                forward(500)
            else:
                stop()
            flag_turning.value = 0
        elif auto_pilot_index == 5:
            flag_turning.value = 1
            if flag_attack.value == 1:
                turn_left_back()
                turn_left()
                forward(500)
            else:
                stop()
            flag_turning.value = 0
        else:
            if flag_tracking.value == 0:
                if flag_attack == 1:
                    up.CDS_SetSpeed(RWHEEL, -290)
                    up.CDS_SetSpeed(LWHEEL, 300)
                else:
                    up.CDS_SetSpeed(RWHEEL, -330)
                    up.CDS_SetSpeed(LWHEEL, 340)

        gray = cv2.cvtColor(image.value, cv2.COLOR_BGR2GRAY)
        turn = edge_detect(gray)
        if turn != 0:
            if flag_attack.value == 1:
                stop()
            else:
                flag_turning.value = 1
                if turn == 1:
                    turn_left_back()
                    turn_left_back()
                    slow(500)
                    forward(1000)
                elif turn == 2:
                    turn_left_back()
                    turn_left_back()
                    slow(500)
                    forward(500)
                elif turn == 3:
                    turn_right_back()
                    turn_right_back()
                    slow(500)
                    forward(500)
                elif turn == 4:
                    turn_left()
                    turn_left()
                    forward(500)
                elif turn == 5:
                    turn_right()
                    turn_right()
                    forward(500)
                flag_turning.value = 0


def detect_tag(image, flag_stop_autopilot, flag_video_ok, flag_tracking, flag_turning):
    while True:
        pid_output = 0
        tag_distance = 0
        speed = 340
        if flag_stop_autopilot.value == 0:
            robot_x = robot_detect(image.value)
            if not robot_x is None:
                flag_tracking.value = 1
                input_value = robot_x - video_width / 2
                pid_output = pid.update(input_value, 0)
                if flag_turning.value == 0:
                    up.CDS_SetSpeed(RWHEEL, -speed+10 - pid_output)
                    up.CDS_SetSpeed(LWHEEL, speed - pid_output)
                print("robot x:" + str(robot_x) + " pid_output:" + str(pid_output))
                continue
        flag_tracking.value = 0
        gray = cv2.cvtColor(image.value, cv2.COLOR_BGR2GRAY)
        results = atag.detect(gray)
        results_len = len(results)
        if results_len == 0:
            flag_stop_autopilot.value = 0
            continue
        # 确认标签ID
        tag_id = atag.get_id(results)
        if tag_id != 0:
            flag_stop_autopilot.value = 0
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
    cap = cv2.VideoCapture(0)
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
        try:
            for i in range(8):
                tmp = datas[0][i]
            for i in range(9):
                tmp = datas[1][i]
            _ = tmp
        except:
            datas[0] = [0,0,0,0,0,0,0,0]
            datas[1] = [0,0,0,0,0,0,0,0,0]

def main(flag_stop_autopilot,datas,flag_attack,stand_event):
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
    time.sleep(1.8)
    up.CDS_SetAngle(LFOOT, 542, 512)
    up.CDS_SetAngle(RFOOT, 542, 512)
    up.CDS_SetSpeed(RWHEEL, -380)
    up.CDS_SetSpeed(LWHEEL, 530)
    print(542)
    time.sleep(1.3)
    forward(2000)
    while True:
        if not stand_event.is_set():
            iodata = datas[0]
            adcdata = datas[1]
            if adcdata[ANGLE] < 600 or adcdata[ANGLE] > 3300:
                continue
            if flag_stop_autopilot.value == 1:
                time.sleep(0.2)
                continue
            if adcdata[DISTANCE_FRONT] > 350 or adcdata[DISTANCE_HEAD] > 350:
                print("front hit")
                flag_attack.value = 1
                hit_2()
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value))
                flag_attack.value = 0
                continue
            if adcdata[DISTANCE_LEFT] > 350:
                print("left hit")
                flag_attack.value = 1
                hit_3_L()
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value))
                flag_attack.value = 0
                continue
            if adcdata[DISTANCE_RIGHT] > 350:
                print("right hit")
                flag_attack.value = 1
                hit_3_R()
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value))
                flag_attack.value = 0
                continue
            if iodata[DISTANCE_RF] == 0:
                print("right front hit")
                flag_attack.value = 1
                hit_3_RF()
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value))
                flag_attack.value = 0
                continue
            if iodata[DISTANCE_LF] == 0:
                print("left front hit")
                flag_attack.value = 1
                hit_3_LF()
                print("front:" + str(adcdata[DISTANCE_FRONT]) + 
                    " head:" + str(adcdata[DISTANCE_HEAD]) + 
                    " left:" + str(adcdata[DISTANCE_LEFT]) + 
                    " right:" + str(adcdata[DISTANCE_RIGHT]) + 
                    " attack:" + str(flag_attack.value) + 
                    " stop_autopilot:" + str(flag_stop_autopilot.value))
                flag_attack.value = 0
                continue
        

if __name__ == '__main__':
    subprocess.call("v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=55", shell=True)
    signal.signal(signal.SIGINT, signal_handler)
    stand_event = multiprocessing.Event()
    start_event = multiprocessing.Event()
    image = multiprocessing.Manager().Value(cv2.CV_8UC3, None)
    flag_stop_autopilot = multiprocessing.Manager().Value('i',0)
    flag_attack = multiprocessing.Manager().Value('i',0)
    flag_video_ok = multiprocessing.Manager().Value('i',0)
    flag_tracking = multiprocessing.Manager().Value('i',0)
    flag_turning = multiprocessing.Manager().Value('i',0)
    hit = multiprocessing.Manager().Value('i',0)
    datas = multiprocessing.Manager().list([[],[]])
    videocap_proc = multiprocessing.Process(target=videocap, args=(image, flag_video_ok,))
    videocap_proc.start()
    print("videocap start")
    print("wait for video ok")
    while flag_video_ok.value == 0:
        time.sleep(0.2)
    print("video ok")
    dataget_proc = multiprocessing.Process(target=dataget, args=(datas,))
    main_proc = multiprocessing.Process(target=main, args=(flag_stop_autopilot,datas,flag_attack,stand_event,))
    autopilot_proc = multiprocessing.Process(target=autopilot, args=(flag_stop_autopilot,datas,flag_attack,stand_event,start_event,flag_tracking,flag_turning,image,))
    detect_tag_proc = multiprocessing.Process(target=detect_tag, args=(image,flag_stop_autopilot,flag_video_ok,flag_tracking,flag_turning,))
    time.sleep(1)
    dataget_proc.start()
    print("dataget start")
    time.sleep(1)
    main_proc.start()
    print("main start")
    while not start_event.is_set():
        pass
    time.sleep(2.0)
    detect_tag_proc.start()
    print("detect_tag start")
    time.sleep(2.0)
    autopilot_proc.start()
    print("autopilot start")
    videocap_proc.join()
    main_proc.join()
    autopilot_proc.join()
