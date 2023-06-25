#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time
import signal
import cv2
import multiprocessing
import sys
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
video_width = 640
video_height = 480


def signal_handler():
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


def boost():
    up.CDS_SetSpeed(RWHEEL, -480)
    up.CDS_SetSpeed(LWHEEL, 520)
    time.sleep(0.3)


def boost_back():
    up.CDS_SetSpeed(RWHEEL, 620)
    up.CDS_SetSpeed(LWHEEL, -660)
    time.sleep(0.3)


def go_back():
    up.CDS_SetSpeed(RWHEEL, 500)
    up.CDS_SetSpeed(LWHEEL, -520)
    time.sleep(0.4)


def turn_left():
    up.CDS_SetSpeed(RWHEEL, -480)
    up.CDS_SetSpeed(LWHEEL, -505)
    time.sleep(0.4)


def turn_right():
    up.CDS_SetSpeed(RWHEEL, 480)
    up.CDS_SetSpeed(LWHEEL, 505)
    time.sleep(0.4)


def stop():
    up.CDS_SetSpeed(RWHEEL, 0)
    up.CDS_SetSpeed(LWHEEL, 0)


def back_1():
    up.CDS_SetAngle(LELBOW, 818, 512)
    up.CDS_SetAngle(RELBOW, 818, 512)
    up.CDS_SetAngle(LHAND, 668, 512)
    up.CDS_SetAngle(RHAND, 336, 512)
    up.CDS_SetAngle(LSHOULDER, 512, 512)
    up.CDS_SetAngle(RSHOULDER, 512, 512)


def back_2():
    up.CDS_SetAngle(LSHOULDER, 206, 768)
    up.CDS_SetAngle(RSHOULDER, 818, 768)


def back_3():
    up.CDS_SetAngle(LELBOW, 450, 768)
    up.CDS_SetAngle(RELBOW, 450, 768)
    up.CDS_SetAngle(LFOOT, 770, 768)
    up.CDS_SetAngle(RFOOT, 770, 768)


def back_4():
    up.CDS_SetAngle(LHAND, 512, 908)
    up.CDS_SetAngle(RHAND, 512, 908)
    up.CDS_SetAngle(LELBOW, 370, 768)
    up.CDS_SetAngle(RELBOW, 370, 768)
    up.CDS_SetAngle(LFOOT, 472, 768)
    up.CDS_SetAngle(RFOOT, 472, 768)
    time.sleep(0.2)
    up.CDS_SetAngle(LSHOULDER, 818, 768)
    up.CDS_SetAngle(RSHOULDER, 206, 768)
    # up.CDS_SetSpeed(RWHEEL, 350)
    # up.CDS_SetSpeed(LWHEEL, -370)
    time.sleep(0.1)
    up.CDS_SetAngle(LFOOT, 542, 768)
    up.CDS_SetAngle(RFOOT, 542, 768)
    up.CDS_SetAngle(LELBOW, 206, 512)
    up.CDS_SetAngle(RELBOW, 206, 512)
    time.sleep(0.1)
    stop()


def front_0():
    pass


def front_1():
    pass


def front_2():
    pass


def front_3():
    pass


def front_4():
    pass


def front_stand():
    time.sleep(0.3)
    front_0()
    time.sleep(0.4)
    front_1()
    time.sleep(0.6)
    front_2()
    time.sleep(0.6)
    front_3()
    time.sleep(0.6)
    front_4()
    time.sleep(0.4)

def back_stand():
    back_1()
    time.sleep(0.6)
    back_2()
    time.sleep(0.8)
    back_3()
    time.sleep(0.9)
    back_4()
    time.sleep(0.6)

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
    up.CDS_SetAngle(LFOOT, 512, 256)
    up.CDS_SetAngle(RFOOT, 512, 256)
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
    forward(1000)

def hit_2():
    up.CDS_SetAngle(LFOOT, 512, 256)
    up.CDS_SetAngle(RFOOT, 512, 256)
    up.CDS_SetAngle(LELBOW, 240, 256)
    up.CDS_SetAngle(RELBOW, 240, 256)
    up.CDS_SetAngle(LSHOULDER, 908, 256)
    up.CDS_SetAngle(RSHOULDER, 116, 256)
    time.sleep(0.4)
    # 抬起手臂
    up.CDS_SetAngle(LELBOW, 435, 900)
    up.CDS_SetAngle(RELBOW, 435, 900)
    time.sleep(0.4)
    # 挥动肩膀推下标签
    up.CDS_SetAngle(LSHOULDER, 700, 768)
    up.CDS_SetAngle(RSHOULDER, 312, 768)
    time.sleep(0.5)
    # 推下完毕，恢复姿态，后退
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
    forward(1000)

def hit_3_L():
    up.CDS_SetAngle(LFOOT, 462, 650)
    up.CDS_SetAngle(RFOOT, 462, 650)
    up.CDS_SetSpeed(RWHEEL, -580)
    up.CDS_SetSpeed(LWHEEL, -605)

    time.sleep(0.3)
    up.CDS_SetAngle(LSHOULDER, 240, 812)
    up.CDS_SetAngle(RSHOULDER, 260, 812)
    up.CDS_SetAngle(LELBOW, 680, 812)
    up.CDS_SetAngle(LHAND, 630, 812)
    up.CDS_SetAngle(RELBOW, 640, 812)
    up.CDS_SetAngle(RHAND, 680, 812)

    time.sleep(0.6)
    up.CDS_SetSpeed(RWHEEL, 480)
    up.CDS_SetSpeed(LWHEEL, 505)

    time.sleep(0.3)
    up.CDS_SetAngle(LELBOW, 420, 812)
    up.CDS_SetAngle(LHAND, 370, 812)
    up.CDS_SetAngle(RELBOW, 380, 812)
    up.CDS_SetAngle(RHAND, 420, 812)

    time.sleep(0.6)
    up.CDS_SetAngle(LFOOT, 462, 512)
    up.CDS_SetAngle(RFOOT, 462, 512)


def hit_3_R():
    up.CDS_SetAngle(LFOOT, 462, 650)
    up.CDS_SetAngle(RFOOT, 462, 650)
    up.CDS_SetSpeed(RWHEEL, 580)
    up.CDS_SetSpeed(LWHEEL, 605)

    time.sleep(0.3)
    up.CDS_SetAngle(LSHOULDER, 240, 812)
    up.CDS_SetAngle(RSHOULDER, 260, 812)
    up.CDS_SetAngle(LELBOW, 420, 812)
    up.CDS_SetAngle(LHAND, 370, 812)
    up.CDS_SetAngle(RELBOW, 380, 812)
    up.CDS_SetAngle(RHAND, 420, 812)

    time.sleep(0.6)
    up.CDS_SetSpeed(RWHEEL, -500)
    up.CDS_SetSpeed(LWHEEL, -535)

    time.sleep(0.3)
    up.CDS_SetAngle(LELBOW, 680, 812)
    up.CDS_SetAngle(LHAND, 630, 812)
    up.CDS_SetAngle(RELBOW, 640, 812)
    up.CDS_SetAngle(RHAND, 680, 812)

    time.sleep(0.6)
    up.CDS_SetAngle(LFOOT, 462, 512)
    up.CDS_SetAngle(RFOOT, 462, 512)


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
    up.CDS_SetAngle(LSHOULDER, 170, 812)
    up.CDS_SetAngle(RSHOULDER, 280, 812)
    up.CDS_SetAngle(LELBOW, 320, 256)
    up.CDS_SetAngle(LHAND, 270, 256)
    up.CDS_SetAngle(RELBOW, 740, 256)
    up.CDS_SetAngle(RHAND, 780, 256)


def alert_delay():
    up.CDS_SetAngle(LSHOULDER, 170, 812)
    up.CDS_SetAngle(RSHOULDER, 280, 812)

    time.sleep(0.6)
    up.CDS_SetAngle(LELBOW, 320, 256)
    up.CDS_SetAngle(LHAND, 270, 256)
    up.CDS_SetAngle(RELBOW, 740, 256)
    up.CDS_SetAngle(RHAND, 780, 256)


def attack_back():
    up.CDS_SetSpeed(RWHEEL, 400)
    up.CDS_SetSpeed(LWHEEL, -420)
    alert_delay()
    time.sleep(0.4)
    stop()


def autopilot(flag_halt):
    up_controller = UpController()
    time.sleep(2)
    up.CDS_SetSpeed(RWHEEL, -430)
    up.CDS_SetSpeed(LWHEEL, 490)
    up.CDS_SetAngle(LFOOT, 482, 512)
    up.CDS_SetAngle(RFOOT, 482, 512)
    time.sleep(2.0)
    up.CDS_SetAngle(LFOOT, 532, 512)
    up.CDS_SetAngle(RFOOT, 532, 512)
    while True:
        if flag_halt.value == 1:
            continue
        iodata = up_controller.io_data
        auto_pilot_index = 0
        if iodata[0] == 1:
            auto_pilot_index += 1
        if iodata[1] == 1:
            auto_pilot_index += 2
        if iodata[2] == 1:
            auto_pilot_index += 4
        if iodata[3] == 1:
            auto_pilot_index += 8

        print(iodata)

        if auto_pilot_index == 0:
            up.CDS_SetSpeed(RWHEEL, -330)
            up.CDS_SetSpeed(LWHEEL, 340)
        elif auto_pilot_index == 1:
            # go_back()
            turn_right()
            turn_right()
            forward(500)
        elif auto_pilot_index == 2:
            turn_right()
            forward(500)
        elif auto_pilot_index == 3:
            turn_right()
            turn_right()
            forward(1000)
        elif auto_pilot_index == 4:
            # go_back()
            turn_left()
            turn_left()
            forward(500)
        elif auto_pilot_index == 5:
            # go_back()
            turn_left()
            turn_left()
            forward(300)
        elif auto_pilot_index == 8:
            turn_left()
            forward(500)
        elif auto_pilot_index == 10:
            # boost()
            # boost()
            forward(500)
        elif auto_pilot_index == 12:
            turn_left()
            turn_left()
            forward(500)
        elif auto_pilot_index == 15:
            stop()
        else:
            pass


def detect_tag(image, flag_halt):
    pid_output = 0
    tag_distance = 0
    results = atag.detect(image.value)
    if len(results) == 0:
        print("not detected")
        flag_halt.value = 0
        return None
    # 确认标签ID
    tag_id = atag.get_id(results)
    if tag_id != 0:
        flag_halt.value = 0
        return None
    # 使用标签实际大小判断距离
    tag_distance = atag.get_distance(results[0].homography, 4300)
    print("id:" + str(tag_id) + " distance:" + str(tag_distance), end="")
    flag_halt.value = 1
    if tag_distance < 65:
        print("push")
        push_tag()
        return None
    # 获取标签x坐标，与屏幕中心作差值进行pid运算
    tag_x = atag.get_center(results)[0]
    input_value = tag_x - video_width / 2
    pid_output = pid.update(input_value, 0)
    up.CDS_SetSpeed(RWHEEL, -330 - pid_output)
    up.CDS_SetSpeed(LWHEEL, 340 - pid_output)
    print(" x:" + str(tag_x) + " pid_output:" + str(pid_output))


def push_tag():
    up.CDS_SetSpeed(RWHEEL, -290)
    up.CDS_SetSpeed(LWHEEL, 300)
    time.sleep(1.0)
    # 准备推动
    stop()
    up.CDS_SetAngle(LFOOT, 483, 256)
    up.CDS_SetAngle(RFOOT, 483, 256)
    up.CDS_SetAngle(LELBOW, 240, 256)
    up.CDS_SetAngle(RELBOW, 240, 256)
    up.CDS_SetAngle(LSHOULDER, 908, 256)
    up.CDS_SetAngle(RSHOULDER, 116, 256)
    time.sleep(0.4)
    # 抬起手臂
    up.CDS_SetAngle(LELBOW, 355, 900)
    up.CDS_SetAngle(RELBOW, 355, 900)
    time.sleep(0.3)
    # 挥动肩膀推下标签
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
    time.sleep(0.5)
    go_back()
    time.sleep(0.5)
    turn_left()
    turn_left()
    forward(1000)

def videocap(image):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)
    while True:
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image.value = gray
        

def main(image,flag_halt,hit):
    # init()
    stop()
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        # _, image = cap.read()
        # detect_tag(image, flag_halt)
        # up.CDS_SetAngle(LFOOT, 552, 256)
        # up.CDS_SetAngle(RFOOT, 552, 256)
        # if hit.value == 1:
        #     hit_3_L()
        # elif hit.value == 2:
        #     hit_3_R()
        # elif hit.value == 3:
        #     hit_2()
        # elif hit.value == 4:
        #     hit_1()
        # hit.value = 0
        # reset_to_512()
        back_stand()
        time.sleep(7)


if __name__ == '__main__':
    image = multiprocessing.Manager().Value(cv2.CV_8UC3, None)
    flag_halt = multiprocessing.Manager().Value('i',0)
    hit = multiprocessing.Manager().Value('i',0)
    process1 = multiprocessing.Process(target=videocap, args=(image,))
    process1.start()
    time.sleep(3)
    process2 = multiprocessing.Process(target=main, args=(image,flag_halt,hit,))
    process2.start()
    process3 = multiprocessing.Process(target=autopilot, args=(flag_halt,))
    # process3.start()
    while True:
        if sys.stdin.read(1) == 'a':
            hit.value = 1
        if sys.stdin.read(1) == 'd':
            hit.value = 2
        if sys.stdin.read(1) == 'w':
            hit.value = 3
        if sys.stdin.read(1) == 's':
            hit.value = 4
    process1.join()
    process2.join()
    process3.join()
