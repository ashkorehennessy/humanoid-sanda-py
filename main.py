#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time
import signal
import cv2
import threading
from pid import pid
from atag import Atag
from up_controller import UpController

condition = threading.Condition()

pid_output = 0
pid_output_lock = threading.Lock()

tag_size = 0
tag_size_lock = threading.Lock()

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


class Robot:
    def __init__(self):
        self.image = None
        self.up = uptech.UpTech()
        self.up_controller = UpController()
        self.atag = Atag()
        self.pid = pid()
        self.up.ADC_IO_Open()
        self.up.LCD_Open(2)
        #self.up.ADC_Led_SetColor(0, 0x07E0)
        #self.up.ADC_Led_SetColor(1, 0x07E0)
        self.up.LCD_PutString(10, 0, '666')
        self.up.LCD_Refresh()
        self.up.CDS_Open()
        servo_ids = [LFOOT, RFOOT, LSHOULDER, RSHOULDER, LELBOW, LHAND, RELBOW, RHAND]
        motor_ids = [RWHEEL, LWHEEL]
        self.up_controller.set_cds_mode(servo_ids, 0)
        self.up_controller.set_cds_mode(motor_ids, 1)
        signal.signal(signal.SIGINT, self.signal_handler)
        self.video_width = 640
        self.video_height = 480
        self.cap = cv2.VideoCapture(0)
        self.detect_tag_thread = threading.Thread(name="detect_tag_thread",target=self.detect_tag)
        self.autopilot_thread = threading.Thread(name="autopilot_thread",target=self.autopilot)

    def init_video(self):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.video_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.video_height)
        _,self.image = self.cap.read()

    def signal_handler(self, sig, frame):
        self.stop()
        exit(0)

    def test(self):
        while True:
            time.sleep(1)
            self.up.CDS_SetAngle(3, 800, 512)
            time.sleep(1)
            self.up.CDS_SetAngle(3, 200, 512)

    def reset_to_512(self):
        self.up.CDS_SetAngle(3, 512, 256)
        self.up.CDS_SetAngle(4, 512, 256)
        self.up.CDS_SetAngle(5, 512, 256)
        self.up.CDS_SetAngle(6, 512, 256)
        self.up.CDS_SetAngle(7, 512, 256)
        self.up.CDS_SetAngle(8, 512, 256)
        self.up.CDS_SetAngle(9, 512, 256)
        self.up.CDS_SetAngle(10, 512, 256)

    def init(self):
        self.up.CDS_SetAngle(LFOOT, 552, 384)
        self.up.CDS_SetAngle(RFOOT, 552, 384)
        self.up.CDS_SetAngle(LSHOULDER, 512, 384)
        self.up.CDS_SetAngle(RSHOULDER, 512, 384)
        time.sleep(0.5)
        self.up.CDS_SetAngle(LHAND, 512, 384)
        self.up.CDS_SetAngle(RHAND, 512, 384)
        time.sleep(0.5)
        self.up.CDS_SetAngle(LELBOW, 512, 384)
        self.up.CDS_SetAngle(RELBOW, 512, 384)
        time.sleep(0.5)
        self.up.CDS_SetAngle(LSHOULDER, 818, 384)
        self.up.CDS_SetAngle(RSHOULDER, 206, 384)
        time.sleep(0.5)
        self.up.CDS_SetAngle(LELBOW, 206, 384)
        self.up.CDS_SetAngle(RELBOW, 206, 384)
        time.sleep(0.5)

    def forward(self,ms=100):
        self.up.CDS_SetSpeed(RWHEEL, -330)
        self.up.CDS_SetSpeed(LWHEEL, 340)
        time.sleep(ms/1000)

    def boost(self):
        self.up.CDS_SetSpeed(RWHEEL, -480)
        self.up.CDS_SetSpeed(LWHEEL, 520)
        time.sleep(0.3)

    def boost_back(self):
        self.up.CDS_SetSpeed(RWHEEL, 620)
        self.up.CDS_SetSpeed(LWHEEL, -660)
        time.sleep(0.3)

    def go_back(self):
        self.up.CDS_SetSpeed(RWHEEL, 500)
        self.up.CDS_SetSpeed(LWHEEL, -520)
        time.sleep(0.4)

    def turn_left(self):
        self.up.CDS_SetSpeed(RWHEEL, -480)
        self.up.CDS_SetSpeed(LWHEEL, -505)
        time.sleep(0.4)

    def turn_right(self):
        self.up.CDS_SetSpeed(RWHEEL, 480)
        self.up.CDS_SetSpeed(LWHEEL, 505)
        time.sleep(0.4)

    def stop(self):
        self.up.CDS_SetSpeed(RWHEEL, 0)
        self.up.CDS_SetSpeed(LWHEEL, 0)
    
    def back_1(self):
        self.up.CDS_SetAngle(LELBOW, 818, 768)
        self.up.CDS_SetAngle(RELBOW, 818, 768)
        self.up.CDS_SetAngle(LHAND, 698, 768)
        self.up.CDS_SetAngle(RHAND, 326, 768)
        self.up.CDS_SetAngle(LSHOULDER, 512, 768)
        self.up.CDS_SetAngle(RSHOULDER, 512, 768)
    
    def back_2(self):
        self.up.CDS_SetAngle(LSHOULDER, 206, 768)
        self.up.CDS_SetAngle(RSHOULDER, 818, 768)
    
    def back_3(self):
        self.up.CDS_SetAngle(LELBOW, 450, 768)
        self.up.CDS_SetAngle(RELBOW, 450, 768)
        self.up.CDS_SetAngle(LFOOT, 790, 768)
        self.up.CDS_SetAngle(RFOOT, 790, 768)
      
    def back_4(self):
        self.up.CDS_SetAngle(LHAND, 512, 908)
        self.up.CDS_SetAngle(RHAND, 512, 908)
        self.up.CDS_SetAngle(LFOOT, 472, 908)
        self.up.CDS_SetAngle(RFOOT, 472, 908)
        time.sleep(0.1)
        self.up.CDS_SetAngle(LELBOW, 236, 512)
        self.up.CDS_SetAngle(RELBOW, 236, 512)
        self.go_back()
        self.up.CDS_SetAngle(LELBOW, 206, 768)
        self.up.CDS_SetAngle(RELBOW, 206, 768)
        self.up.CDS_SetAngle(LFOOT, 512, 768)
        self.up.CDS_SetAngle(RFOOT, 512, 768)
        self.up.CDS_SetAngle(LSHOULDER, 818, 768)
        self.up.CDS_SetAngle(RSHOULDER, 206, 768)
        time.sleep(0.2)
        self.up.CDS_SetSpeed(RWHEEL, 350)
        self.up.CDS_SetSpeed(LWHEEL, -370)
        time.sleep(0.1)
        self.up.CDS_SetAngle(LFOOT, 542, 768)
        self.up.CDS_SetAngle(RFOOT, 542, 768)
        time.sleep(0.1)
        self.stop()



    def back_5(self):
        #self.go_back()
        #self.stop()
        #time.sleep(0.5)
        #self.up.CDS_SetAngle(LFOOT, 512, 800)
        #self.up.CDS_SetAngle(RFOOT, 512, 800)
        #self.up.CDS_SetAngle(LHAND, 512, 800)
        #self.up.CDS_SetAngle(RHAND, 512, 800)
        #time.sleep(0.3)
        #self.up.CDS_SetSpeed(RWHEEL, 740)
        #self.up.CDS_SetSpeed(LWHEEL, -700)
        time.sleep(0.5)
        self.stop()
        # time.sleep(0.2)
        # self.up.CDS_SetSpeed(RWHEEL, 330)
        # self.up.CDS_SetSpeed(LWHEEL, -310)
        # time.sleep(0.75)
        # self.stop()

    def front_stand(self):
        time.sleep(0.3)
        self.front_0()
        time.sleep(0.7)
        self.front_1()
        time.sleep(0.6)
        self.front_2()
        time.sleep(0.6)
        self.front_3()
        time.sleep(1.0)
        self.front_4()
        time.sleep(0.9)
        self.front_5()
        time.sleep(1.0)

    def back_stand(self):
        self.back_1()
        time.sleep(0.6)
        self.back_2()
        time.sleep(0.2)
        self.back_3()
        time.sleep(0.9)
        self.back_4()
        time.sleep(0.9)
        self.back_5()
        time.sleep(1.0)

    def hit_left(self):
        self.up.CDS_SetAngle(LFOOT, 462, 650)
        self.up.CDS_SetAngle(RFOOT, 462, 650)
        self.up.CDS_SetAngle(LELBOW, 630, 968)
        self.up.CDS_SetAngle(LHAND, 580, 968)
        self.up.CDS_SetSpeed(RWHEEL, -580)
        self.up.CDS_SetSpeed(LWHEEL, -605)
        time.sleep(0.8)
        self.up.CDS_SetAngle(LELBOW, 680, 612)
        self.up.CDS_SetAngle(LHAND, 630, 612)

    def hit_right(self):
        self.up.CDS_SetAngle(LFOOT, 462, 650)
        self.up.CDS_SetAngle(RFOOT, 462, 650)
        self.up.CDS_SetAngle(RELBOW, 430, 968)
        self.up.CDS_SetAngle(RHAND, 470, 968)
        self.up.CDS_SetSpeed(RWHEEL, 580)
        self.up.CDS_SetSpeed(LWHEEL, 605)

        time.sleep(0.8)
        self.up.CDS_SetAngle(RELBOW, 740, 612)
        self.up.CDS_SetAngle(RHAND, 780, 612)

    def hit_1(self):
        self.up.CDS_SetAngle(LSHOULDER, 170, 512)
        self.up.CDS_SetAngle(LELBOW, 550, 256)
        self.up.CDS_SetAngle(LHAND, 530, 256)
        self.up.CDS_SetAngle(RSHOULDER, 885, 512)
        self.up.CDS_SetAngle(RELBOW, 512, 256)
        self.up.CDS_SetAngle(RHAND, 512, 256)

    def hit_2(self):
        self.up.CDS_SetAngle(LFOOT, 462, 650)
        self.up.CDS_SetAngle(RFOOT, 462, 650)
        self.up.CDS_SetAngle(LELBOW, 630, 968)
        self.up.CDS_SetAngle(LHAND, 580, 968)
        self.up.CDS_SetAngle(RELBOW, 430, 968)
        self.up.CDS_SetAngle(RHAND, 470, 968)
        self.up.CDS_SetAngle(LSHOULDER, 160, 512)
        self.up.CDS_SetAngle(RSHOULDER, 250, 512)

        time.sleep(0.5)
        self.up.CDS_SetAngle(LELBOW, 320, 968)
        self.up.CDS_SetAngle(LHAND, 270, 968)
        self.up.CDS_SetAngle(RELBOW, 740, 968)
        self.up.CDS_SetAngle(RHAND, 780, 968)
        self.up.CDS_SetAngle(LFOOT, 462, 512)
        self.up.CDS_SetAngle(RFOOT, 462, 512)
        self.up.CDS_SetSpeed(LWHEEL, 0)
        self.up.CDS_SetSpeed(RWHEEL, 0)

    def hit_3_L(self):
        self.up.CDS_SetAngle(LFOOT, 462, 650)
        self.up.CDS_SetAngle(RFOOT, 462, 650)
        self.up.CDS_SetSpeed(RWHEEL, -580)
        self.up.CDS_SetSpeed(LWHEEL, -605)

        time.sleep(0.3)
        self.up.CDS_SetAngle(LSHOULDER, 240, 812)
        self.up.CDS_SetAngle(RSHOULDER, 260, 812)
        self.up.CDS_SetAngle(LELBOW, 680, 812)
        self.up.CDS_SetAngle(LHAND, 630, 812)
        self.up.CDS_SetAngle(RELBOW, 640, 812)
        self.up.CDS_SetAngle(RHAND, 680, 812)

        time.sleep(0.6)
        self.up.CDS_SetSpeed(RWHEEL, 480)
        self.up.CDS_SetSpeed(LWHEEL, 505)

        time.sleep(0.3)
        self.up.CDS_SetAngle(LELBOW, 420, 812)
        self.up.CDS_SetAngle(LHAND, 370, 812)
        self.up.CDS_SetAngle(RELBOW, 380, 812)
        self.up.CDS_SetAngle(RHAND, 420, 812)

        time.sleep(0.6)
        self.up.CDS_SetAngle(LFOOT, 462, 512)
        self.up.CDS_SetAngle(RFOOT, 462, 512)

    def hit_3_R(self):
        self.up.CDS_SetAngle(LFOOT, 462, 650)
        self.up.CDS_SetAngle(RFOOT, 462, 650)
        self.up.CDS_SetSpeed(RWHEEL, 580)
        self.up.CDS_SetSpeed(LWHEEL, 605)

        time.sleep(0.3)
        self.up.CDS_SetAngle(LSHOULDER, 240, 812)
        self.up.CDS_SetAngle(RSHOULDER, 260, 812)
        self.up.CDS_SetAngle(LELBOW, 420, 812)
        self.up.CDS_SetAngle(LHAND, 370, 812)
        self.up.CDS_SetAngle(RELBOW, 380, 812)
        self.up.CDS_SetAngle(RHAND, 420, 812)

        time.sleep(0.6)
        self.up.CDS_SetSpeed(RWHEEL, -500)
        self.up.CDS_SetSpeed(LWHEEL, -535)

        time.sleep(0.3)
        self.up.CDS_SetAngle(LELBOW, 680, 812)
        self.up.CDS_SetAngle(LHAND, 630, 812)
        self.up.CDS_SetAngle(RELBOW, 640, 812)
        self.up.CDS_SetAngle(RHAND, 680, 812)

        time.sleep(0.6)
        self.up.CDS_SetAngle(LFOOT, 462, 512)
        self.up.CDS_SetAngle(RFOOT, 462, 512)

    def hit_4(self):
        self.up.CDS_SetAngle(LFOOT, 462, 650)
        self.up.CDS_SetAngle(RFOOT, 462, 650)
        self.up.CDS_SetAngle(LELBOW, 380, 512)
        self.up.CDS_SetAngle(LHAND, 330, 512)
        self.up.CDS_SetAngle(RELBOW, 340, 512)
        self.up.CDS_SetAngle(RHAND, 380, 512)

        time.sleep(0.7)
        self.up.CDS_SetAngle(LSHOULDER, 170, 768)
        self.up.CDS_SetAngle(RSHOULDER, 885, 768)
        self.up.CDS_SetSpeed(RWHEEL, -500)
        self.up.CDS_SetSpeed(LWHEEL, -515)

        time.sleep(1.8)
        self.up.CDS_SetAngle(LFOOT, 462, 512)
        self.up.CDS_SetAngle(RFOOT, 462, 512)

    def alert(self):
        self.up.CDS_SetAngle(LSHOULDER, 170, 812)
        self.up.CDS_SetAngle(RSHOULDER, 280, 812)
        self.up.CDS_SetAngle(LELBOW, 320, 256)
        self.up.CDS_SetAngle(LHAND, 270, 256)
        self.up.CDS_SetAngle(RELBOW, 740, 256)
        self.up.CDS_SetAngle(RHAND, 780, 256)

    def alert_delay(self):
        self.up.CDS_SetAngle(LSHOULDER, 170, 812)
        self.up.CDS_SetAngle(RSHOULDER, 280, 812)

        time.sleep(0.6)
        self.up.CDS_SetAngle(LELBOW, 320, 256)
        self.up.CDS_SetAngle(LHAND, 270, 256)
        self.up.CDS_SetAngle(RELBOW, 740, 256)
        self.up.CDS_SetAngle(RHAND, 780, 256)

    def reconfig(self):
        servo_ids = [LFOOT, RFOOT, LSHOULDER, RSHOULDER, LELBOW, LHAND, RELBOW, RHAND]
        motor_ids = [RWHEEL, LWHEEL]
        self.up_controller.set_cds_mode(servo_ids, 0)
        self.up_controller.set_cds_mode(motor_ids, 1)

    def attack_back(self):
        self.up.CDS_SetSpeed(RWHEEL, 400)
        self.up.CDS_SetSpeed(LWHEEL, -420)
        self.alert_delay()
        time.sleep(0.4)
        self.stop()

    def autopilot(self):
        global pid_output
        global tag_size
        global condition

        while True:
            with condition:
                while tag_size > 69:
                    condition.wait()
                # 腿部姿态修正
                #self.up.CDS_SetAngle(LFOOT, 552, 384)
                #self.up.CDS_SetAngle(RFOOT, 552, 384)

                auto_pilot_index = 0
                if self.up_controller.io_data[0] == 1:
                    auto_pilot_index += 1
                if self.up_controller.io_data[1] == 1:
                    auto_pilot_index += 2
                if self.up_controller.io_data[2] == 1:
                    auto_pilot_index += 4
                if self.up_controller.io_data[3] == 1:
                    auto_pilot_index += 8

                if auto_pilot_index == 0:
                    pid_output_lock.acquire()
                    self.up.CDS_SetSpeed(RWHEEL, -330 - pid_output)
                    self.up.CDS_SetSpeed(LWHEEL, 340 - pid_output)
                    pid_output_lock.release()
                elif auto_pilot_index == 1:
                    self.go_back()
                    self.turn_right()
                    self.forward(500)
                elif auto_pilot_index == 2:
                    self.turn_right()
                    self.forward(500)
                elif auto_pilot_index == 3:
                    self.turn_right()
                    self.turn_right()
                    self.forward(1000)
                elif auto_pilot_index == 4:
                    self.go_back()
                    self.turn_left()
                    self.forward(500)
                elif auto_pilot_index == 5:
                    self.go_back()
                    self.turn_left()
                    self.turn_left()
                    self.forward(300)
                elif auto_pilot_index == 8:
                    self.turn_left()
                    self.forward(500)
                elif auto_pilot_index == 10:
                    self.boost()
                    self.boost()
                    self.forward()
                elif auto_pilot_index == 12:
                    self.turn_left()
                    self.turn_left()
                    self.forward(500)
                elif auto_pilot_index == 15:
                    self.stop()
                else:
                    pass

    def detect_tag(self):
        global pid_output
        global tag_size
        global condition
        while True:
            with condition:
                pid_output_lock.acquire()
                pid_output = 0
                pid_output_lock.release()
                tag_size_lock.acquire()
                tag_size = 0
                tag_size_lock.release()
                _,self.image = self.cap.read()
                results = self.atag.detect(self.image)
                if len(results) == 0:
                    continue
                # 确认标签ID
                tag_id = self.atag.get_id(results)
                if tag_id != 0:
                    continue
                # 使用标签实际大小判断距离
                tag_size_lock.acquire()
                tag_size = self.atag.get_size(results)
                tag_size_lock.release()
                print("id:"+str(tag_id)+" size:"+str(tag_size),end="")
                if tag_size > 69:
                    print("")
                    print("push")
                    self.push_tag()
                    condition.notify()
                    continue
                # 获取标签x坐标，与屏幕中心作差值进行pid运算
                tag_x = self.atag.get_center(results)[0]
                input_value = tag_x - self.video_width / 2
                pid_output_lock.acquire()
                pid_output = self.pid.update(input_value,0)
                print(" x:"+str(tag_x)+" pid_output:"+str(pid_output))
                pid_output_lock.release()

    def push_tag(self):
        self.up.CDS_SetSpeed(RWHEEL, -290)
        self.up.CDS_SetSpeed(LWHEEL, 300)
        time.sleep(0.6)
        # 准备推动
        self.stop()
        self.up.CDS_SetAngle(LFOOT, 483, 256)
        self.up.CDS_SetAngle(RFOOT, 483, 256)
        self.up.CDS_SetAngle(LELBOW, 240, 256)
        self.up.CDS_SetAngle(RELBOW, 240, 256)
        self.up.CDS_SetAngle(LSHOULDER, 908, 256)
        self.up.CDS_SetAngle(RSHOULDER, 116, 256)
        time.sleep(0.4)
        # 抬起手臂
        self.up.CDS_SetAngle(LELBOW, 355, 900)
        self.up.CDS_SetAngle(RELBOW, 355, 900)
        time.sleep(0.3)
        # 挥动肩膀推下标签
        self.up.CDS_SetAngle(LSHOULDER, 512, 1000)
        self.up.CDS_SetAngle(RSHOULDER, 512, 1000)
        time.sleep(0.5)
        # 推下完毕，恢复姿态，后退
        self.up.CDS_SetAngle(LSHOULDER, 808, 256)
        self.up.CDS_SetAngle(RSHOULDER, 216, 256)
        self.up.CDS_SetAngle(LELBOW, 206, 256)
        self.up.CDS_SetAngle(RELBOW, 206, 256)
        self.up.CDS_SetAngle(LFOOT, 552, 256)
        self.up.CDS_SetAngle(RFOOT, 552, 256)
        time.sleep(1.0)
        self.go_back()
        self.turn_left()
        self.turn_left()
        self.forward(1000)

    def main(self):
        self.init_video()
        self.reconfig()
        #self.init()

        enemy_L1 = 0  # 左前方敌人检测
        enemy_L2 = 0  # 左侧敌人检测
        enemy_R1 = 0  # 右前方敌人检测
        enemy_R2 = 0  # 右侧敌人检测
        enemy_FRONT = 0  # 前方敌人检测
        enemy_BACK = 0  # 后方敌人检测

        angle = 0  # 倾角数值
        distance = 0  # 距离数值，距离越近数值越大

        attacking = 0  # 正在攻击
        attack_times = 0  # 记录攻击次数，每一轮攻击超过3次自动后退逃跑
        is_init = 1

        servo_ids = [LFOOT, RFOOT, LSHOULDER, RSHOULDER, LELBOW, LHAND, RELBOW, RHAND]
        motor_ids = [RWHEEL, LWHEEL]
        self.up_controller.set_cds_mode(servo_ids, 0)
        self.up_controller.set_cds_mode(motor_ids, 1)
        #time.sleep(3)

        #while self.up_controller.adc_data[1] < 150:
        #    pass  # 接近头部时跳出循环
        #time.sleep(2)
        #self.reset_to_512()
        #self.boost()
        #time.sleep(0.2)
        #self.forward()
        self.stop()
        self.autopilot_thread.start()
        self.detect_tag_thread.start()
        count=0

        while True:
            #self.autopilot()
            #_,self.image = self.cap.read()
            #self.detect_tag()
            #self.reset_to_512()
            self.back_stand()
            time.sleep(10)


if __name__ == '__main__':
    robot = Robot()
    robot.main()
