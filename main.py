#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time
from up_controller import UpController


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
        self.up = uptech.UpTech()
        self.up_controller = UpController()
        self.up.ADC_IO_Open()
        self.up.LCD_Open(2)
        self.up.ADC_Led_SetColor(0, 0x07E0)
        self.up.ADC_Led_SetColor(1, 0x07E0)
        self.up.LCD_PutString(10, 0, '混元形意门')
        self.up.LCD_Refresh()
        self.up.CDS_Open()
        servo_ids = [LFOOT, RFOOT, LSHOULDER, RSHOULDER, LELBOW, LHAND, RELBOW, RHAND, LWHEEL, RWHEEL]
        motor_ids = [RWHEEL, LWHEEL]
        self.up_controller.set_cds_mode(servo_ids, 0)
        self.up_controller.set_cds_mode(motor_ids, 1)
        self.up.CDS_SetAngle(LFOOT, 468, 512)
        self.up.CDS_SetAngle(LELBOW, 550, 512)
        self.up.CDS_SetAngle(LHAND, 550, 512)
        self.up.CDS_SetAngle(RFOOT, 468, 512)
        self.up.CDS_SetAngle(RELBOW, 512, 512)
        self.up.CDS_SetAngle(RHAND, 512, 512)
        time.sleep(0.6)
        self.up.CDS_SetAngle(LSHOULDER, 512, 256)
        self.up.CDS_SetAngle(RSHOULDER, 512, 256)

    def test(self):
        while True:
            time.sleep(1)
            self.up.CDS_SetAngle(3, 800, 512)
            time.sleep(1)
            self.up.CDS_SetAngle(3, 200, 512)

    def init(self):
        self.up.CDS_SetAngle(LFOOT, 468, 512)
        self.up.CDS_SetAngle(LELBOW, 550, 512)
        self.up.CDS_SetAngle(LHAND, 550, 512)
        self.up.CDS_SetAngle(RFOOT, 468, 512)
        self.up.CDS_SetAngle(RELBOW, 512, 512)
        self.up.CDS_SetAngle(RHAND, 512, 512)

        time.sleep(0.6)
        self.up.CDS_SetAngle(LSHOULDER, 512, 256)
        self.up.CDS_SetAngle(RSHOULDER, 512, 256)

    def forward(self):
        self.up.CDS_SetSpeed(RWHEEL, -330)
        self.up.CDS_SetSpeed(LWHEEL, 340)
        time.sleep(0.3)

    def boost(self):
        self.up.CDS_SetSpeed(RWHEEL, -480)
        self.up.CDS_SetSpeed(LWHEEL, 520)
        time.sleep(0.3)

    def boost_back(self):
        self.up.CDS_SetSpeed(RWHEEL, 620)
        self.up.CDS_SetSpeed(LWHEEL, -660)
        time.sleep(0.3)

    def go_back(self):
        self.up.CDS_SetSpeed(RWHEEL, 450)
        self.up.CDS_SetSpeed(LWHEEL, -470)
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

    def front_0(self):
        self.up.CDS_SetAngle(LSHOULDER, 70, 780)
        self.up.CDS_SetAngle(RSHOULDER, 140, 780)
    
    def front_1(self):
        self.up.CDS_SetAngle(LELBOW, 910, 768)
        self.up.CDS_SetAngle(RELBOW, 90, 768)
        self.up.CDS_SetAngle(LHAND, 570, 768)
        self.up.CDS_SetAngle(RHAND, 512, 768)
    
    def back_1(self):
        self.up.CDS_SetAngle(LELBOW, 910, 768)
        self.up.CDS_SetAngle(RELBOW, 90, 768)
        self.up.CDS_SetAngle(LHAND, 570, 768)
        self.up.CDS_SetAngle(RHAND, 512, 768)
    
    def front_2(self):
        self.up.CDS_SetAngle(LSHOULDER, 140, 768)
        self.up.CDS_SetAngle(RSHOULDER, 170, 768)
    
    def back_2(self):
        self.up.CDS_SetAngle(LSHOULDER, 880, 768)
        self.up.CDS_SetAngle(RSHOULDER, 870, 768)
    
    def front_3(self):
        self.up.CDS_SetAngle(LHAND, 130, 768)
        self.up.CDS_SetAngle(RHAND, 900, 768)
    
    def back_3(self):
        self.up.CDS_SetAngle(LHAND, 130, 768)
        self.up.CDS_SetAngle(RHAND, 900, 768)
    
    def front_4(self):
        self.up.CDS_SetAngle(LELBOW, 700, 768)
        self.up.CDS_SetAngle(LHAND, 342, 768)
        self.up.CDS_SetAngle(RELBOW, 346, 768)
        self.up.CDS_SetAngle(RHAND, 710, 768)
        self.up.CDS_SetAngle(LFOOT, 853, 512)
        self.up.CDS_SetAngle(RFOOT, 860, 512)
    
    def back_4(self):
        self.up.CDS_SetAngle(LELBOW, 700, 768)
        self.up.CDS_SetAngle(LHAND, 342, 768)
        self.up.CDS_SetAngle(RELBOW, 346, 768)
        self.up.CDS_SetAngle(RHAND, 710, 768)
        self.up.CDS_SetAngle(LFOOT, 221, 512)
        self.up.CDS_SetAngle(RFOOT, 214, 512)

    def front_5(self):
        self.boost()
        self.boost()
        self.boost()
        self.stop()
        time.sleep(0.5)
        self.up.CDS_SetAngle(LELBOW, 550, 300)
        self.up.CDS_SetAngle(RELBOW, 512, 300)
        self.up.CDS_SetAngle(LHAND, 520, 300)
        self.up.CDS_SetAngle(RHAND, 512, 300)
        self.up.CDS_SetAngle(LSHOULDER, 512, 768)
        self.up.CDS_SetAngle(RSHOULDER, 512, 768)
        self.up.CDS_SetAngle(LFOOT, 468, 300)
        self.up.CDS_SetAngle(RFOOT, 468, 300)
        self.up.CDS_SetSpeed(RWHEEL, -700)
        self.up.CDS_SetSpeed(LWHEEL, 740)
        time.sleep(0.2)
        self.up.CDS_SetSpeed(RWHEEL, -310)
        self.up.CDS_SetSpeed(LWHEEL, 330)
        time.sleep(0.75)
        self.up.stop()

    def back_5(self):
        self.go_back()
        self.go_back()
        self.up.stop()
        time.sleep(0.5)
        self.up.CDS_SetAngle(LELBOW, 550, 300)
        self.up.CDS_SetAngle(RELBOW, 512, 300)
        self.up.CDS_SetAngle(LHAND, 520, 300)
        self.up.CDS_SetAngle(RHAND, 512, 300)
        self.up.CDS_SetAngle(LSHOULDER, 512, 768)
        self.up.CDS_SetAngle(RSHOULDER, 512, 768)
        self.up.CDS_SetAngle(LFOOT, 468, 300)
        self.up.CDS_SetAngle(RFOOT, 468, 300)
        self.up.CDS_SetSpeed(RWHEEL, 740)
        self.up.CDS_SetSpeed(LWHEEL, -700)
        time.sleep(0.2)
        self.up.CDS_SetSpeed(RWHEEL, 330)
        self.up.CDS_SetSpeed(LWHEEL, -310)
        time.sleep(0.75)
        self.up.stop()

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
        self.init()
        time.sleep(0.5)
        self.back_1()
        time.sleep(0.6)
        self.back_2()
        time.sleep(0.6)
        self.back_3()
        time.sleep(1.0)
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

    def reinit(self):
        servo_ids = [LFOOT, RFOOT, LSHOULDER, RSHOULDER, LELBOW, LHAND, RELBOW, RHAND, LWHEEL, RWHEEL]
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
        self.up.CDS_SetAngle(LFOOT, 468, 256)
        self.up.CDS_SetAngle(RFOOT, 468, 256)

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
            self.forward()
        elif auto_pilot_index == 1:
            self.go_back()
            self.turn_right()
            self.turn_right()
            self.forward()
        elif auto_pilot_index == 2:
            self.turn_right()
            self.forward()
        elif auto_pilot_index == 3:
            self.turn_right()
            self.turn_right()
            self.forward()
        elif auto_pilot_index == 4:
            self.go_back()
            self.turn_left()
            self.turn_left()
            self.forward()
        elif auto_pilot_index == 5:
            self.go_back()
            self.turn_left()
            self.turn_left()
            self.turn_left()
            self.forward()
        elif auto_pilot_index == 8:
            self.turn_left()
            self.forward()
        elif auto_pilot_index == 10:
            self.boost()
            self.boost()
            self.forward()
        elif auto_pilot_index == 12:
            self.turn_left()
            self.turn_left()
            self.forward()
        elif auto_pilot_index == 15:
            self.stop()
        else:
            pass

    def main(self):
        self.reinit()

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

        servo_ids = [LFOOT, RFOOT, LSHOULDER, RSHOULDER, LELBOW, LHAND, RELBOW, RHAND, LWHEEL, RWHEEL]
        motor_ids = [RWHEEL, LWHEEL]
        self.up_controller.set_cds_mode(servo_ids, 0)
        self.up_controller.set_cds_mode(motor_ids, 1)
        time.sleep(3)

        while self.up_controller.adc_data[1] < 150:
            pass  # 接近头部时跳出循环

        self.init()
        time.sleep(2)
        self.boost()
        time.sleep(1.5)
        self.forward()

        while True:
            if is_init == 1:
                if 1000 > self.up_controller.adc_data[0] > 400:
                    self.alert_delay()
                    is_init = 0

            enemy_L1 = self.up_controller.io_data[4]  # 检测左前敌人
            enemy_L2 = self.up_controller.io_data[5]  # 检测左侧敌人
            enemy_R1 = self.up_controller.io_data[6]  # 检测右前敌人
            enemy_R2 = self.up_controller.io_data[7]  # 检测右侧敌人
            enemy_FRONT = self.up_controller.io_data[8]  # 检测前方敌人
            enemy_BACK = self.up_controller.io_data[9]  # 检测后方敌人及倒地预警

            angle = self.up_controller.adc_data[0]  # 获取AD输入
            distance = self.up_controller.adc_data[1]  # 获取AD输入

            if angle > 1000:  # 前倒站起
                self.stop()
                self.front_stand()
                self.up.CDS_SetAngle(LFOOT, 468, 256)
                self.up.CDS_SetAngle(RFOOT, 468, 256)
                time.sleep(0.2)
                self.alert_delay()
                self.autopilot()
                continue

            if angle < 400:  # 后倒站起
                self.stop()
                self.back_stand()
                self.up.CDS_SetAngle(LFOOT, 468, 256)
                self.up.CDS_SetAngle(RFOOT, 468, 256)
                time.sleep(0.2)
                self.alert_delay()
                self.autopilot()
                continue

            if distance > 100 or enemy_FRONT == 0:  # 检测到敌人
                self.hit_2()
                if attack_times % 3 == 0:
                    self.attack_back()
                attack_times += 1
                self.autopilot()
                continue
            else:  # 检测不到敌人
                attacking = 0
                self.alert()

            if enemy_BACK == 0:  # 后方检测到敌人，向前逃跑防止被偷袭
                self.turn_right()
                self.autopilot()
                continue

            if enemy_L2 == 0:  # 左侧检测到敌人
                self.hit_left()
                self.stop()
                self.autopilot()
                continue

            if enemy_R2 == 0:  # 右侧检测到敌人
                self.hit_right()
                self.stop()
                self.autopilot()
                continue

            if enemy_L1 == 0:  # 左前方检测到敌人
                self.hit_3_L()
                attack_times += 1
                self.stop()
                self.autopilot()
                continue

            if enemy_R1 == 0:  # 右前方检测到敌人
                self.hit_3_R()
                attack_times += 1
                self.stop()
                self.autopilot()
                continue

            self.autopilot()


if __name__ == '__main__':
    robot = Robot()
    robot.main()
