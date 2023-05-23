#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time
from up_controller import UpController

class SPIDER:
    def __init__(self):
        self.up=uptech.UpTech()
        self.up_controller=UpController()
        self.up.ADC_IO_Open()
        self.up.LCD_Open(2)
        self.up.ADC_Led_SetColor(0,0x07E0)
        self.up.ADC_Led_SetColor(1,0x07E0)
        self.up.LCD_PutString(10, 0, 'uptech_spider')
        self.up.LCD_Refresh()
        self.up.CDS_Open()
        servo_ids = [1,2,3,4,5,6,7,8,9,10]
        self.up_controller.set_cds_mode(servo_ids,0)  #设置舵机为舵机模式
        self.up.CDS_SetAngle(1, 512, 512)
        self.up.CDS_SetAngle(2, 190, 512)
        self.up.CDS_SetAngle(3, 512, 512)
        self.up.CDS_SetAngle(4, 190, 512)
        self.up.CDS_SetAngle(5, 512, 512)
        self.up.CDS_SetAngle(6, 190, 512)
        self.up.CDS_SetAngle(7, 512, 512)
        self.up.CDS_SetAngle(8, 190, 512)
        self.up.CDS_SetAngle(9, 512, 512)
        self.up.CDS_SetAngle(10, 512, 512)

    def start(self):
        while True:
            time.sleep(3)
            self.up.CDS_SetAngle(3, 800, 256)

if __name__ == '__main__':
    spider = SPIDER()
    spider.start() 
    
    
    
            

    





