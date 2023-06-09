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
            if self.up_controller.io_data[0]==1 and self.up_controller.io_data[1]==1:   #前进
                self.up.ADC_Led_SetColor(0,0X0000)
                self.up.ADC_Led_SetColor(1,0X0000)
                self.up.CDS_SetAngle(1, 640, 512)
                self.up.CDS_SetAngle(2, 195, 512)
                self.up.CDS_SetAngle(3, 349, 512)
                self.up.CDS_SetAngle(4, 196, 1023)
                self.up.CDS_SetAngle(5, 559, 512)
                self.up.CDS_SetAngle(6, 190, 512)
                self.up.CDS_SetAngle(7, 475, 512)
                self.up.CDS_SetAngle(8, 189, 512)
                self.up.CDS_SetAngle(9, 512, 512)
                self.up.CDS_SetAngle(10, 312, 512)
                time.sleep(0.3)
        
                self.up.CDS_SetAngle(1, 340, 908)
                self.up.CDS_SetAngle(2, 512, 960)
                self.up.CDS_SetAngle(3, 383, 300)
                self.up.CDS_SetAngle(4, 196, 300)
                self.up.CDS_SetAngle(5, 459, 302)
                self.up.CDS_SetAngle(6, 190, 300)
                self.up.CDS_SetAngle(7, 375, 302)
                self.up.CDS_SetAngle(8, 189, 300)
                self.up.CDS_SetAngle(9, 512, 300)
                self.up.CDS_SetAngle(10, 712, 900)
                time.sleep(0.3)
        
                self.up.CDS_SetAngle(1, 340, 300)
                self.up.CDS_SetAngle(2, 195, 960)
                self.up.CDS_SetAngle(3, 449, 300)
                self.up.CDS_SetAngle(5, 459, 300)
                self.up.CDS_SetAngle(7, 375, 300)
                self.up.CDS_SetAngle(10, 312, 900)
                time.sleep(0.3)
        
                self.up.CDS_SetAngle(1, 440, 302)
                self.up.CDS_SetAngle(2, 195, 300)
                self.up.CDS_SetAngle(3, 549, 302)
                self.up.CDS_SetAngle(5, 359, 302)
                self.up.CDS_SetAngle(7, 675, 908)
                self.up.CDS_SetAngle(8, 512, 978)
                self.up.CDS_SetAngle(10, 712, 900)
                time.sleep(0.3)
        
                self.up.CDS_SetAngle(1, 440, 300)
                self.up.CDS_SetAngle(3, 549, 300)
                self.up.CDS_SetAngle(5, 359, 300)
                self.up.CDS_SetAngle(7, 675, 300)
                self.up.CDS_SetAngle(8, 189, 978)
                self.up.CDS_SetAngle(10, 312, 900)
                time.sleep(0.3)
        
                self.up.CDS_SetAngle(1, 540, 302)
                self.up.CDS_SetAngle(3, 649, 302)
                self.up.CDS_SetAngle(5, 659, 908)
                self.up.CDS_SetAngle(6, 512, 975)
                self.up.CDS_SetAngle(7, 623, 300)
                self.up.CDS_SetAngle(8, 189, 300)
                self.up.CDS_SetAngle(10, 712, 900)
                time.sleep(0.3)
        
                self.up.CDS_SetAngle(1, 540, 300)
                self.up.CDS_SetAngle(3, 649, 300)
                self.up.CDS_SetAngle(5, 659, 300)
                self.up.CDS_SetAngle(6, 190, 975)
                self.up.CDS_SetAngle(7, 575, 300)
                self.up.CDS_SetAngle(10, 312, 900)
                time.sleep(0.3)
        
                self.up.CDS_SetAngle(1, 640, 302)
                self.up.CDS_SetAngle(3, 349, 908)
                self.up.CDS_SetAngle(4, 512, 956)
                self.up.CDS_SetAngle(5, 559, 302)
                self.up.CDS_SetAngle(6, 190, 300)
                self.up.CDS_SetAngle(7, 475, 302)
                self.up.CDS_SetAngle(10, 712, 900)
                time.sleep(0.3)
            elif self.up_controller.io_data[0]==0 and self.up_controller.io_data[1]==0:    #后退
                self.up.ADC_Led_SetColor(0,0xF800)
                self.up.ADC_Led_SetColor(1,0XF800)
                self.up.CDS_SetAngle(1, 440, 512)
                self.up.CDS_SetAngle(2, 195, 512)
                self.up.CDS_SetAngle(3, 649, 512)
                self.up.CDS_SetAngle(4, 196, 512)
                self.up.CDS_SetAngle(5, 359, 512)
                self.up.CDS_SetAngle(6, 190, 1023)
                self.up.CDS_SetAngle(7, 675, 512)
                self.up.CDS_SetAngle(8, 189, 512)
                self.up.CDS_SetAngle(9, 512, 512)
                self.up.CDS_SetAngle(10, 512, 512)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 340, 302)
                self.up.CDS_SetAngle(2, 195, 300)
                self.up.CDS_SetAngle(3, 580, 300)
                self.up.CDS_SetAngle(4, 196, 300)
                self.up.CDS_SetAngle(5, 459, 302)
                self.up.CDS_SetAngle(6, 190, 300)
                self.up.CDS_SetAngle(7, 375, 908)
                self.up.CDS_SetAngle(8, 512, 978)
                self.up.CDS_SetAngle(9, 512, 300)
                self.up.CDS_SetAngle(10, 512, 300)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 340, 300)
                self.up.CDS_SetAngle(5, 459, 300)
                self.up.CDS_SetAngle(7, 375, 300)
                self.up.CDS_SetAngle(8, 189, 978)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 640, 908)
                self.up.CDS_SetAngle(2, 512, 960)
                self.up.CDS_SetAngle(3, 449, 396)
                self.up.CDS_SetAngle(5, 559, 302)
                self.up.CDS_SetAngle(7, 475, 302)
                self.up.CDS_SetAngle(8, 189, 300)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 640, 300)
                self.up.CDS_SetAngle(2, 195, 960)
                self.up.CDS_SetAngle(3, 449, 300)
                self.up.CDS_SetAngle(5, 559, 300)
                self.up.CDS_SetAngle(7, 475, 300)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 602, 300)
                self.up.CDS_SetAngle(2, 195, 300)
                self.up.CDS_SetAngle(3, 749, 908)
                self.up.CDS_SetAngle(4, 512, 956)
                self.up.CDS_SetAngle(5, 659, 302)
                self.up.CDS_SetAngle(7, 522, 300)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(3, 749, 300)
                self.up.CDS_SetAngle(4, 196, 956)
                self.up.CDS_SetAngle(5, 659, 300)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 440, 490)
                self.up.CDS_SetAngle(3, 649, 302)
                self.up.CDS_SetAngle(4, 196, 300)
                self.up.CDS_SetAngle(5, 359, 908)
                self.up.CDS_SetAngle(6, 512, 975)
                self.up.CDS_SetAngle(7, 675, 463)
                
                time.sleep(0.3)                
            elif self.up_controller.io_data[1]==0:                   #左转
                self.up.ADC_Led_SetColor(0,0xF800)
                self.up.ADC_Led_SetColor(1,0X0000)
                self.up.CDS_SetAngle(1, 540, 512)
                self.up.CDS_SetAngle(2, 195, 512)
                self.up.CDS_SetAngle(3, 749, 512)
                self.up.CDS_SetAngle(4, 196, 512)
                self.up.CDS_SetAngle(5, 459, 512)
                self.up.CDS_SetAngle(6, 190, 512)
                self.up.CDS_SetAngle(7, 375, 512)
                self.up.CDS_SetAngle(8, 189, 1023)
                self.up.CDS_SetAngle(9, 512, 512)
                self.up.CDS_SetAngle(10, 512, 512)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 640, 302)
                self.up.CDS_SetAngle(2, 195, 300)
                self.up.CDS_SetAngle(3, 449, 908)
                self.up.CDS_SetAngle(4, 512, 956)
                self.up.CDS_SetAngle(5, 559, 302)
                self.up.CDS_SetAngle(6, 190, 300)
                self.up.CDS_SetAngle(7, 475, 302)
                self.up.CDS_SetAngle(8, 189, 300)
                self.up.CDS_SetAngle(9, 512, 300)
                self.up.CDS_SetAngle(10, 512, 300)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 640, 300)
                self.up.CDS_SetAngle(3, 449, 300)
                self.up.CDS_SetAngle(4, 196, 956)
                self.up.CDS_SetAngle(5, 559, 300)
                self.up.CDS_SetAngle(7, 475, 300)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 340, 908)
                self.up.CDS_SetAngle(2, 512, 960)
                self.up.CDS_SetAngle(3, 549, 302)
                self.up.CDS_SetAngle(4, 196, 300)
                self.up.CDS_SetAngle(7, 675, 605)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 340, 300)
                self.up.CDS_SetAngle(2, 195, 960)
                self.up.CDS_SetAngle(3, 549, 300)
                self.up.CDS_SetAngle(7, 675, 300)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 440, 302)
                self.up.CDS_SetAngle(2, 195, 300)
                self.up.CDS_SetAngle(3, 649, 302)
                self.up.CDS_SetAngle(5, 359, 605)
                self.up.CDS_SetAngle(6, 512, 975)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 440, 300)
                self.up.CDS_SetAngle(3, 649, 300)
                self.up.CDS_SetAngle(5, 359, 300)
                self.up.CDS_SetAngle(6, 190, 975)
                
                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 540, 302)
                self.up.CDS_SetAngle(3, 749, 302)
                self.up.CDS_SetAngle(5, 459, 302)
                self.up.CDS_SetAngle(6, 190, 300)
                self.up.CDS_SetAngle(7, 375, 908)
                self.up.CDS_SetAngle(8, 512, 978)
                
                time.sleep(0.3) 
            else:                                 #右转
                self.up.ADC_Led_SetColor(0,0x0000)
                self.up.ADC_Led_SetColor(1,0XF800)
                self.up.CDS_SetAngle(1, 540, 512)
                self.up.CDS_SetAngle(2, 195, 1023)
                self.up.CDS_SetAngle(3, 749, 512)
                self.up.CDS_SetAngle(4, 196, 512)
                self.up.CDS_SetAngle(5, 459, 512)
                self.up.CDS_SetAngle(6, 190, 512)
                self.up.CDS_SetAngle(7, 375, 512)
                self.up.CDS_SetAngle(8, 189, 512)
                self.up.CDS_SetAngle(9, 512, 512)
                self.up.CDS_SetAngle(10, 512, 512)

                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 440, 302)
                self.up.CDS_SetAngle(2, 195, 300)
                self.up.CDS_SetAngle(3, 649, 302)
                self.up.CDS_SetAngle(4, 196, 300)
                self.up.CDS_SetAngle(5, 359, 302)
                self.up.CDS_SetAngle(6, 190, 300)
                self.up.CDS_SetAngle(7, 675, 908)
                self.up.CDS_SetAngle(8, 512, 978)
                self.up.CDS_SetAngle(9, 512, 300)
                self.up.CDS_SetAngle(10, 512, 300)

                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 440, 300)
                self.up.CDS_SetAngle(3, 649, 300)
                self.up.CDS_SetAngle(5, 359, 300)
                self.up.CDS_SetAngle(7, 675, 300)
                self.up.CDS_SetAngle(8, 189, 978)

                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 340, 302)
                self.up.CDS_SetAngle(3, 549, 302)
                self.up.CDS_SetAngle(5, 659, 908)
                self.up.CDS_SetAngle(6, 512, 975)
                self.up.CDS_SetAngle(7, 575, 302)
                self.up.CDS_SetAngle(8, 189, 300)

                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 340, 300)
                self.up.CDS_SetAngle(3, 549, 300)
                self.up.CDS_SetAngle(6, 190, 975)
                self.up.CDS_SetAngle(7, 575, 300)

                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 640, 908)
                self.up.CDS_SetAngle(2, 512, 960)
                self.up.CDS_SetAngle(3, 449, 302)
                self.up.CDS_SetAngle(5, 559, 302)
                self.up.CDS_SetAngle(6, 190, 300)
                self.up.CDS_SetAngle(7, 475, 302)

                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 640, 300)
                self.up.CDS_SetAngle(2, 195, 960)
                self.up.CDS_SetAngle(3, 449, 300)
                self.up.CDS_SetAngle(5, 559, 300)
                self.up.CDS_SetAngle(7, 475, 300)

                time.sleep(0.3)
                self.up.CDS_SetAngle(1, 540, 302)
                self.up.CDS_SetAngle(2, 195, 300)
                self.up.CDS_SetAngle(3, 749, 908)
                self.up.CDS_SetAngle(4, 512, 956)
                self.up.CDS_SetAngle(5, 459, 302)
                self.up.CDS_SetAngle(7, 375, 302)

                time.sleep(0.3) 

if __name__ == '__main__':
    spider = SPIDER()
    spider.start() 
    
    
    
            

    





