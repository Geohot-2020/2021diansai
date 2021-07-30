#ifndef __SYS_H
#define __SYS_H	 
/////////////////////////////////////////////////////////////
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "oled.h"
#include "gpio.h"
#include "motor.h"
#include "control.h"
#include "tim.h"
#include "i2c.h"
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stm32f1xx_it.h"
#include "exti.h"
/////////////////////////////////////////////////////////////
#include <string.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//////////////////////////////////////////////////////////////
extern float Voltage;                        //电池电压
extern float pitch,roll,yaw;                 //欧拉角
extern short aacx,aacy,aacz;                 //加速度传感器原始数据
extern short gyrox,gyroy,gyroz;              //陀螺仪原始数据
extern uint32_t enc1,enc2;                   //左右编码器的脉冲计数
extern int Moto1,Moto2;           //计算出来的最终结果赋值给电机的PWM
extern int Balance_Pwm;
extern float Mechanical_angle;
#endif
