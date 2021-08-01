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
#include "adc.h"
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
extern int Moto1,Moto2;                      //计算出来的最终结果赋值给电机的PWM
extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Seek_Pwm;                //直立,速度，转向,差速循迹PWM
extern float Mechanical_angle;
extern uint16_t Cx,Cy,Cw,Ch; 
extern uint8_t acom_data;
extern uint8_t Uart2_RxCounter1;
extern uint16_t Uart2_RxBuffer1[10];
extern uint8_t Uart2_RxState;	
extern uint8_t Uart2_RxFlag1;
extern uint16_t ADC_Value;
extern uint8_t Light_Sens; 
#endif
///////////////////////////////////////////////////////////////

/**********************************文档注释***********************************
 * oled模块         --> oled.c
 * 电机模块         --> motor.c
 * 编码器模块       --> timc
 * 核心控制(PID)模块--> control.c
 * openmv模块      -->  usart.c
 * mpu6050模块     -->  mpu6050.c/inv_mpu.c/IIC.c/inv_mpu_dmp_motion_drive.c
 * 外部中断模块     --> gpio.c
******************************************************************************/

/*******************************中断优先级配置***********************************
 * TIM3:            P:0     S:0
 * USART1:          P:2     S:0
 * USART2:          P:2     S:0
 * 外部中断：        P:1     S:0
******************************************************************************/