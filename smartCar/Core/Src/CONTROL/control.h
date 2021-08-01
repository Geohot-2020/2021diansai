#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
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


#define PI 3.14159265
void EXTI9_5_IRQHandler(void);
int balance_UP(float Angle,float Mechanical_balance,float Gyro);
int velocity(int encoder_left,int encoder_right,int target);
int turn(int encoder_left,int encoder_right,float gyro,int target);
int seek_Beacon(uint8_t x);
#endif
