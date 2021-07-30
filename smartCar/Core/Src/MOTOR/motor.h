#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	

#define PWMA TIM1->CCR1     //PA8

#define PWMB TIM1->CCR4     //PA11

void Motor_Init(void);
void Set_Pwm(int moto1,int moto2);
int myabs(int a);
void Lock_Pwm(void);
void Turn_Off(float angle, float voltage);

#endif
