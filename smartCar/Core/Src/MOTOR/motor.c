#include "motor.h"

/***************************************************************
 * 程序功能：电机基础控制
 * atuthor：zhengyc
 * date：2021/7/28
*****************************************************************/

void Motor_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
}

/****************************************************************
 * 函数功能：赋值给PWM寄存器
 * 入口参数：左轮PWM，右轮PWM
 * 返回值：无
****************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	if(moto1<0){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
    }else{
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
    }
    TIM1->CCR1 = myabs(moto1);

    if(moto2<0){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
    }else{
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
    }
    TIM1->CCR4 = myabs(moto2);
}


/****************************************************************
 * 函数功能：绝对值函数
 * 入口参数：int
 * 返回值：unsigned int
****************************************************************/
int myabs(int a)
{
    int temp;
    if(a<0)  temp=-a;  
    else temp=a;
    return temp;
}


/****************************************************************
 * 函数功能：限制PWM赋值
 * 入口参数：无
 * 返回值：无
****************************************************************/
void Lock_Pwm(void)
{
    //===PWM满幅是7200 限制在7000
    if(Moto1<-7000 ) Moto1=-7000 ;
    if(Moto1>7000 )  Moto1=7000 ;
    if(Moto2<-7000 ) Moto2=-7000 ;
    if(Moto2>7000 )  Moto2=7000 ;
}


/****************************************************************
 * 函数功能：异常关闭电机
 * 入口参数：倾角和电压
 * 返回值：无
****************************************************************/
void Turn_Off(float angle, float voltage)
{
    if(angle<-40||angle>40||voltage<11.1)	 //电池电压低于11.1V关闭电机
    {	                                   //===倾角大于40度关闭电机																			 
        Moto1=0;
        Moto2=0;
    }		
}
	