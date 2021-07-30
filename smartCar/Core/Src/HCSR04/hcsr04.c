#include "hcsr04.h"
#include "gpio.h"
#include "delay.h"
#include "usart.h"
 
#define HCSR04_PORT     GPIOA
#define HCSR04_TRIG     LL_GPIO_PIN_3
#define ECHO_Reci  LL_GPIO_IsInputPinSet(HCSR04_PORT,LL_GPIO_PIN_2)
 
 
void TRIG_pulse()
{
    LL_GPIO_SetOutputPin(HCSR04_PORT,HCSR04_TRIG);
    Delay_us(30);
    LL_GPIO_ResetOutputPin(HCSR04_PORT,HCSR04_TRIG);
}
 
float HCSR04_GetLength(void)
{
	uint32_t startval,endval,tickn,ticknend,delayt;
	double delays;
	uint32_t timeout = 23530;//uS
	int i = 0;
	float lengthTemp = 0;
	float sum = 0;
	delays = 0;
 
	while(i!=5)
	{
		TRIG_pulse();
		startval = SysTick->VAL;
		tickn = HAL_GetTick();
		while(ECHO_Reci == 0)      //等待接收口高电平输出
		{
			endval = SysTick->VAL;//读取系统滴答值
			ticknend = HAL_GetTick();
			if(ticknend == tickn)
			{
				if(startval > endval)
				{
					delayt = (startval - endval);
				}
				else//系统滴答进入下一毫秒
				{
					delayt = (startval +72000 - endval);
				}
			}
			else
			{
				delayt = ((ticknend - tickn) * 72000 + startval - endval);
			}
 
			if(delayt > 360000)//时间超时
			{
				printf("s%d,e%d;t%d,d%d\r\n",startval,endval,tickn,ticknend);
				return delayt;
			}
 
		}
 
		startval = SysTick->VAL;//跳出上一循环意味着Echo来了上升沿，读取此时滴答（时间）
		tickn = HAL_GetTick();
 
			i = i + 1;
 
		while(ECHO_Reci == 1)
			{
				endval = SysTick->VAL;
				ticknend = HAL_GetTick();
				if(ticknend == tickn)
				{
					if(startval > endval)
					{
						delays = (startval - endval)/72.0;
					}
					else
					{
						delays = (startval +72000 - endval)/72.0;
					}
				}
 
				else
				{
					delays = ((ticknend - tickn) * 72000 + startval - endval)/72.0;
				}
 
				if(delays > timeout)//超时时间
				{
					printf("s%d,e%d;t%d,d%d\r\n",startval,endval,tickn,ticknend);
					return delays;
				}
 
			}
		endval = SysTick->VAL;//跳出上一循环意味着Echo来了下降沿，读取此时滴答（时间）
		ticknend = HAL_GetTick();
		delays = ((ticknend - tickn) * 72000 + startval - endval)/72;//计算时间差
 
		lengthTemp = ((float)delays/58.0);//单位cm
		sum = lengthTemp + sum ;
	}
 
	lengthTemp = sum/5.0;
	return lengthTemp;
}