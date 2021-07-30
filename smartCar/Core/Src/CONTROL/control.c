#include "control.h"
/**************************************************
 * 程序功能：核心控制
 * author: zhengyc
 * date: 2020/7/28
***************************************************/


//直立环
float balance_UP_KP=300;    
float balance_UP_KD=1.9;



/**************************************************
 * 函数功能： 直立PD控制
 * 入口参数： 角度、机械平衡角度、角速度
 * 返回值： 直立控制PWM
***************************************************/
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{
    float Bias;
    int balance;
    Bias=Angle-Mechanical_balance;                  //===求出平衡的角度中值和机械相关
    balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===求出平衡控制的PWM PD控制
    return balance;
}