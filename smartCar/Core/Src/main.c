/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RXBUFFERSIZE  256
char RxBuffer[RXBUFFERSIZE]; 



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// void EXTI9_5_IRQHandler(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*****************************全局变量******************************************/
float Voltage;                                        //电池电压
float pitch,roll,yaw;                                 //欧拉角
short aacx,aacy,aacz;                                 //加速度传感器原始数据
short gyrox,gyroy,gyroz;                              //陀螺仪原始数据
uint32_t enc1,enc2;                                   //左右编码器的脉冲计数
int Moto1=0,Moto2=0;                                  //计算出来的最终结果赋值给电机的PWM
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;                //直立,速度，转向PWM
float Mechanical_angle=0;                             //机械中值

/* OEPNMV用 */
///////////////////////////////////
uint8_t com_data; 
uint8_t acom_data;

static uint8_t Uart2_RxCounter1=0;
static uint16_t Uart2_RxBuffer1[10]={0};
static uint8_t Uart2_RxState = 0;	
static uint8_t Uart2_RxFlag1 = 0;

static uint8_t Cx=0,Cy=0,Cw=0,Ch=0;
////////////////////////////////////


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //使能PWM输出
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);  //打开encoder
  Motor_Init();
  MPU_Init();   //MPU6050
  mpu_dmp_init();   //dmp初始化
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);
  OLED_init();  //OLED
  HAL_TIM_Base_Start_IT(&htim3);    //打开核心控制
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // HAL_Delay(10);
    
    // enc1=-Read_Encoder(2);
    // enc2=Read_Encoder(4);
		OLED_operate_gram(PEN_CLEAR);
		OLED_printf(0,0,"encoderL: %d",enc1);	
		OLED_printf(1,0,"encoderR: %d",enc2);	
    OLED_printf(2,0,"Cx: %d,Cy: %d",Cx,Cy);
    OLED_printf(3,0,"pitch: %f",pitch);
    
    // Balance_Pwm=balance_UP(pitch, Mechanical_angle, gyroy);     //直立环PID控制
    // Moto1=Balance_Pwm;
    // Moto2=Balance_Pwm;      //计算左右轮电机最终PWM
    // Lock_Pwm();             //限幅
    // Set_Pwm(Moto1,Moto2);   //赋值驱动
    // printf("\n\rax:%d, ay:%d, az:%d\r\n",aacx,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  aacy,aacz);
    // printf("\n\rgx:%d, gy:%d, gz:%d\r\n",gyrox,gyroy,gyroz);
    

    OLED_refresh_gram();	//oled
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == (&htim3))
  {
    
    mpu_dmp_get_data(&pitch, &roll, &yaw);	//必须要用while等待，才能读取成功
		MPU_Get_Accelerometer(&aacx,&aacy, &aacz);		//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);		//得到陀螺仪数据
    printf("\n\r%f\r\n", pitch);							//串口1输出pitch信息
    enc1=-Read_Encoder(2);
    enc2=Read_Encoder(4);
    printf("\n\renc1: %d, enc2: %d\r\n",enc1,enc2);
    Balance_Pwm=balance_UP(pitch,Mechanical_angle,gyroy);

    Moto1=Balance_Pwm;
    Moto2=Balance_Pwm;
    Lock_Pwm();
    Set_Pwm(Moto1,Moto2);
  }

}


// void EXTI9_5_IRQHandler(void) 
// {
//   if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5))
//   {
//     EXTI->PR=1<<5;                    //===清除中断位标志
//     printf("\r\ntest_EXIT5\r\n");     

//   }
// }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
    
    if(Uart2_RxState==0&&acom_data==0x2C)   //0X2c帧头
    {
      Uart2_RxState=1;
      Uart2_RxBuffer1[Uart2_RxCounter1++] = acom_data;
    }
    else if(Uart2_RxState==1&&acom_data==0x12)    //0x12帧头
    {
      Uart2_RxState=2;
      Uart2_RxBuffer1[Uart2_RxCounter1++] = acom_data;
    }
    else if(Uart2_RxState==2)
    {
      Uart2_RxBuffer1[Uart2_RxCounter1++] = acom_data;
      
      if(Uart2_RxCounter1>=10||acom_data == 0x5B)   //接收数据满了，接收数据结束
      {
          Uart2_RxState=3;
          Uart2_RxFlag1=1;
          Cx=Uart2_RxBuffer1[Uart2_RxCounter1-5];
          Cy=Uart2_RxBuffer1[Uart2_RxCounter1-4];
          Cw=Uart2_RxBuffer1[Uart2_RxCounter1-3];
          Ch=Uart2_RxBuffer1[Uart2_RxCounter1-2];

      }
    }

    else if(Uart2_RxState==3)   //检查是否接收到结束标志
    {
      if(Uart2_RxBuffer1[Uart2_RxCounter1-1] == 0x5B)
      {
        Uart2_RxFlag1=0;
        Uart2_RxCounter1=0;
        Uart2_RxState=0;

        HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断
      }
    }


  
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
