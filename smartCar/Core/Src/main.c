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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
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

void OLED_show(void);
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
int Balance_Pwm=0,Velocity_Pwm=0,Turn_Pwm=0,Seek_Pwm=0;                //直立,速度，转向,差速循迹PWM
float Mechanical_angle=0;                             //机械中值
uint16_t ADC_Value=0;                                 //PA4 ADC1_IN5 AD采样值
uint8_t Light_Sens=0;                                 //0-100：0，最暗；100，最亮
/* OEPNMV用 */
///////////////////////////////////
uint8_t acom_data;                                    //每位接收的数据            
uint8_t Uart2_RxCounter1=0;                           //接收缓冲计数
uint16_t Uart2_RxBuffer1[10]={0};                     //接收缓冲数组
uint8_t Uart2_RxState = 0;	                          //接收状态||1：第一个帧头0x2C；2：第二个帧头0x12；3：帧尾0x5B
uint8_t Uart2_RxFlag1 = 0;                            //接收状态||成功1，失败0

uint16_t Cx=160,Cy=0,Cw=0,Ch=0;                       //采集的图像信息
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  ///////////////////////////////////////////初始化///////////////////////////////////////////////////
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);                     //===使能PWM输出
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);                //===打开encoder
  Motor_Init();                                                 //===电机初始化
  MPU_Init();                                                   //===MPU6050初始化
  mpu_dmp_init();                                               //===dmp初始化
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);       //===openmv接收串口初始化
  HAL_ADCEx_Calibration_Start(&hadc1);                          //===AD校准
  OLED_init();                                                  //===OLED初始化
  HAL_TIM_Base_Start_IT(&htim3);                                //===打开核心控制
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  OLED_operate_gram(PEN_CLEAR);
  OLED_refresh_gram();	                                        //===oled清屏，刷屏
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
    /* ADC */
    HAL_ADC_Start(&hadc1);                  //启动ADC转换
    HAL_ADC_PollForConversion(&hadc1,50);   //等待转换完成，50为最大等待时间，单位为ms
    /* oled */
    OLED_show();
    HAL_Delay(20);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* 备用PB5外部中断，以防定时器资源不足 */

// void EXTI9_5_IRQHandler(void) 
// {
//   if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5))
//   {
//     EXTI->PR=1<<5;                    //===清除中断位标志
//     printf("\r\ntest_EXIT5\r\n");     

//   }
// }



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
