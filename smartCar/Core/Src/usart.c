/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "sys.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */
  
  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

/*********************************************************************************
 * 函数功能： openmv串口中断
 * 入口参数： 串口名，usart2
 * 返回值：   无
**********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */


    // printf("\n\r%d\r\n",acom_data);
    // HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断
    uint8_t i;
    
    if(Uart2_RxState==0&&acom_data==0x2C)   //0X2c帧头
    {
      Uart2_RxState=1;
      Uart2_RxBuffer1[Uart2_RxCounter1++] = acom_data;
      // printf("0x2c: %x  ",acom_data);
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断
      
    }
    else if(Uart2_RxState==1&&acom_data==0x12)    //0x12帧头
    {
      Uart2_RxState=2;
      Uart2_RxBuffer1[Uart2_RxCounter1++] = acom_data;
      // printf("0x12: %x  ",acom_data);
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断
      
    }
    else if(Uart2_RxState==2)
    {
      Uart2_RxBuffer1[Uart2_RxCounter1++] = acom_data;
      // printf("ex: %x  ",acom_data);
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断

      if(Uart2_RxCounter1>=10||acom_data == 0x5B)   //接收数据满了，接收数据结束
      {
          Uart2_RxState=3;
          Uart2_RxFlag1=1;
          Cx=Uart2_RxBuffer1[Uart2_RxCounter1-5];
          Cy=Uart2_RxBuffer1[Uart2_RxCounter1-4];
          Cw=Uart2_RxBuffer1[Uart2_RxCounter1-3];
          Ch=Uart2_RxBuffer1[Uart2_RxCounter1-2];
          Cx=Cx+Cw;
          // printf("ok: %x  ",acom_data);
          HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断

      }
    }

    else if(Uart2_RxState==3)   //检查是否接收到结束标志
    {
      if(Uart2_RxBuffer1[Uart2_RxCounter1-1] == 0x5B)
      {

        ///////////////////////////////////test/////////////////////////
        if(Uart2_RxFlag1)
        {
          printf("cx:%d, cy:%d",Cx,Cy);
        // printf("aab");
        }
        ///////////////////////////////////////////////////////////////
        Uart2_RxFlag1=0;
        Uart2_RxCounter1=0;
        Uart2_RxState=0;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断
      }
      else  //接收错误
      {
        Uart2_RxState=0;
        Uart2_RxCounter1=0;
        for(i=0;i<10;i++)
          Uart2_RxBuffer1[i]=0x00;  //清零存放数据数组
        printf("error1");
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断
      }
    }
    
    else  //接收异常
    {
      Uart2_RxState=0;
      Uart2_RxCounter1=0;
      for(i=0;i<10;i++)
        Uart2_RxBuffer1[i]=0x00;  //清零存放数据数组
      printf("error2");
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&acom_data, 1);   //再开启接收中断
    }



  
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
