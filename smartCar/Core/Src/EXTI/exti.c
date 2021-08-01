#include "exti.h"

// void MPU6050_EXTI_Init(void)
// {
//   LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
//     /**/
//   LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE5);

//   /**/
//   EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_5;
//   EXTI_InitStruct.LineCommand = ENABLE;
//   EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
//   EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
//   LL_EXTI_Init(&EXTI_InitStruct);

//   /**/
//   LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);

//   /**/
//   LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT);

//   /* EXTI interrupt init*/
//   NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
//   NVIC_EnableIRQ(EXTI9_5_IRQn);
// }
