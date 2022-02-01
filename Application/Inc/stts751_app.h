/**
  ******************************************************************************
  * @file    stts751_app.h
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   Header for stts751_app.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STTS751_APP_H
#define __STTS751_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h" 
#include "cmsis_os.h"
#include "sensors_manager.h"
#include "stts751_reg.h"

#define STTS751_INT_Pin                  GPIO_PIN_15
#define STTS751_INT_GPIO_Port            GPIOE
#define STTS751_INT_EXTI_IRQn            EXTI15_10_IRQn
#define STTS751_INT_EXTI_LINE            EXTI_LINE_15
#define STTS751_INT_GPIO_ADDITIONAL()    
#define STTS751_INT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()

extern EXTI_HandleTypeDef stts751_exti;

void STTS751_Peripheral_Init(void);
void STTS751_OS_Init(void);
void STTS751_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp);
void STTS751_Set_ODR(float newODR);
void STTS751_Set_FS(float newFS1, float newFS2);
void STTS751_Start(void);
void STTS751_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __STTS751_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
