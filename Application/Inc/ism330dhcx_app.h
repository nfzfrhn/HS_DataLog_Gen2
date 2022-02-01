/**
  ******************************************************************************
  * @file    ism330dhcx_app.h
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   Header
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
#ifndef __ISM330DHCX_H
#define __ISM330DHCX_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32l4xx_hal.h" 
#include "main.h"
#include "cmsis_os.h"
#include "ism330dhcx_reg.h"

#define ISM330DHCX_GY_SAMPLES_PER_IT         (256)
#define ISM330DHCX_AX_SAMPLES_PER_IT         (0)
#define ISM330DHCX_WTM_LEVEL                 (ISM330DHCX_GY_SAMPLES_PER_IT)

#define ISM330DHCX_SPI_CS_Pin                GPIO_PIN_13
#define ISM330DHCX_SPI_CS_GPIO_Port          GPIOF
#define ISM330DHCX_SPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()

#define ISM330DHCX_INT1_Pin                  GPIO_PIN_8
#define ISM330DHCX_INT1_GPIO_Port            GPIOE
#define ISM330DHCX_INT1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()
#define ISM330DHCX_INT1_EXTI_IRQn            EXTI9_5_IRQn
#define ISM330DHCX_INT1_EXTI_LINE            EXTI_LINE_8

extern EXTI_HandleTypeDef ism330dhcx_exti;

void ISM330DHCX_Peripheral_Init(void);
void ISM330DHCX_OS_Init(void);
void ISM330DHCX_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp);
void ISM330DHCX_Set_ODR(float newODR);
void ISM330DHCX_Set_FS(float newFS1, float newFS2);
void ISM330DHCX_Start(void);
void ISM330DHCX_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __ISM330DHCX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
