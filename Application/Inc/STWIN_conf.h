/**
******************************************************************************
* @file    STWIN_conf.h
* @author  SRA - Central Labs
* @version v2.1.1
* @date    26-Feb-2020
* @brief   This file contains definitions for the components bus interfaces
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the 
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STWIN_CONF_H__
#define __STWIN_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "STWIN_bus.h"
#include "STWIN_errno.h"

/* Battery Charger */

/* Define 1 to use already implemented callback; 0 to implement callback
   into an application file */ 
   
#define USE_BC_TIM_IRQ_CALLBACK         1U  
#define USE_BC_GPIO_IRQ_HANDLER         1U  
#define USE_BC_GPIO_IRQ_CALLBACK        0U 
  
/* stts751 */  
#define BSP_STTS751_INT_GPIO_CLK_ENABLE() __GPIOE_CLK_ENABLE()
#define BSP_STTS751_INT_PORT GPIOE
#define BSP_STTS751_INT_PIN GPIO_PIN_15  
#define BSP_STTS751_INT_EXTI_IRQn           EXTI15_10_IRQn  

  
#ifdef __cplusplus
}
#endif

#endif /* __STWIN_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

