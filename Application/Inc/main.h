/**
  ******************************************************************************
  * @file    main.h
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   This file contains all the functions prototypes for the main.c
  *          file.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"
#include "usbd_desc.h"
#include "usbd_wcid_streaming.h"
#include "usbd_wcid_interface.h"
#include "sensors_manager.h"
#include "com_manager.h"
  
#include "stwin.h"
#include "cmsis_os.h"
#include "sdcard_manager.h"

#include "arm_math.h"

#include "iis3dwb_app.h"


#include "mp23abs1_app.h"
#include "imp34dt05_app.h"
#include "ism330dhcx_app.h"
#include "iis2mdc_app.h"
#include "iis2dh_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

#include "rtc.h"



//#include "debugPins.h"
#include "STWIN_debug_pins.h"

#include "freeRTOS.h"
  
#include "HSD_json.h"
#include "device_description.h"  

#ifndef M_PI   
  #define M_PI   3.14159265358979323846264338327950288
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define SD_THREAD_PRIO                osPriorityBelowNormal
#define I2C_RD_THREAD_PRIO            osPriorityNormal
#define SPI_RD_THREAD_PRIO            osPriorityNormal
#define IIS3DWB_THREAD_PRIO           osPriorityNormal
#define IIS2MDC_THREAD_PRIO           osPriorityNormal
#define IIS2DH_THREAD_PRIO            osPriorityNormal
#define STTS751_THREAD_PRIO           osPriorityNormal
#define LPS22HH_THREAD_PRIO           osPriorityNormal
#define HTS221_THREAD_PRIO            osPriorityNormal
#define IMP34DT05_THREAD_PRIO         osPriorityNormal
#define MP23ABS1_THREAD_PRIO          osPriorityNormal
#define ISM330DHCX_THREAD_PRIO        osPriorityNormal

/* 
Set configUSE_APPLICATION_TASK_TAG to 1 in FreeRTOSConfig.h to enable the Task debugging mode.
Each time a task is executing the corresponding pin is SET otherwise is RESET
 */

// IDLE Task Pin cannot be changed
#define TASK_IDLE_DEBUG_PIN           DEBUG_PIN7
#define TASK_IIS3DWB_DEBUG_PIN        DEBUG_PIN8
#define TASK_SDM_DEBUG_PIN            DEBUG_PIN9
#define TASK_IIS2MDC_DEBUG_PIN        DEBUG_PIN10
#define TASK_STTS751_DEBUG_PIN        DEBUG_PIN11
#define TASK_LPS22HH_DEBUG_PIN        DEBUG_PIN12
#define TASK_HTS221_DEBUG_PIN         DEBUG_PIN13
#define TASK_IMP34DT05_DEBUG_PIN      DEBUG_PIN14
#define TASK_MP23ABS1_DEBUG_PIN       DEBUG_PIN17
#define TASK_ISM330DHCX_DEBUG_PIN     DEBUG_PIN18
#define TASK_SM_SPI_DEBUG_PIN         DEBUG_PIN19
#define TASK_SM_I2C_DEBUG_PIN         DEBUG_PIN20
 

/* Exported macro ------------------------------------------------------------*/  
#define USART2_RX_Pin GPIO_PIN_6
#define USART2_RX_GPIO_Port GPIOD
#define USART2_RTS_Pin GPIO_PIN_4
#define USART2_RTS_GPIO_Port GPIOD
#define USART2_TX_Pin GPIO_PIN_5
#define USART2_TX_GPIO_Port GPIOD
//#define DATA_TEST     /* Uncomment to use fake data for debugging */
  
//#define LOG_ERROR     /* Uncomment to save a 'log_error.txt' file on the SD_Card */  
  
//#define HSD_SD_LOGGING_MODE     HSD_SD_LOGGING_MODE_CONTINUOUS
#define HSD_SD_LOGGING_MODE 	HSD_SD_LOGGING_MODE_INTERMITTENT		// Gärprozess sensor

/* Memory management macros */   
#define HSD_malloc               malloc
#define HSD_calloc               calloc
#define HSD_free                 free
#define HSD_memset               memset
#define HSD_memcpy               memcpy
  
/* Exported functions ------------------------------------------------------- */
/* Includes ------------------------------------------------------------------*/

#include "usbd_def.h"


/** USB device core handle. */
extern USBD_HandleTypeDef USBD_Device;


/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);
  
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
