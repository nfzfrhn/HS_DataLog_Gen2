/**
  ******************************************************************************
  * @file    mp23abs1_app.h
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   Header for mp23abs1_app.c module.
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
#ifndef __MP23ABS1_APP_H
#define __MP23ABS1_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h" 
#include "main.h"
#include "cmsis_os.h"
#include "sensors_manager.h"

#define AMIC_SAMPLING_FREQUENCY                 (uint32_t)(192000)
#define IMP34DT05_SAMPLING_FREQUENCY                 (uint32_t)(48000)
#define AMIC_MS                                 (uint32_t)(1)
#define IMP34DT05_MS                                 (uint32_t)(1)
#define FFT_LEN_DMIC                            (uint32_t)(1024)
#define FFT_LEN_AMIC                            (uint32_t)(1024)
#define OVLP                                    (float)(0.25)
  
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

extern DMA_HandleTypeDef hdma_dfsdm1_flt1;

void MP23ABS1_Peripheral_Init(void);
void MP23ABS1_OS_Init(void);
void MP23ABS1_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp);
void MP23ABS1_Set_ODR(float newODR);
void MP23ABS1_Set_FS(float newFS1, float newFS2);
void MP23ABS1_Start(void);
void MP23ABS1_Stop(void);


#ifdef __cplusplus
}
#endif

#endif /* __MP23ABS1_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
