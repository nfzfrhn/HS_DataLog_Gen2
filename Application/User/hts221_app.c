/**
  ******************************************************************************
  * @file    hts221_app.c
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   This file provides a set of functions to handle hts221 sensor
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

/* Includes ------------------------------------------------------------------*/
#include "hts221_app.h"
/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

SM_Init_Param_t HTS221_Init_Param;
SM_Sensor_State_t HTS221_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
static osSemaphoreId hts221_data_ready_sem_id;
static osSemaphoreDef(hts221_data_ready_sem);

/* Semaphore used to wait on BUS data read complete, managed by "sensors manager" */
static osSemaphoreId hts221_data_read_cmplt_sem_id;
static osSemaphoreDef(hts221_data_read_cmplt_sem);

static sensor_handle_t hts221_hdl_instance = {HTS221_ID, HTS221_I2C_ADDRESS, NULL, 0, &hts221_data_read_cmplt_sem_id};
static stmdev_ctx_t hts221_ctx_instance = {SM_I2C_Write_Os, SM_I2C_Read_Os, &hts221_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId HTS221_Thread_Id;
static void HTS221_Thread(void const *argument);

EXTI_HandleTypeDef hts221_exti;
static void HTS221_Int_Callback(void);

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_hts221 = 0;


/**
* @brief GPIO Initialization Function, initialize CS and IT pins
* @param None
* @retval None
*/
void HTS221_Peripheral_Init(void)
{  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  HTS221_INT_GPIO_ADDITIONAL();
  HTS221_INT_GPIO_CLK_ENABLE();
  /*Configure GPIO pins : STTS751_INT_Pin hts221_INT1_Pin */
  GPIO_InitStruct.Pin =  HTS221_INT_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HTS221_INT_GPIO_Port, &GPIO_InitStruct);  
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(HTS221_INT_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(HTS221_INT_EXTI_IRQn);
  HAL_EXTI_GetHandle(&hts221_exti, HTS221_INT_EXTI_LINE);  
  HAL_EXTI_RegisterCallback(&hts221_exti,  HAL_EXTI_COMMON_CB_ID, HTS221_Int_Callback);
}

/**
* @brief HTS221 Threads Creation
* @param None
* @retval None
*/
void HTS221_OS_Init(void)
{  
  /* Data read complete semaphore initialization */  
  hts221_data_read_cmplt_sem_id = osSemaphoreCreate(osSemaphore(hts221_data_read_cmplt_sem), 1);
  osSemaphoreWait(hts221_data_read_cmplt_sem_id,osWaitForever);
  
  /* Data ready interrupt semaphore initialization */  
  hts221_data_ready_sem_id = osSemaphoreCreate(osSemaphore(hts221_data_ready_sem), 1);
  osSemaphoreWait(hts221_data_ready_sem_id,  osWaitForever);
  
  /* Thread definition: read data */  
  osThreadDef(HTS221_Acquisition_Thread, HTS221_Thread, HTS221_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);  
  /* Start thread */
  HTS221_Thread_Id = osThreadCreate(osThread(HTS221_Acquisition_Thread), NULL); 
  /* Suspend thread */
  osThreadSuspend(HTS221_Thread_Id);
}

volatile float tout_dbg = 0.0f;
volatile float hout_dbg = 0.0f;
static void HTS221_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_HTS221_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_HTS221_DEBUG_PIN );
#endif
  
#ifdef DATA_TEST
  static uint16_t usbTestData = 0;
#endif
  
  float x0_t=0, y0_t=0, x1_t=0, y1_t=0;
  float x0_h=0, y0_h=0, x1_h=0, y1_h=0;
  hts221_axis1bit16_t data_raw_humidity;
  hts221_axis1bit16_t data_raw_temperature;
  hts221_axis1bit16_t coeff;
  
  for (;;)
  {    
    /* Read raw data (roughly) each 1 s */    
    if (HTS221_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {  
      uint8_t reg0;
      
      ts_hts221 = 0;
      tim_value_old = 0;
      
      hts221_device_id_get( &hts221_ctx_instance, (uint8_t *)&reg0);  
      hts221_power_on_set(&hts221_ctx_instance, PROPERTY_DISABLE);
      hts221_boot_set(&hts221_ctx_instance, PROPERTY_ENABLE);
      
      /* Set BDU*/  
      hts221_block_data_update_set(&hts221_ctx_instance, PROPERTY_ENABLE);
      /* Enable Interrupt */
      hts221_drdy_on_int_set(&hts221_ctx_instance, PROPERTY_ENABLE); 
      /* Set Data Rate */
      
      if(HTS221_Init_Param.ODR < 2.0f)
        hts221_data_rate_set(&hts221_ctx_instance, HTS221_ODR_1Hz);
      else if(HTS221_Init_Param.ODR < 8.0f)
        hts221_data_rate_set(&hts221_ctx_instance, HTS221_ODR_7Hz);
      else if(HTS221_Init_Param.ODR < 13.0f)
        hts221_data_rate_set(&hts221_ctx_instance, HTS221_ODR_12Hz5);      
      
      /* Get calibration values (only first time) */      
      hts221_temp_adc_point_0_get(&hts221_ctx_instance, coeff.u8bit);
      x0_t = (float)coeff.i16bit;  
      
      hts221_temp_deg_point_0_get(&hts221_ctx_instance, coeff.u8bit);
      y0_t = (float)coeff.u8bit[0];
      
      hts221_temp_adc_point_1_get(&hts221_ctx_instance, coeff.u8bit);
      x1_t = (float)coeff.i16bit;
      
      hts221_temp_deg_point_1_get(&hts221_ctx_instance, coeff.u8bit);
      y1_t = (float)coeff.u8bit[0];
      
      hts221_hum_adc_point_0_get(&hts221_ctx_instance, coeff.u8bit);
      x0_h = (float)coeff.i16bit;  
      
      hts221_hum_rh_point_0_get(&hts221_ctx_instance, coeff.u8bit);
      y0_h = (float)coeff.u8bit[0];
      
      hts221_hum_adc_point_1_get(&hts221_ctx_instance, coeff.u8bit);
      x1_h = (float)coeff.i16bit;
      
      hts221_hum_rh_point_1_get(&hts221_ctx_instance, coeff.u8bit);
      y1_h = (float)coeff.u8bit[0]; 
      
      /* Power Up */
      hts221_power_on_set(&hts221_ctx_instance, PROPERTY_ENABLE);       
            
      HTS221_Sensor_State = SM_SENSOR_STATE_RUNNING;
      hts221_temperature_raw_get(&hts221_ctx_instance, data_raw_temperature.u8bit);
      hts221_humidity_raw_get(&hts221_ctx_instance, data_raw_humidity.u8bit);
    }
    else if(HTS221_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      float dataOut[2];
      osSemaphoreWait(hts221_data_ready_sem_id,  osWaitForever);
      
      if(HTS221_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {      
        
      if(tim_value >= tim_value_old)
      {
        period = tim_value - tim_value_old;
      }
      else
      {
        period = tim_value + (0xFFFFFFFF - tim_value_old);
      }
      
      tim_value_old = tim_value;
      ts_hts221 +=  period;
      
      hts221_temperature_raw_get(&hts221_ctx_instance, data_raw_temperature.u8bit);
      /* Apply calibration */ /* To be optimized eventually */   
      dataOut[0] = (((y1_t - y0_t) * (float)(data_raw_temperature.i16bit)) + ((x1_t * y0_t) - (x0_t * y1_t))) / (x1_t - x0_t);     
      /*Just to see the value in the debug window:*/
      tout_dbg = dataOut[0];      
      
      hts221_humidity_raw_get(&hts221_ctx_instance, data_raw_humidity.u8bit);
      /* Apply calibration */ /* To be optimized eventually */   
      dataOut[1] = (((y1_h - y0_h) * (float)(data_raw_humidity.i16bit)) + ((x1_h * y0_h) - (x0_h * y1_h))) / (x1_h - x0_h);     
      /*Just to see the value in the debug window:*/
      hout_dbg = dataOut[1];

      
#ifdef DATA_TEST
      uint16_t i = 0;

      int16_t * p16 = (int16_t *)dataOut;      
      for (i = 0; i < 4; i++)    
      {          
        *p16++ = usbTestData++;        
      }
#endif       
        if(HTS221_Init_Param.subSensorActive[0] && !HTS221_Init_Param.subSensorActive[1]) /* Only Temperature */
        {
          HTS221_Data_Ready((uint8_t *)dataOut, 4, (double)ts_hts221/(double)(SystemCoreClock));
        }
        else if(!HTS221_Init_Param.subSensorActive[0] && HTS221_Init_Param.subSensorActive[1]) /* Only Humidity */
        {
          HTS221_Data_Ready((uint8_t *)&dataOut[1], 4, (double)ts_hts221/(double)(SystemCoreClock));
        }
        else if(HTS221_Init_Param.subSensorActive[0] && HTS221_Init_Param.subSensorActive[1]) /* Both Sub Sensors */
        {
          HTS221_Data_Ready((uint8_t *)dataOut, 8, (double)ts_hts221/(double)(SystemCoreClock));
        } 
      }
    } 
    else if ( HTS221_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {      
#ifdef DATA_TEST
      usbTestData = 0;
#endif
      hts221_power_on_set(&hts221_ctx_instance, PROPERTY_DISABLE);
      HTS221_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(HTS221_Thread_Id);
    }    
  }     
}

static void HTS221_Int_Callback(void)
{
  tim_value = hsm_tim.Instance->CNT;
  
  osSemaphoreRelease(hts221_data_ready_sem_id);    
}

void HTS221_Set_State(SM_Sensor_State_t newState)
{
  HTS221_Sensor_State = newState;
}

void HTS221_Set_ODR(float newODR)
{
  HTS221_Init_Param.ODR = newODR;
}

void HTS221_Set_FS(float newFS1, float newFS2)
{
  HTS221_Init_Param.FS[0] = newFS1;
  HTS221_Init_Param.FS[0] = newFS2;
}

void HTS221_Start(void)
{
  HTS221_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(HTS221_Thread_Id);
}

void HTS221_Stop(void)
{
  HTS221_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void HTS221_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  
}

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
