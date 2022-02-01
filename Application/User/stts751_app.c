/**
  ******************************************************************************
  * @file    stts751_app.c
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   This file provides a set of functions to handle stts751 sensor
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
#include "stts751_app.h"

SM_Init_Param_t STTS751_Init_Param;
SM_Sensor_State_t STTS751_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
/* Semaphore used to wait on BUS data read complete, managed by lower layer */

static void STTS751_Thread(void const *argument);
osThreadId STTS751_Thread_Id;

osSemaphoreId STTS751_ReadSem_id;
osSemaphoreDef(STTS751_ReadSem);

static sensor_handle_t stts751_hdl_instance = {0, STTS751_0xxxx_ADD_7K5, NULL, 0, &STTS751_ReadSem_id};
static stmdev_ctx_t stts751_ctx_instance= {SM_I2C_Write_Os, SM_I2C_Read_Os, &stts751_hdl_instance};

int16_t temperature;

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_stts751 = 0;

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
* @brief STTS751 GPIO Initialization Function
* @param None
* @retval None
*/
void STTS751_Peripheral_Init(void)
{
  
  
}

/**
* @brief STTS751 Threads Creation
* @param None
* @retval None
*/
void STTS751_OS_Init(void)
{  
  STTS751_ReadSem_id = osSemaphoreCreate(osSemaphore(STTS751_ReadSem), 1);
  osSemaphoreWait(STTS751_ReadSem_id,osWaitForever);
  
  /* Thread 1 definition */  
  osThreadDef(STTS751_RD_USR_THREAD, STTS751_Thread, STTS751_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);  
  /* Start thread 1 */
  STTS751_Thread_Id = osThreadCreate(osThread(STTS751_RD_USR_THREAD), NULL);  
  /* Suspend thread */
  osThreadSuspend(STTS751_Thread_Id); 
  
}


/**
* @brief  Get data raw from sensors to queue
* @param  thread not used
* @retval None
*/ 
static void STTS751_Thread(void const *argument)
{
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_STTS751_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_STTS751_DEBUG_PIN );
#endif
  
#ifdef DATA_TEST
  static uint16_t usbTestData = 0;
#endif
  
  uint16_t taskDelay = 1000;
  
  for (;;)
  {
    if (STTS751_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {  
      stts751_id_t STTS751_Id; 
      ts_stts751 = 0;
      tim_value_old = 0;
      
      stts751_device_id_get(&stts751_ctx_instance,  (stts751_id_t *)&STTS751_Id);
      /* ToDo: check Id */
      stts751_temp_data_rate_set(&stts751_ctx_instance,  STTS751_TEMP_ODR_8Hz);
      stts751_resolution_set(&stts751_ctx_instance,  STTS751_12bit);
      
      if(STTS751_Init_Param.ODR < 2.0f)
      {       
        taskDelay = 1000;
      }
      else if(STTS751_Init_Param.ODR < 3.0f)
      {
        taskDelay = 500;
      }
      else if(STTS751_Init_Param.ODR < 5.0f)
      {
        taskDelay = 250;
      }      
      
      STTS751_Sensor_State = SM_SENSOR_STATE_RUNNING;      
    }
    else if(STTS751_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      vTaskDelay(taskDelay);  
      
      if(STTS751_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {    
        float temperature_celsius;        
        tim_value = hsm_tim.Instance->CNT;
        
        if(tim_value >= tim_value_old)
        {
          period = tim_value - tim_value_old;
        }
        else
        {
          period = tim_value + (0xFFFFFFFF - tim_value_old);
        }
        
        tim_value_old = tim_value;
        ts_stts751 += period;              
        
        stts751_temperature_raw_get(&stts751_ctx_instance,  (int16_t *)&temperature);  
        temperature_celsius = (float)temperature / 256.0f;
        
#ifdef DATA_TEST
        uint16_t i = 0;        
        int16_t * p16 = (int16_t *)&temperature_celsius;
        
        for (i = 0; i < 2; i++)    
        {          
          *p16++ = usbTestData++;        
        }
#endif        
        STTS751_Data_Ready((uint8_t *)&temperature_celsius, 4, (double)ts_stts751/(double)(SystemCoreClock));
      }
      
    } 
    else if ( STTS751_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {      
      stts751_temp_data_rate_set(&stts751_ctx_instance,  STTS751_TEMP_ODR_OFF);
      STTS751_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(STTS751_Thread_Id);
    }      
  }
}

void STTS751_Set_State(SM_Sensor_State_t newState)
{
  STTS751_Sensor_State = newState;
}

void STTS751_Set_ODR(float newODR)
{
  STTS751_Init_Param.ODR = newODR;
}

void STTS751_Set_FS(float newFS1, float newFS2)
{
  STTS751_Init_Param.FS[0] = newFS1;
  STTS751_Init_Param.FS[0] = newFS2;
}

void STTS751_Start(void)
{
  STTS751_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(STTS751_Thread_Id);
}

void STTS751_Stop(void)
{
  STTS751_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void STTS751_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
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
