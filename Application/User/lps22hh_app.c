/**
  ******************************************************************************
  * @file    lps22hh_app.c
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   This file provides a set of functions to handle lps22hh sensor
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
#include "lps22hh_app.h"
/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

SM_Init_Param_t LPS22HH_Init_Param;
SM_Sensor_State_t LPS22HH_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on BUS data read complete, managed by "sensors manager" */
static osSemaphoreId lps22hh_data_read_cmplt_sem_id;
static osSemaphoreDef(lps22hh_data_read_cmplt_sem);

static sensor_handle_t lps22hh_hdl_instance = {LPS22HH_ID, LPS22HH_I2C_ADD_H, NULL, 0, &lps22hh_data_read_cmplt_sem_id};
static stmdev_ctx_t lps22hh_ctx_instance = {SM_I2C_Write_Os, SM_I2C_Read_Os, &lps22hh_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId LPS22HH_Thread_Id;
static void LPS22HH_Thread(void const *argument);

static uint8_t lps22hh_mem[256 * 5];
static float lps22hh_mem_f[256 * 2];

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_lps22hh = 0;


/**
* @brief GPIO Initialization Function, initialize CS and IT pins
* @param None
* @retval None
*/
void LPS22HH_Peripheral_Init(void)
{  
  /*No additional initialization to be done here */
}

/**
* @brief LPS22HH Threads Creation
* @param None
* @retval None
*/
void LPS22HH_OS_Init(void)
{  
  /* Data read complete semaphore initialization */  
  lps22hh_data_read_cmplt_sem_id = osSemaphoreCreate(osSemaphore(lps22hh_data_read_cmplt_sem), 1);
  osSemaphoreWait(lps22hh_data_read_cmplt_sem_id,osWaitForever);
  
  /* Thread definition: read data */  
  osThreadDef(LPS22HH_Acquisition_Thread, LPS22HH_Thread, LPS22HH_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);  
  /* Start thread 1 */
  LPS22HH_Thread_Id = osThreadCreate(osThread(LPS22HH_Acquisition_Thread), NULL);
  /* Suspend thread */
  osThreadSuspend(LPS22HH_Thread_Id);
  
}

volatile float pout_dbg = 0.0f;
volatile float tpout_dbg = 0.0f;
volatile uint8_t fifo_status_LPS22HH = 0;
static void LPS22HH_Thread(void const *argument)
{
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_LPS22HH_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_LPS22HH_DEBUG_PIN );
#endif
  
#ifdef DATA_TEST
  static uint16_t usbTestData = 0;
#endif
  
  uint8_t reg0;
  uint16_t taskDelay = 0;
  
  for (;;)
  {    
    if (LPS22HH_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {  
      ts_lps22hh = 0;
      tim_value_old = 0;
      
      lps22hh_device_id_get( &lps22hh_ctx_instance, (uint8_t *)&reg0);  
      /* Power Down */
      
      /* Disable MIPI I3C(SM) interface */
      lps22hh_i3c_interface_set(&lps22hh_ctx_instance, LPS22HH_I3C_DISABLE);
      
      /* Power down the device, set Low Noise Enable (bit 5), clear One Shot (bit 4) */
      lps22hh_data_rate_set(&lps22hh_ctx_instance, (lps22hh_odr_t)(LPS22HH_POWER_DOWN | 0x10));
      
      /* Disable low-pass filter on LPS22HH pressure data */
      lps22hh_lp_bandwidth_set(&lps22hh_ctx_instance, LPS22HH_LPF_ODR_DIV_2);
      
      /* Set block data update mode */
      lps22hh_block_data_update_set(&lps22hh_ctx_instance, PROPERTY_ENABLE);
      
      /* Set autoincrement for multi-byte read/write */
      lps22hh_auto_increment_set(&lps22hh_ctx_instance, PROPERTY_ENABLE);
      
      lps22hh_reset_set(&lps22hh_ctx_instance,1);
      
      /* Set fifo mode */
      lps22hh_fifo_mode_set(&lps22hh_ctx_instance, LPS22HH_STREAM_MODE);  
      
      if(LPS22HH_Init_Param.ODR < 2.0f)
      {
        lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_1_Hz);
        taskDelay = 1000;
      }
      else if(LPS22HH_Init_Param.ODR < 11.0f)
      {
        lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_10_Hz);
        taskDelay = 1000;
      }
      else if(LPS22HH_Init_Param.ODR < 26.0f)
      {
        lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_25_Hz); 
        taskDelay = 1000;
      }   
      else if(LPS22HH_Init_Param.ODR < 51.0f)
      {
        lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_50_Hz);
        taskDelay = 1000;
      }
      else if(LPS22HH_Init_Param.ODR < 76.0f)
      {
        lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_75_Hz);
        taskDelay = 1000;
      }
      else if(LPS22HH_Init_Param.ODR < 101.0f)
      {
        lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_100_Hz);
        taskDelay = 1000;
      }
      else if(LPS22HH_Init_Param.ODR < 201.0f)
      {
        lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_200_Hz);
        taskDelay = 500;
      }
      LPS22HH_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if(LPS22HH_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      vTaskDelay(taskDelay);     
      
      if(LPS22HH_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {      
        lps22hh_fifo_data_level_get(&lps22hh_ctx_instance, (uint8_t *)&fifo_status_LPS22HH) ;
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
        ts_lps22hh += period;
        
        lps22hh_read_reg(&lps22hh_ctx_instance, LPS22HH_FIFO_DATA_OUT_PRESS_XL, (uint8_t *) lps22hh_mem, 5*fifo_status_LPS22HH);
        uint16_t i = 0;
        
        for(i = 0; i < fifo_status_LPS22HH; i++)
        {
          uint32_t temp_p = (((uint32_t)lps22hh_mem[5 * i + 0])) | (((uint32_t)lps22hh_mem[5 * i + 1]) << (8 * 1)) | (((uint32_t)lps22hh_mem[5 * i + 2]) << (8 * 2));
          
          /* convert the 2's complement 24 bit to 2's complement 32 bit */
          if(temp_p & 0x00800000)
            temp_p |= 0xFF000000;
          
          uint16_t temp_t = *((uint16_t *)(&lps22hh_mem[5 * i + 3]));
          
          if(LPS22HH_Init_Param.subSensorActive[0] && !LPS22HH_Init_Param.subSensorActive[1]) /* Only Pressure */
          {
            lps22hh_mem_f[i] = (float)temp_p/4096.0f; /* Pressure */
          }
          else if(!LPS22HH_Init_Param.subSensorActive[0] && LPS22HH_Init_Param.subSensorActive[1]) /* Only Temperature */
          {        
            lps22hh_mem_f[i] = (float)temp_t/100.0f; /* Temperature */
          }
          else if(LPS22HH_Init_Param.subSensorActive[0] && LPS22HH_Init_Param.subSensorActive[1]) /* Both Sub Sensors */
          {
            lps22hh_mem_f[2*i] = (float)temp_p/4096.0f; /* Pressure */
            lps22hh_mem_f[2*i + 1] = (float)temp_t/100.0f; /* Temperature */
          }  
        }
        
#ifdef DATA_TEST
        
        uint16_t * p16 = (uint16_t *)lps22hh_mem_f;
        
        for (i = 0; i < fifo_status_LPS22HH*4 ; i++)    
        {          
          *p16++ = usbTestData++;        
        }
#endif     
        
        if(LPS22HH_Init_Param.subSensorActive[0] && LPS22HH_Init_Param.subSensorActive[1]) /* Both Sub Sensors */
        {
          LPS22HH_Data_Ready((uint8_t *)lps22hh_mem_f, 8*fifo_status_LPS22HH , (double)ts_lps22hh/(double)(SystemCoreClock)); /*Todo check dimension / format...*/
        } 
        else if(LPS22HH_Init_Param.subSensorActive[0] || LPS22HH_Init_Param.subSensorActive[1])
        {
          LPS22HH_Data_Ready((uint8_t *)lps22hh_mem_f, 4*fifo_status_LPS22HH , (double)ts_lps22hh/(double)(SystemCoreClock)); /*Todo check dimension / format...*/
        }
      }
    } 
    else if ( LPS22HH_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#ifdef DATA_TEST
      usbTestData = 0;
#endif      
      lps22hh_data_rate_set(&lps22hh_ctx_instance, (lps22hh_odr_t)(LPS22HH_POWER_DOWN | 0x10));
      LPS22HH_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(LPS22HH_Thread_Id);
    }    
  }     
}

void LPS22HH_Set_State(SM_Sensor_State_t newState)
{
  LPS22HH_Sensor_State = newState;
}

void LPS22HH_Set_ODR(float newODR)
{
  LPS22HH_Init_Param.ODR = newODR;
}

void LPS22HH_Set_FS(float newFS1, float newFS2)
{
  LPS22HH_Init_Param.FS[0] = newFS1;
  LPS22HH_Init_Param.FS[0] = newFS2;
}

void LPS22HH_Start(void)
{
  LPS22HH_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(LPS22HH_Thread_Id);
}

void LPS22HH_Stop(void)
{
  LPS22HH_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void LPS22HH_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
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
