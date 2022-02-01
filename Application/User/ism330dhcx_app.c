/**
  ******************************************************************************
  * @file    ism330dhcx_app.c
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   This file provides a set of functions to handle ism330dhcx sensor
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
#include "ism330dhcx_app.h"
SM_Init_Param_t ISM330DHCX_Init_Param;
SM_Sensor_State_t ISM330DHCX_Sensor_State = SM_SENSOR_STATE_INITIALIZING;


/* Todo Remove: */
static uint8_t ism330dhcx_mem[ISM330DHCX_GY_SAMPLES_PER_IT * 7];

EXTI_HandleTypeDef ism330dhcx_exti;
static void ISM330DHCX_Int_Callback(void);

/* Semaphore used to wait on component interrupt */
osSemaphoreId ism330dhcx_DreadySem_id;
osSemaphoreDef(ism330dhcx_DreadySem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */

osSemaphoreId ism330dhcxReadSem_id;
osSemaphoreDef(ism330dhcxReadSem);

static sensor_handle_t ism330dhcx_hdl_instance = {0, 0, ISM330DHCX_SPI_CS_GPIO_Port, ISM330DHCX_SPI_CS_Pin, &ism330dhcxReadSem_id};
static stmdev_ctx_t ism330dhcx_ctx_instance = {SM_SPI_Write, SM_SPI_Read_Os, &ism330dhcx_hdl_instance};

osThreadId ISM330DHCX_Thread_Id;
static void ISM330DHCX_Thread(void const *argument);

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_ism330dhcx = 0;


/**
* @brief IIS3DWB GPIO Initialization Function
* @param None
* @retval None
*/
void ISM330DHCX_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  ISM330DHCX_SPI_CS_GPIO_CLK_ENABLE();
  ISM330DHCX_INT1_GPIO_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM330DHCX_SPI_CS_GPIO_Port, ISM330DHCX_SPI_CS_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
  GPIO_InitStruct.Pin = ISM330DHCX_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ISM330DHCX_SPI_CS_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  ISM330DHCX_INT1_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ISM330DHCX_INT1_GPIO_Port, &GPIO_InitStruct);  
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ISM330DHCX_INT1_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ISM330DHCX_INT1_EXTI_IRQn);
  
  HAL_EXTI_GetHandle(&ism330dhcx_exti, ISM330DHCX_INT1_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&ism330dhcx_exti,  HAL_EXTI_COMMON_CB_ID, ISM330DHCX_Int_Callback);
}

int32_t retVal = 0;
void ISM330DHCX_OS_Init(void)
{  
  /* Data read complete semaphore initialization */  
  ism330dhcxReadSem_id = osSemaphoreCreate(osSemaphore(ism330dhcxReadSem), 1);
  osSemaphoreWait(ism330dhcxReadSem_id,osWaitForever);
  
  /* Data ready interrupt semaphore initialization */  
  ism330dhcx_DreadySem_id = osSemaphoreCreate(osSemaphore(ism330dhcx_DreadySem), 1);
  osSemaphoreWait(ism330dhcx_DreadySem_id,  osWaitForever);
  
  /* Thread 1 definition */  
  osThreadDef(ISM330_RD_USR_THREAD, ISM330DHCX_Thread, ISM330DHCX_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);  
  /* Start thread 1 */
  ISM330DHCX_Thread_Id = osThreadCreate(osThread(ISM330_RD_USR_THREAD), NULL);   
  /* Suspend thread */
  osThreadSuspend(ISM330DHCX_Thread_Id);
  
}


ism330dhcx_pin_int1_route_t int1_route;
uint8_t testTAGS[ISM330DHCX_GY_SAMPLES_PER_IT];
static void ISM330DHCX_Thread(void const *argument)
{     
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_ISM330DHCX_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_ISM330DHCX_DEBUG_PIN );
#endif
  
#ifdef DATA_TEST
  static uint16_t usbTestData = 0;
#endif
  
  uint8_t reg0;
  uint8_t reg1;
  uint16_t i = 0;
  static uint8_t tempDHCX[ISM330DHCX_GY_SAMPLES_PER_IT*7];
  
  for (;;)
  {
    if (ISM330DHCX_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {  
      ts_ism330dhcx = 0; 
      tim_value_old = 0;
      
      retVal = ism330dhcx_device_id_get(&ism330dhcx_ctx_instance, (uint8_t *)&reg0);
      /*ToDo check who am I and kill thread if not ok */      
      
      retVal = ism330dhcx_reset_set(&ism330dhcx_ctx_instance, 1);
      ism330dhcx_i2c_interface_set(&ism330dhcx_ctx_instance, ISM330DHCX_I2C_DISABLE); 
      
      /* AXL FS */ 
      if(ISM330DHCX_Init_Param.FS[0] < 3.0f)
        ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_2g); 
      else if(ISM330DHCX_Init_Param.FS[0] < 5.0f)
        ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_4g);  
      else if(ISM330DHCX_Init_Param.FS[0] < 9.0f)
        ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_8g);  
      else if(ISM330DHCX_Init_Param.FS[0] < 17.0f)
        ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_16g);  
      
      /* GYRO FS */ 
      if(ISM330DHCX_Init_Param.FS[1] < 126.0f)
        ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_125dps); 
      else if(ISM330DHCX_Init_Param.FS[1] < 251.0f)
        ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_250dps); 
      else if(ISM330DHCX_Init_Param.FS[1] < 501.0f)
        ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_500dps); 
      else if(ISM330DHCX_Init_Param.FS[1] < 1001.0f)
        ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_1000dps); 
      else if(ISM330DHCX_Init_Param.FS[1] < 2001.0f)
        ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_2000dps); 
      else if(ISM330DHCX_Init_Param.FS[1] < 4001.0f)
        ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_4000dps); 
      
      retVal = ism330dhcx_fifo_watermark_set(&ism330dhcx_ctx_instance, ISM330DHCX_WTM_LEVEL);  
      
      int1_route.int1_ctrl.int1_fifo_th = 1;
      retVal = ism330dhcx_pin_int1_route_set(&ism330dhcx_ctx_instance, &int1_route);
      
      retVal = ism330dhcx_fifo_mode_set(&ism330dhcx_ctx_instance, ISM330DHCX_STREAM_MODE);  
      
      ism330dhcx_odr_xl_t ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_OFF;
      ism330dhcx_bdr_xl_t ism330dhcx_bdr_xl = ISM330DHCX_XL_NOT_BATCHED;        
      ism330dhcx_odr_g_t ism330dhcx_odr_g = ISM330DHCX_GY_ODR_OFF;
      ism330dhcx_bdr_gy_t ism330dhcx_bdr_gy = ISM330DHCX_GY_NOT_BATCHED;
      
      if(ISM330DHCX_Init_Param.ODR < 13.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_12Hz5;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_12Hz5;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_12Hz5;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_12Hz5;     
      }
      else if(ISM330DHCX_Init_Param.ODR < 27.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_26Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_26Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_26Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_26Hz;             
      }
      else if(ISM330DHCX_Init_Param.ODR < 53.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_52Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_52Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_52Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_52Hz;             
      }
      else if(ISM330DHCX_Init_Param.ODR < 105.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_104Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_104Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_104Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_104Hz;             
      }
      else if(ISM330DHCX_Init_Param.ODR < 209.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_208Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_208Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_208Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_208Hz;             
      }
      else if(ISM330DHCX_Init_Param.ODR < 418.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_417Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_417Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_417Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_417Hz;             
      }
      else if(ISM330DHCX_Init_Param.ODR < 834.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_833Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_833Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_833Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_833Hz;             
      }
      else if(ISM330DHCX_Init_Param.ODR < 1668.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_1667Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_1667Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_1667Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_1667Hz;             
      }
      else if(ISM330DHCX_Init_Param.ODR < 3334.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_3333Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_3333Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_3333Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_3333Hz;             
      }
      else if(ISM330DHCX_Init_Param.ODR < 6668.0f)
      {
        ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_6667Hz;
        ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_6667Hz;     
        ism330dhcx_odr_g = ISM330DHCX_GY_ODR_6667Hz;
        ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_6667Hz;             
      }     
      
      if(ISM330DHCX_Init_Param.subSensorActive[0])
      {
        ism330dhcx_xl_data_rate_set(&ism330dhcx_ctx_instance, ism330dhcx_odr_xl);
        ism330dhcx_fifo_xl_batch_set(&ism330dhcx_ctx_instance, ism330dhcx_bdr_xl);          
      }
      if(ISM330DHCX_Init_Param.subSensorActive[1])
      {
        ism330dhcx_gy_data_rate_set(&ism330dhcx_ctx_instance, ism330dhcx_odr_g);
        ism330dhcx_fifo_gy_batch_set(&ism330dhcx_ctx_instance, ism330dhcx_bdr_gy);
      } 
      
      HAL_NVIC_EnableIRQ(ISM330DHCX_INT1_EXTI_IRQn);  
      ISM330DHCX_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if(ISM330DHCX_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      osSemaphoreWait(ism330dhcx_DreadySem_id,  osWaitForever); 
      
      if(ISM330DHCX_Sensor_State == SM_SENSOR_STATE_RUNNING)
      {  
        /* Check FIFO_WTM_IA anf fifo level. We do not use PID in order to avoid reading one register twice */
        ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_FIFO_STATUS1, &reg0, 1);
        ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_FIFO_STATUS2, &reg1, 1);     
        
        uint16_t fifo_level = ((reg1 & 0x03) << 8) + reg0; 
        
        if((reg1) & 0x80  && (fifo_level>=ISM330DHCX_GY_SAMPLES_PER_IT) )
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
          ts_ism330dhcx += period;
          
          ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_FIFO_DATA_OUT_TAG, (uint8_t *)ism330dhcx_mem, ISM330DHCX_GY_SAMPLES_PER_IT * 7);
          
#ifdef DATA_TEST
          int16_t * p16 = (int16_t *)ism330dhcx_mem;
          
          for (i = 0; i < ISM330DHCX_GY_SAMPLES_PER_IT; i++)    
          {
            testTAGS[i] = ism330dhcx_mem[i*7]>>3;
            
            *p16++ = usbTestData++;
            *p16++ = usbTestData++;
            *p16++ = usbTestData++;
          }
#else
          if(ism330dhcx_mem[0]>>3 == 0x02 || !(ISM330DHCX_Init_Param.subSensorActive[0]) || !(ISM330DHCX_Init_Param.subSensorActive[1]))   /* First Sample in the fifo is AXL || 1 subsensor active only --> simply drop TAGS */
          {
            int16_t * p16src = (int16_t *)ism330dhcx_mem;
            int16_t * p16dest = (int16_t *)ism330dhcx_mem;
            for (i = 0; i < ISM330DHCX_GY_SAMPLES_PER_IT; i++)    
            {      
              p16src = (int16_t *)&((uint8_t *)(p16src))[1];          
              *p16dest++ = *p16src++;
              *p16dest++ = *p16src++;
              *p16dest++ = *p16src++;
            }
          }
          else /* First Sample in the fifo is NOT AXL && 2 subsensors active, rearrange data (TODO) */
          {   
            int16_t * p16src = (int16_t *)ism330dhcx_mem;
            int16_t * p16temp = (int16_t *)tempDHCX;
            for (i = 0; i < ISM330DHCX_GY_SAMPLES_PER_IT; i++)    
            {      
              p16src = (int16_t *)&((uint8_t *)(p16src))[1];          
              *p16temp++ = *p16src++;
              *p16temp++ = *p16src++;
              *p16temp++ = *p16src++;
            }
            
            int16_t * p16dest = (int16_t *)ism330dhcx_mem;
            int16_t * p16acc = (int16_t *)&((uint8_t *)(tempDHCX))[6];
            int16_t * p16gyro = (int16_t *)tempDHCX;
            for (i = 0; i < ISM330DHCX_GY_SAMPLES_PER_IT/2; i++)    
            {                
              *p16dest++ = *p16acc++;
              *p16dest++ = *p16acc++;
              *p16dest++ = *p16acc++;
              p16acc = (int16_t *)&((uint8_t *)(p16acc))[6];
                  
              *p16dest++ = *p16gyro++;
              *p16dest++ = *p16gyro++;
              *p16dest++ = *p16gyro++;  
              p16gyro = (int16_t *)&((uint8_t *)(p16gyro))[6];
            }
            
          }
#endif    
          ISM330DHCX_Data_Ready((uint8_t *)ism330dhcx_mem, ISM330DHCX_GY_SAMPLES_PER_IT * 6, (double)ts_ism330dhcx/(double)(SystemCoreClock));      
        }
      }      
    } 
    else if ( ISM330DHCX_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#ifdef DATA_TEST
      usbTestData = 0;
#endif      
      ism330dhcx_fifo_gy_batch_set(&ism330dhcx_ctx_instance, ISM330DHCX_GY_NOT_BATCHED);  /* ToDo power down */
      ISM330DHCX_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(ISM330DHCX_Thread_Id);
    }  
  }    
}


static void ISM330DHCX_Int_Callback(void)
{
  tim_value = hsm_tim.Instance->CNT;
  osSemaphoreRelease(ism330dhcx_DreadySem_id);    
}

void ISM330DHCX_Set_State(SM_Sensor_State_t newState)
{
  ISM330DHCX_Sensor_State = newState;
}

void ISM330DHCX_Set_ODR(float newODR)
{
  ISM330DHCX_Init_Param.ODR = newODR;
}

void ISM330DHCX_Set_FS(float newFS1, float newFS2)
{
  ISM330DHCX_Init_Param.FS[0] = newFS1;
  ISM330DHCX_Init_Param.FS[0] = newFS2;
}

void ISM330DHCX_Start(void)
{
  ISM330DHCX_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(ISM330DHCX_Thread_Id);
}

void ISM330DHCX_Stop(void)
{  
  ISM330DHCX_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void ISM330DHCX_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
