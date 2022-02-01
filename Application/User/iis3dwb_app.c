/**
  ******************************************************************************
  * @file    iis3dwb_app.c
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   This file provides a set of functions to handle iis3dwb sensor
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
#include "iis3dwb_app.h"

SM_Init_Param_t IIS3DWB_Init_Param;
SM_Sensor_State_t IIS3DWB_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
static osSemaphoreId iis3dwb_data_ready_sem_id;
static osSemaphoreDef(iis3dwb_data_ready_sem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
static osSemaphoreId iis3dwb_data_read_cmplt_sem_id;
static osSemaphoreDef(iis3dwb_data_read_cmplt_sem);

EXTI_HandleTypeDef iis3dwb_exti;
/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
sensor_handle_t iis3dwb_hdl_instance = {IIS3DWB_ID, 0, IIS3DWB_SPI_CS_GPIO_Port, IIS3DWB_SPI_CS_Pin, &iis3dwb_data_read_cmplt_sem_id};
stmdev_ctx_t iis3dwb_ctx_instance = {SM_SPI_Write_Os, SM_SPI_Read_Os, &iis3dwb_hdl_instance};

static uint8_t iis3dwb_mem[IIS3DWB_SAMPLES_PER_IT * 7];
osThreadId IIS3DWB_Thread_Id;

/* Private function prototypes -----------------------------------------------*/
static void IIS3DWB_Thread(void const *argument);
static void IIS3DWB_Int_Callback(void);

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_iis3dwb = 0;


/**
* @brief IIS3DWB GPIO Initialization Function
* @param None
* @retval None
*/
void IIS3DWB_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IIS3DWB_SPI_CS_GPIO_Port, IIS3DWB_SPI_CS_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
  GPIO_InitStruct.Pin = IIS3DWB_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IIS3DWB_SPI_CS_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  IIS3DWB_INT1_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);  
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  HAL_EXTI_GetHandle(&iis3dwb_exti, EXTI_LINE_14);  
  HAL_EXTI_RegisterCallback(&iis3dwb_exti,  HAL_EXTI_COMMON_CB_ID, IIS3DWB_Int_Callback);
  
}

/**
* @brief IIS3DWB Threads Creation
* @param None
* @retval None
*/
void IIS3DWB_OS_Init(void)
{  
  /* Data read complete semaphore initialization */  
  iis3dwb_data_read_cmplt_sem_id = osSemaphoreCreate(osSemaphore(iis3dwb_data_read_cmplt_sem), 1);
  vQueueAddToRegistry( iis3dwb_data_read_cmplt_sem_id, "iis3dwb_data_read_cmplt_sem_id" );

  osSemaphoreWait(iis3dwb_data_read_cmplt_sem_id,osWaitForever);
  
  /* Data ready interrupt semaphore initialization */  
  iis3dwb_data_ready_sem_id = osSemaphoreCreate(osSemaphore(iis3dwb_data_ready_sem), 1);
  vQueueAddToRegistry( iis3dwb_data_ready_sem_id, "iis3dwb_data_ready_sem_id" );

  osSemaphoreWait(iis3dwb_data_ready_sem_id,  osWaitForever);
  

  /* Thread definition: read data */  
  osThreadDef(IIS3DWB_Acquisition_Thread, IIS3DWB_Thread, IIS3DWB_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);  
  /* Start thread 1 */
  IIS3DWB_Thread_Id = osThreadCreate(osThread(IIS3DWB_Acquisition_Thread), NULL); 
  osThreadSuspend(IIS3DWB_Thread_Id);
}


static void IIS3DWB_Thread(void const *argument)
{
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_IIS3DWB_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_IIS3DWB_DEBUG_PIN );
#endif
  
#ifdef DATA_TEST
  static uint16_t usbTestData = 0;
#endif
  
  uint8_t reg0;
  uint8_t reg1;
  
  volatile uint16_t fifo_level = 0;
  
  for (;;)
  {
    if (IIS3DWB_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {  
      ts_iis3dwb = 0;
      tim_value_old = 0;
      
      iis3dwb_device_id_get( &iis3dwb_ctx_instance, (uint8_t *)&reg0);
      iis3dwb_reset_set(&iis3dwb_ctx_instance, 1);
      iis3dwb_read_reg(&iis3dwb_ctx_instance, IIS3DWB_CTRL1_XL, (uint8_t *)&reg0, 1);
      reg0 |= 0xA0;
      iis3dwb_write_reg(&iis3dwb_ctx_instance, IIS3DWB_CTRL1_XL, (uint8_t *)&reg0, 1);
      /*Set fifo in continuous / stream mode*/
      iis3dwb_i2c_interface_set(&iis3dwb_ctx_instance, IIS3DWB_I2C_DISABLE); 
      iis3dwb_fifo_mode_set(&iis3dwb_ctx_instance, IIS3DWB_STREAM_MODE); 
      /*Set watermark*/
      iis3dwb_fifo_watermark_set(&iis3dwb_ctx_instance, IIS3DWB_WTM_LEVEL);
      /*Data Ready pulse mode*/
      iis3dwb_data_ready_mode_set(&iis3dwb_ctx_instance, IIS3DWB_DRDY_PULSED);
      /*Set full scale*/
      if(IIS3DWB_Init_Param.FS[0] < 3.0f)
        iis3dwb_xl_full_scale_set(&iis3dwb_ctx_instance, IIS3DWB_2g);
      else if(IIS3DWB_Init_Param.FS[0] < 5.0f)
        iis3dwb_xl_full_scale_set(&iis3dwb_ctx_instance, IIS3DWB_4g);
      else if(IIS3DWB_Init_Param.FS[0] < 9.0f)
        iis3dwb_xl_full_scale_set(&iis3dwb_ctx_instance, IIS3DWB_8g);
      else if(IIS3DWB_Init_Param.FS[0] < 17.0f)
        iis3dwb_xl_full_scale_set(&iis3dwb_ctx_instance, IIS3DWB_16g);
      
      /*Set 2nd stage filter*/
      iis3dwb_xl_hp_path_on_out_set(&iis3dwb_ctx_instance,IIS3DWB_LP_5kHz);
      /* FIFO_WTM_IA routing on pin INT1 */
      iis3dwb_pin_int1_route_t pin_int1_route;
      *(uint8_t*)&(pin_int1_route.int1_ctrl) = 0;
      *(uint8_t*)&(pin_int1_route.md1_cfg) = 0;
      pin_int1_route.int1_ctrl.int1_fifo_th = 1;
      iis3dwb_pin_int1_route_set(&iis3dwb_ctx_instance, &pin_int1_route);
      
      /*Enable writing to FIFO*/
      iis3dwb_fifo_xl_batch_set(&iis3dwb_ctx_instance, IIS3DWB_XL_BATCHED_AT_26k7Hz);
      
      HAL_NVIC_EnableIRQ(IIS3DWB_INT1_EXTI_IRQn); 
      IIS3DWB_Sensor_State = SM_SENSOR_STATE_RUNNING;      
    }
    else if(IIS3DWB_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      osSemaphoreWait(iis3dwb_data_ready_sem_id,  osWaitForever);     
      
      if(IIS3DWB_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        /* Check FIFO_WTM_IA anf fifo level. We do not use PID in order to avoid reading one register twice */
        iis3dwb_read_reg(&iis3dwb_ctx_instance, IIS3DWB_FIFO_STATUS1, &reg0, 1);
        iis3dwb_read_reg(&iis3dwb_ctx_instance, IIS3DWB_FIFO_STATUS2, &reg1, 1);        
        fifo_level = ((reg1 & 0x03) << 8) + reg0; 
        if((reg1) & 0x80  && (fifo_level>=IIS3DWB_SAMPLES_PER_IT) )
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
          ts_iis3dwb +=  period;
          
          uint16_t i = 0;
          iis3dwb_read_reg(&iis3dwb_ctx_instance, IIS3DWB_FIFO_DATA_OUT_TAG, (uint8_t *)iis3dwb_mem, IIS3DWB_SAMPLES_PER_IT * 7);
          
          /* Arrange Data */
#ifdef DATA_TEST
          int16_t * p16 = (int16_t *)iis3dwb_mem;
          
          for (i = 0; i < IIS3DWB_SAMPLES_PER_IT; i++)    
          {          
            *p16++ = usbTestData++;
            *p16++ = usbTestData++;
            *p16++ = usbTestData++;
          }
#else
          int16_t * p16src = (int16_t *)iis3dwb_mem;
          int16_t * p16dest = (int16_t *)iis3dwb_mem;
          for (i = 0; i < IIS3DWB_SAMPLES_PER_IT; i++)    
          {      
            p16src = (int16_t *)&((uint8_t *)(p16src))[1];          
            *p16dest++ = *p16src++;
            *p16dest++ = *p16src++;
            *p16dest++ = *p16src++;
          }
#endif  
          IIS3DWB_Data_Ready((uint8_t *)iis3dwb_mem, IIS3DWB_SAMPLES_PER_IT * 6, (double)ts_iis3dwb/(double)SystemCoreClock);        
        }
      }
    }
    else if ( IIS3DWB_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#ifdef DATA_TEST
      usbTestData = 0;
#endif      
      iis3dwb_fifo_xl_batch_set(&iis3dwb_ctx_instance, IIS3DWB_XL_NOT_BATCHED);
      IIS3DWB_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(IIS3DWB_Thread_Id);
    }    
  }
}



static void IIS3DWB_Int_Callback(void)
{
  tim_value = hsm_tim.Instance->CNT;
  osSemaphoreRelease(iis3dwb_data_ready_sem_id);
}


void IIS3DWB_Set_State(SM_Sensor_State_t newState)
{
  IIS3DWB_Sensor_State = newState;
}

void IIS3DWB_Set_ODR(float newODR)
{
  IIS3DWB_Init_Param.ODR = newODR;
}

void IIS3DWB_Set_FS(float newFS1, float newFS2)
{
  IIS3DWB_Init_Param.FS[0] = newFS1;
  IIS3DWB_Init_Param.FS[0] = newFS2;
}

void IIS3DWB_Start(void)
{
  IIS3DWB_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(IIS3DWB_Thread_Id);
}

void IIS3DWB_Stop(void)
{
  IIS3DWB_Set_State(SM_SENSOR_STATE_SUSPENDING);  
}


__weak void IIS3DWB_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
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
