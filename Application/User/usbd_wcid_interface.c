/**
******************************************************************************
* @file    usbd_wcid_interface.c
* @author  SRA - Central Labs
* @version v2.1.1
* @date    26-Feb-2020
* @brief   Source file for USBD WCID interface
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
#include "usbd_wcid_interface.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern uint8_t iis3dwb_com_id;
extern uint8_t hts221_com_id;
extern uint8_t iis2dh_com_id;
extern uint8_t iis2mdc_com_id;
extern uint8_t imp34dt05_com_id;
extern uint8_t mp23abs1_com_id;
extern uint8_t ism330dhcx_com_id;
extern uint8_t lps22hh_com_id;
extern uint8_t stts751_com_id;

extern SM_Init_Param_t IIS3DWB_Init_Param;
extern SM_Init_Param_t HTS221_Init_Param;
extern SM_Init_Param_t IIS2DH_Init_Param;
extern SM_Init_Param_t IIS2MDC_Init_Param;
extern SM_Init_Param_t IMP34DT05_Init_Param;
extern SM_Init_Param_t MP23ABS1_Init_Param;
extern SM_Init_Param_t ISM330DHCX_Init_Param;
extern SM_Init_Param_t LPS22HH_Init_Param;
extern SM_Init_Param_t STTS751_Init_Param;

extern uint8_t sensor_first_dataReady[];

uint8_t  *USB_RxBuffer = NULL;
uint8_t * TxBuffer[9];
uint8_t Dummy[APP_TX_DATA_SIZE];

/* Private function prototypes -----------------------------------------------*/
static int8_t WCID_STREAMING_Itf_Init     (void);
static int8_t WCID_STREAMING_Itf_DeInit   (void);
static int8_t WCID_STREAMING_Itf_Control (uint8_t isHostToDevice, uint8_t cmd, uint16_t wValue, uint16_t wIndex, uint8_t* pbuf, uint16_t length);
static int8_t WCID_STREAMING_Itf_Receive  (uint8_t* pbuf, uint32_t Len);

static void _Error_Handler( void );


USBD_WCID_STREAMING_ItfTypeDef USBD_WCID_STREAMING_fops = 
{
  WCID_STREAMING_Itf_Init,
  WCID_STREAMING_Itf_DeInit,
  WCID_STREAMING_Itf_Control,
  WCID_STREAMING_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
* @brief  WCID_STREAMING_Itf_Init
*         Initializes the WCID media low layer
* @param  None
* @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t WCID_STREAMING_Itf_Init(void)
{
  /*ToDo : add state variable, check for allocation success */
  USB_RxBuffer = HSD_calloc(512, sizeof(uint8_t));
  if(USB_RxBuffer == NULL)
  {
    /* Error */
  }    
  
  USBD_WCID_STREAMING_SetRxDataBuffer(&USBD_Device, (uint8_t *)USB_RxBuffer);  
  return (USBD_OK);
}

/**
* @brief  WCID_STREAMING_Itf_DeInit
*         DeInitializes the WCID media low layer
* @param  None
* @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t WCID_STREAMING_Itf_DeInit(void)
{
  /*ToDo : add state variable, check for allocation success */  
  if( USB_RxBuffer != NULL)
  {
    HSD_free(USB_RxBuffer);
    USB_RxBuffer = NULL;
  }
  
  return (USBD_OK);
}


/**
* @brief  WCID_STREAMING_Itf_Control
*         Manage the WCID class requests
* @param  Cmd: Command code            
* @param  Buf: Buffer containing command data (request parameters)
* @param  Len: Number of data to be sent (in bytes)
* @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t WCID_STREAMING_Itf_Control (uint8_t isHostToDevice, uint8_t cmd, uint16_t wValue, uint16_t wIndex, uint8_t* pbuf, uint16_t length)
{ 
  if (com_status != HS_DATALOG_IDLE && com_status != HS_DATALOG_USB_STARTED )
    return USBD_FAIL;
  
  uint32_t i = 0;  
  static uint16_t size = 0;
  static char * serialized = 0;
  static char* p = 0;  
  static COM_Command_t outCommand; 
  static COM_Sensor_t tempSensor;
  COM_SensorStatus_t * myStatus;
  COM_DeviceDescriptor_t * myDeviceDescriptor;
  
  static uint8_t state = STATE_WAITING;
  
  if(isHostToDevice)
  {
    switch(state)
    {
    case STATE_WAITING:
      
      if(cmd != CMD_SIZE_SET)
        return -1; /* error */      
      
      size = *(uint16_t *)pbuf;
      serialized = HSD_malloc(size);
      p = serialized;
      state = STATE_SIZE_RCVD;     
      
      break;
    case STATE_SIZE_RCVD:
      if(cmd != CMD_DATA_SET)
        return -1; /* error */ 
      
      for (i = 0; i < length; i ++)
      {
        *p++ = pbuf[i];
        size--;        
      }
      
      if (size == 0)
      {
        
        HSD_JSON_parse_Command((char *)serialized, &outCommand);
        state = STATE_REQUEST_SET;     
        
        
        if(outCommand.command == COM_COMMAND_SET) 
        {
          myStatus = COM_GetSensorStatus(outCommand.sensorId);
          memcpy(&tempSensor.sensorStatus, myStatus, sizeof(COM_SensorStatus_t));               
          HSD_JSON_parse_Status((char *)serialized, &tempSensor.sensorStatus);    
          HSD_JSON_free(serialized);
          
          update_sensorStatus(myStatus, &tempSensor.sensorStatus, outCommand.sensorId);
          
          
          /* Update the sensor-specific config structure */
          update_sensors_config();        
          
          state = STATE_WAITING;
        }
        else if(outCommand.command == COM_COMMAND_START) 
        {
          com_status = HS_DATALOG_USB_STARTED;
          myDeviceDescriptor = COM_GetDeviceDescriptor();
          SM_TIM_Start();
          
          for (int i = 0; i < myDeviceDescriptor->nSensor; i++)
          {
            myStatus = COM_GetSensorStatus(i);  
            
            if(myStatus->comChannelNumber != -1 && myStatus->isActive)
            {
              TxBuffer[myStatus->comChannelNumber] = NULL;
              TxBuffer[myStatus->comChannelNumber] = HSD_calloc((myStatus->usbDataPacketSize *2 +2), sizeof(uint8_t));
              if(TxBuffer[myStatus->comChannelNumber] == NULL)
              {
                /* Error */
                _Error_Handler();
              }
              
              USBD_WCID_STREAMING_SetTxDataBuffer(&USBD_Device, myStatus->comChannelNumber, TxBuffer[myStatus->comChannelNumber], myStatus->usbDataPacketSize);
              USBD_WCID_STREAMING_CleanTxDataBuffer(&USBD_Device, myStatus->comChannelNumber);
              if(i == iis3dwb_com_id)
              {          
                IIS3DWB_Start();
              }
              else if(i == hts221_com_id)
              {          
                HTS221_Start();
              }
              else if(i == iis2dh_com_id)
              {          
                IIS2DH_Start();
              }
              else if(i == iis2mdc_com_id)
              {          
                IIS2MDC_Start();
              }
              else if(i == imp34dt05_com_id)
              {          
                IMP34DT05_Start();
              }
              else if(i == mp23abs1_com_id)
              {          
                MP23ABS1_Start();
                
              }
              else if(i == ism330dhcx_com_id)
              {          
                ISM330DHCX_Start();
              }
              else if(i == lps22hh_com_id)
              {          
                LPS22HH_Start();
                
              }
              else if(i == stts751_com_id)
              {          
                STTS751_Start();          
              }               
              sensor_first_dataReady[i] = 1;
            }  
            
          }   
          USBD_WCID_STREAMING_StartStreaming(&USBD_Device);
          state = STATE_WAITING;
        }
        else if(outCommand.command == COM_COMMAND_STOP) 
        {
          USBD_WCID_STREAMING_StopStreaming(&USBD_Device);
          com_status = HS_DATALOG_IDLE;
          
          for (int i = 0; i < N_CHANNELS_MAX; i++)
          {
            if( TxBuffer[i] != NULL)
            {
              HSD_free(TxBuffer[i]);
              TxBuffer[i] = NULL;
            }    
          }
          
          myDeviceDescriptor = COM_GetDeviceDescriptor();
          
          for (int i = 0; i < myDeviceDescriptor->nSensor; i++)
          {          
            myStatus = COM_GetSensorStatus(i);  
            
            if(myStatus->comChannelNumber != -1)
            {           
              
              if(i == iis3dwb_com_id)
              {          
                IIS3DWB_Stop();
              }
              else if(i == hts221_com_id)
              {          
                HTS221_Stop();
              }
              else if(i == iis2dh_com_id)
              {          
                IIS2DH_Stop();
              }
              else if(i == iis2mdc_com_id)
              {          
                IIS2MDC_Stop();
              }
              else if(i == imp34dt05_com_id)
              {          
                IMP34DT05_Stop();
              }
              else if(i == mp23abs1_com_id)
              {          
                MP23ABS1_Stop();              
              }
              else if(i == ism330dhcx_com_id)
              {          
                ISM330DHCX_Stop();
              }
              else if(i == lps22hh_com_id)
              {          
                LPS22HH_Stop();              
              }
              else if(i == stts751_com_id)
              {          
                STTS751_Stop();          
              }               
              sensor_first_dataReady[i] = 0;
              //            myStatus->comChannelNumber = -1;
              
            }          
          }        
          
          SM_TIM_Stop(); 
          state = STATE_WAITING;
        }
      }
      break;
    }     
  }
  else /* Device to host */
  {
    switch(state)
    {
    case STATE_REQUEST_SET: /* Host needs size */
      
      if(cmd != CMD_SIZE_GET)
        return -1; /* error*/       
      
      HSD_JSON_free(serialized);
      
      switch(outCommand.request)
      {
        COM_Device_t * myDevice;
        COM_DeviceDescriptor_t * myDeviceDescriptor;
      case COM_REQUEST_DEVICE:
        myDevice = COM_GetDevice();
        size = HSD_JSON_serialize_Device(myDevice, &serialized, SHORT_JSON);
        break;
        
      case COM_REQUEST_DEVICE_INFO:
        myDeviceDescriptor = COM_GetDeviceDescriptor();
        size = HSD_JSON_serialize_DeviceInfo(myDeviceDescriptor, &serialized);
        break;       
        
      case COM_REQUEST_DESCRIPTOR:
        if (outCommand.subSensorId < 0) /* Request is for Sensor, since subSensor was not present in the Json */
        {
          COM_SensorDescriptor_t * mySensorDescriptor =  COM_GetSensorDescriptor(outCommand.sensorId);
          size = HSD_JSON_serialize_SensorDescriptor(mySensorDescriptor, &serialized);           
        }
        else
        {      
          COM_SubSensorDescriptor_t * mySubSensorDescriptor =  COM_GetSubSensorDescriptor(outCommand.sensorId, outCommand.subSensorId);
          size = HSD_JSON_serialize_SubSensorDescriptor(mySubSensorDescriptor, &serialized);
        }       
        break;       
      case COM_REQUEST_STATUS:
        if (outCommand.subSensorId < 0) /* Request is for Sensor, since subSensor was not present in the Json */
        {
          COM_SensorStatus_t * mySensorStatus =  COM_GetSensorStatus(outCommand.sensorId);
          size = HSD_JSON_serialize_SensorStatus(mySensorStatus, &serialized);
        }
        else
        {
          COM_SubSensorStatus_t * mySubSensorStatus = COM_GetSubSensorStatus(outCommand.sensorId, outCommand.subSensorId);           
          size = HSD_JSON_serialize_SubSensorStatus(mySubSensorStatus, &serialized);     
        }
        break;         
      }
      
      *(uint16_t *)pbuf = size;           
      p = serialized;       
      
      state = STATE_SIZE_SENT;       
      break;
    case STATE_SIZE_SENT:
      
      if(cmd != CMD_DATA_GET)
        return -1; /* error*/ 
      
      for (i = 0; i < length; i++)
      {
        pbuf[i] = *p++;
        size--;          
      }
      if(size == 0)
      {
        HSD_JSON_free(serialized);
        serialized = NULL;
        state = STATE_WAITING;         
      }       
      break;    
    }
  }
  
  return (USBD_OK);
}

/**
* @brief  SS_WCID_Itf_DataRx
*         Data received over USB OUT endpoint are sent over WCID interface 
*         through this function.
* @param  Buf: Buffer of data to be transmitted
* @param  Len: Number of data received (in bytes)
* @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t WCID_STREAMING_Itf_Receive(uint8_t* Buf, uint32_t Len)
{  
  return (USBD_OK);
}


/**
* @brief  This function is executed in case of error occurrence
* @param  None
* @retval None
*/
static void _Error_Handler( void )
{
  while (1)
  {}
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
