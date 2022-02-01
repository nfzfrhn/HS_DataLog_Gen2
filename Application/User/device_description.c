/**
  ******************************************************************************
  * @file    device_description.c
  * @author  SRA - Central Labs
  * @version v2.1.1
  * @date    26-Feb-2020
  * @brief   This file provides a set of functions to handle the COM structure
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
#include "device_description.h"
#include "sensors_manager.h"
#include "stdlib.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_IIS3DWB       (uint32_t)(32000)
#define WRITE_BUFFER_SIZE_HTS221        (uint32_t)(500)  
#define WRITE_BUFFER_SIZE_IIS2DH        (uint32_t)(8000) 
#define WRITE_BUFFER_SIZE_IIS2MDC       (uint32_t)(2000) 
#define WRITE_BUFFER_SIZE_IMP34DT05     (uint32_t)(32000)
#define WRITE_BUFFER_SIZE_ISM330DHCX    (uint32_t)(16000)
#define WRITE_BUFFER_SIZE_LPS22HH       (uint32_t)(8000) 
#define WRITE_BUFFER_SIZE_MP23ABS1      (uint32_t)(65000)
#define WRITE_BUFFER_SIZE_STTS751       (uint32_t)(100)  

#define DEVICE_ID_REG_1  (0x1FFF7590)
#define DEVICE_ID_REG_2  (0x1FFF7594)
#define DEVICE_ID_REG_3  (0x1FFF7598)

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

extern uint32_t maxWriteTimeSensor[];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


static void get_unique_id(char * id)
{ 
  uint32_t deviceserial[3];
  
  deviceserial[0] = *(uint32_t*)DEVICE_ID_REG_1;
  deviceserial[1] = *(uint32_t*)DEVICE_ID_REG_2;
  deviceserial[2] = *(uint32_t*)DEVICE_ID_REG_3; 
  
  uint16_t y = (deviceserial[0] & 0xFFFF);
  uint16_t x = ((deviceserial[0] >> 16) & 0xFFFF);  
  uint8_t waf_num = (deviceserial[1] & 0xFF);
    
  id[0] = (deviceserial[1]>>8) & 0xFF;
  id[1] = (deviceserial[1]>>16) & 0xFF;
  id[2] = (deviceserial[1]>>24) & 0xFF;  
  id[3] = (deviceserial[2]) & 0xFF;
  id[4] = (deviceserial[2]>>8) & 0xFF;
  id[5] = (deviceserial[2]>>16) & 0xFF;
  id[6] = (deviceserial[2]>>24) & 0xFF;
  
  int n;
  char * p = &id[7];
  
  n = sprintf(p, "%03u", waf_num);
  p+=n;
  
  n = sprintf(p, "%05u", x);
  p+=n;
  
  n = sprintf(p, "%05u", y);
  p+=n;
  
  *p = '\0';
  /*OUT: | LOT NUMBER [7] | WAF NUMBER [3] | X ON WAF [5] | Y ON WAF [5] | */  
}

/**
* @brief Set default device description
* @param None
* @retval None
*/
void set_default_description(void)
{  
  COM_Sensor_t * tempSensor;
  COM_DeviceDescriptor_t * tempDeviceDescriptor;
  tempDeviceDescriptor = COM_GetDeviceDescriptor();
  
  get_unique_id(tempDeviceDescriptor->serialNumber);
  strcpy(tempDeviceDescriptor->alias, "STWIN_001"); 
    
  /***** IIS3DWB *****/
  iis3dwb_com_id = COM_AddSensor();
  
  tempSensor = COM_GetSensor(iis3dwb_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "IIS3DWB"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_INT16;
  tempSensor->sensorDescriptor.ODR[0] = 26667.0f;
  tempSensor->sensorDescriptor.ODR[1] = COM_END_OF_LIST_FLOAT;  /* Terminate list */
  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 26667.0f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 1000;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 3000;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS3DWB;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 3;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "mg"); 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = 4.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[2] = 8.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[3] = 16.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[4] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 16.0f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.061f *  tempSensor->sensorStatus.subSensorStatus[0].FS/2;

  IIS3DWB_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  IIS3DWB_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  IIS3DWB_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  
  maxWriteTimeSensor[iis3dwb_com_id] = 1000 * WRITE_BUFFER_SIZE_IIS3DWB / (uint32_t)(IIS3DWB_Init_Param.ODR * 6); 
  
  /*****                                                                 *****/ 
  
  /***** HTS221 *****/
  hts221_com_id = COM_AddSensor();
  tempSensor = COM_GetSensor(hts221_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "HTS221"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_FLOAT;
  tempSensor->sensorDescriptor.ODR[0] = 1.0f;
  tempSensor->sensorDescriptor.ODR[1] = 7.0f;
  tempSensor->sensorDescriptor.ODR[2] = 12.5f;
  tempSensor->sensorDescriptor.ODR[3] = COM_END_OF_LIST_FLOAT;
  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 2;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 12.5f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 50;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 16;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_HTS221;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_TEMP; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 1;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Celsius");
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 120.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 120.0f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  
    /* SUBSENSOR 1 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  tempSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_HUM; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].dataPerSample = 1;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[1].unit, "%");
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 100.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 1 STATUS */
  tempSensor->sensorStatus.subSensorStatus[1].FS = 100.0f;
  tempSensor->sensorStatus.subSensorStatus[1].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[1].sensitivity = 1.0f;
  
  HTS221_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  HTS221_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  HTS221_Init_Param.FS[1] = tempSensor->sensorStatus.subSensorStatus[1].FS;
  HTS221_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  HTS221_Init_Param.subSensorActive[1] = tempSensor->sensorStatus.subSensorStatus[1].isActive;
  
  maxWriteTimeSensor[hts221_com_id] = 1000 * WRITE_BUFFER_SIZE_HTS221 / (uint32_t)(HTS221_Init_Param.ODR * 8); 
  
  /*****                                                                  *****/ 
    
  /***** IIS2DH *****/
  iis2dh_com_id = COM_AddSensor();
  tempSensor = COM_GetSensor(iis2dh_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "IIS2DH"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_INT16;  
  tempSensor->sensorDescriptor.ODR[0] = 1.0f;
  tempSensor->sensorDescriptor.ODR[1] = 10.0f;
  tempSensor->sensorDescriptor.ODR[2] = 25.0f;
  tempSensor->sensorDescriptor.ODR[3] = 50.0f;
  tempSensor->sensorDescriptor.ODR[4] = 100.0f;
  tempSensor->sensorDescriptor.ODR[5] = 200.0f;
  tempSensor->sensorDescriptor.ODR[6] = 400.0f;
  tempSensor->sensorDescriptor.ODR[7] = 1344.0f;
  tempSensor->sensorDescriptor.ODR[8] = COM_END_OF_LIST_FLOAT;  
  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 1344.0f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 1000;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 2400;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS2DH;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 3;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "mg");
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = 4.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[2] = 8.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[3] = 16.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[4] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 16.0f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  if (tempSensor->sensorStatus.subSensorStatus[0].FS == 16.0f)
  {
    tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.75f;
  }
  else
  {
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.0625f *  tempSensor->sensorStatus.subSensorStatus[0].FS/2;
  }
  
  IIS2DH_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  IIS2DH_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2DH_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  maxWriteTimeSensor[iis2dh_com_id] = 1000 * WRITE_BUFFER_SIZE_IIS2DH / (uint32_t)(IIS2DH_Init_Param.ODR * 6);
  
  /**********/ 
  
  /* IIS2MDC */
  iis2mdc_com_id = COM_AddSensor();  
  tempSensor = COM_GetSensor(iis2mdc_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "IIS2MDC"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_INT16;  
  tempSensor->sensorDescriptor.ODR[0] = 10.0f;
  tempSensor->sensorDescriptor.ODR[1] = 20.0f;
  tempSensor->sensorDescriptor.ODR[2] = 50.0f;
  tempSensor->sensorDescriptor.ODR[3] = 100.0f;
  tempSensor->sensorDescriptor.ODR[4] = COM_END_OF_LIST_FLOAT;  
  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 100.0f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 100;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 600;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS2MDC;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_MAG; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 3;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "gauss");
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 50.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;

  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 50.0f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.5;
  
  IIS2MDC_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  IIS2MDC_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2MDC_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  maxWriteTimeSensor[iis2mdc_com_id] = 1000 * WRITE_BUFFER_SIZE_IIS2MDC / (uint32_t)(IIS2MDC_Init_Param.ODR * 6); 
  
  /**********/ 
  
  /* IMP34DT05 */
  imp34dt05_com_id = COM_AddSensor();
   
  tempSensor = COM_GetSensor(imp34dt05_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "IMP34DT05"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_INT16;  
  tempSensor->sensorDescriptor.ODR[0] = 48000.0f;
  tempSensor->sensorDescriptor.ODR[1] = COM_END_OF_LIST_FLOAT;  
  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 48000.0f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 1000;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 4096;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_IMP34DT05;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;  
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_MIC; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 1;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Waveform");
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 122.5f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 122.5f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0;
  
  IMP34DT05_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  IMP34DT05_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  IMP34DT05_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  maxWriteTimeSensor[imp34dt05_com_id] = 1000 * WRITE_BUFFER_SIZE_IMP34DT05 / (uint32_t)(IMP34DT05_Init_Param.ODR * 2); 
  
/**********/ 
  
  /* ISM330DHCX */
  ism330dhcx_com_id = COM_AddSensor();
  
    tempSensor = COM_GetSensor(ism330dhcx_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "ISM330DHCX"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_INT16;
  tempSensor->sensorDescriptor.ODR[0] = 12.5f;
  tempSensor->sensorDescriptor.ODR[1] = 26.0f;
  tempSensor->sensorDescriptor.ODR[2] = 52.0f;
  tempSensor->sensorDescriptor.ODR[3] = 104.0f;
  tempSensor->sensorDescriptor.ODR[4] = 208.0f;
  tempSensor->sensorDescriptor.ODR[5] = 417.0f;
  tempSensor->sensorDescriptor.ODR[6] = 833.0f;
  tempSensor->sensorDescriptor.ODR[7] = 1667.0f;
  tempSensor->sensorDescriptor.ODR[8] = 3333.0f;
  tempSensor->sensorDescriptor.ODR[9] = 6667.0f;
  tempSensor->sensorDescriptor.ODR[10] = COM_END_OF_LIST_FLOAT;
  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 2;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 6667.0f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 1000;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 2048;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_ISM330DHCX;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 3;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "mg");
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = 4.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[2] = 8.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[3] = 16.0f;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[4] = COM_LIST_SEPARATOR_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 16.0f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.061f * tempSensor->sensorStatus.subSensorStatus[0].FS/2.0f;
  
    /* SUBSENSOR 1 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  tempSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_GYRO; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].dataPerSample = 3;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[1].unit, "mdps");
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 125.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[1] = 250.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[2] = 500.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[3] = 1000.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[4] = 2000.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[5] = 4000.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[6] = COM_END_OF_LIST_FLOAT; 
  
  /* SUBSENSOR 1 STATUS */
  tempSensor->sensorStatus.subSensorStatus[1].FS = 4000.0f;
  tempSensor->sensorStatus.subSensorStatus[1].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[1].sensitivity = 4.375f * tempSensor->sensorStatus.subSensorStatus[1].FS/125.0f;
  
  ISM330DHCX_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  ISM330DHCX_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  ISM330DHCX_Init_Param.FS[1] = tempSensor->sensorStatus.subSensorStatus[1].FS;
  ISM330DHCX_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  ISM330DHCX_Init_Param.subSensorActive[1] = tempSensor->sensorStatus.subSensorStatus[1].isActive;
  
  maxWriteTimeSensor[ism330dhcx_com_id] = 1000 * WRITE_BUFFER_SIZE_ISM330DHCX / (uint32_t)(ISM330DHCX_Init_Param.ODR * 12); 
  
  /**********/ 
  
  /* LPS22HH */
  lps22hh_com_id = COM_AddSensor();
  
  tempSensor = COM_GetSensor(lps22hh_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "LPS22HH"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_FLOAT;
  tempSensor->sensorDescriptor.ODR[0] = 1.0f;
  tempSensor->sensorDescriptor.ODR[1] = 10.0f;
  tempSensor->sensorDescriptor.ODR[2] = 25.0f;
  tempSensor->sensorDescriptor.ODR[3] = 50.0f;
  tempSensor->sensorDescriptor.ODR[4] = 75.0f;
  tempSensor->sensorDescriptor.ODR[5] = 100.0f;
  tempSensor->sensorDescriptor.ODR[6] = 200.0f;
  tempSensor->sensorDescriptor.ODR[7] = COM_END_OF_LIST_FLOAT; 

  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 2;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 200.0f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 200;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 1600;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_LPS22HH;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_PRESS; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 1;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "hPa");
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 1260.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 1260.0f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  
    /* SUBSENSOR 1 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  tempSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_TEMP; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].dataPerSample = 1;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[1].unit, "Celsius");
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 85.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[1].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 1 STATUS */
  tempSensor->sensorStatus.subSensorStatus[1].FS = 85.0f;
  tempSensor->sensorStatus.subSensorStatus[1].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[1].sensitivity = 1.0f;   
  
  LPS22HH_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  LPS22HH_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  LPS22HH_Init_Param.FS[1] = tempSensor->sensorStatus.subSensorStatus[1].FS;
  LPS22HH_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  LPS22HH_Init_Param.subSensorActive[1] = tempSensor->sensorStatus.subSensorStatus[1].isActive;  
  
  maxWriteTimeSensor[lps22hh_com_id] = 1000 * WRITE_BUFFER_SIZE_LPS22HH / (uint32_t)(LPS22HH_Init_Param.ODR * 8); 
  
  /**********/ 
  
  /* MP23ABS1 */
  mp23abs1_com_id = COM_AddSensor();
  
   tempSensor = COM_GetSensor(mp23abs1_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "MP23ABS1"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_INT16;  
  tempSensor->sensorDescriptor.ODR[0] = 192000.0f;
  tempSensor->sensorDescriptor.ODR[1] = COM_END_OF_LIST_FLOAT;  
  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 192000.0f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 1000;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 4096;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_MP23ABS1;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_MIC; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 1;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Waveform");
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 130.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 130.0f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0;
  
  MP23ABS1_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  MP23ABS1_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  MP23ABS1_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  
    
  maxWriteTimeSensor[mp23abs1_com_id] = 1000 * WRITE_BUFFER_SIZE_MP23ABS1 / (uint32_t)(MP23ABS1_Init_Param.ODR * 2); 
  
  /**********/ 
  
  /* STTS751 */
  stts751_com_id = COM_AddSensor();
  
  tempSensor = COM_GetSensor(stts751_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(tempSensor->sensorDescriptor.name, "STTS751"); 
  tempSensor->sensorDescriptor.dataType = DATA_TYPE_FLOAT;  
  tempSensor->sensorDescriptor.ODR[0] = 1.0f;
  tempSensor->sensorDescriptor.ODR[1] = 2.0f; 
  tempSensor->sensorDescriptor.ODR[2] = 4.0f; 
  tempSensor->sensorDescriptor.ODR[3] = COM_END_OF_LIST_FLOAT;  
  tempSensor->sensorDescriptor.samplesPerTimestamp[0] = 0;
  tempSensor->sensorDescriptor.samplesPerTimestamp[1] = 1000;  
  tempSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SENSOR STATUS */
  tempSensor->sensorStatus.ODR = 4.0f;
  tempSensor->sensorStatus.measuredODR = 0.0f;
  tempSensor->sensorStatus.initialOffset = 0.0f;
  tempSensor->sensorStatus.samplesPerTimestamp = 20;
  tempSensor->sensorStatus.isActive = 0;
  tempSensor->sensorStatus.usbDataPacketSize = 16;
  tempSensor->sensorStatus.sdWriteBufferSize = WRITE_BUFFER_SIZE_STTS751;
  tempSensor->sensorStatus.comChannelNumber = -1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  tempSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  tempSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_TEMP; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 1;
  strcpy(tempSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Celsius");
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 100.0f; 
  tempSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  tempSensor->sensorStatus.subSensorStatus[0].FS = 100.0f;
  tempSensor->sensorStatus.subSensorStatus[0].isActive = 1;
  tempSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0;
  
  STTS751_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  STTS751_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  STTS751_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  

  maxWriteTimeSensor[stts751_com_id] = 1000 * WRITE_BUFFER_SIZE_STTS751 / (uint32_t)(STTS751_Init_Param.ODR * 4); 

}

void update_sensorStatus(COM_SensorStatus_t * oldSensorStatus, COM_SensorStatus_t * newSensorStatus, uint8_t sID)
{
  /* Check differencies between oldSensorStatus and newSensorStatus, act properly*/
  /* ODR */
  if(oldSensorStatus->ODR != newSensorStatus->ODR)
  {
    if(COM_IsOdrLegal(newSensorStatus->ODR, sID))
    {
      oldSensorStatus->ODR = newSensorStatus->ODR; /* Todo Setters and getters */
    }
  }
  
  /* isActive */
  if(oldSensorStatus->isActive != newSensorStatus->isActive)
  {
    oldSensorStatus->isActive = newSensorStatus->isActive;
  }
  
  /* subsensor: FS, is Active*/
  for (int i = 0; i < COM_GetSubSensorNumber(sID); i++)
  {
    if(oldSensorStatus->subSensorStatus[i].FS != newSensorStatus->subSensorStatus[i].FS)
    {
      oldSensorStatus->subSensorStatus[i].FS = newSensorStatus->subSensorStatus[i].FS; /* Todo Setters and getters */
    }
    
    if(oldSensorStatus->subSensorStatus[i].isActive != newSensorStatus->subSensorStatus[i].isActive)
    {
      oldSensorStatus->subSensorStatus[i].isActive = newSensorStatus->subSensorStatus[i].isActive; /* Todo Setters and getters */
    }
  }
  
  if (sID == iis3dwb_com_id)
  {
    oldSensorStatus->subSensorStatus[0].sensitivity = 0.061f *  oldSensorStatus->subSensorStatus[0].FS/2;
  }
  else if (sID == iis2dh_com_id)
  {    
    if (oldSensorStatus->subSensorStatus[0].FS == 16.0f)
    {
      oldSensorStatus->subSensorStatus[0].sensitivity = 0.75f;
    }
    else
    {
      oldSensorStatus->subSensorStatus[0].sensitivity = 0.0625f *  oldSensorStatus->subSensorStatus[0].FS/2;
    }
  }
  else if (sID == ism330dhcx_com_id)
  {        
    oldSensorStatus->subSensorStatus[0].sensitivity = 0.061f *  oldSensorStatus->subSensorStatus[0].FS/2;
    oldSensorStatus->subSensorStatus[1].sensitivity = 4.375f * oldSensorStatus->subSensorStatus[1].FS/125.0f;
  }
  
  /* CHANNEL NUMBER */
  if(oldSensorStatus->comChannelNumber != newSensorStatus->comChannelNumber)
  {
    oldSensorStatus->comChannelNumber = newSensorStatus->comChannelNumber; /* Todo Setters and getters */         
  }  
  
   /* CHANNEL NUMBER */
  if(oldSensorStatus->samplesPerTimestamp != newSensorStatus->samplesPerTimestamp)
  {
    oldSensorStatus->samplesPerTimestamp = newSensorStatus->samplesPerTimestamp; /* Todo Setters and getters */         
  }  
}

void update_sensors_config(void)
{  
  COM_Sensor_t * tempSensor;  
  
  tempSensor = COM_GetSensor(iis3dwb_com_id);  
  IIS3DWB_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  IIS3DWB_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  IIS3DWB_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  
  tempSensor = COM_GetSensor(hts221_com_id);
  HTS221_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  HTS221_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  HTS221_Init_Param.FS[1] = tempSensor->sensorStatus.subSensorStatus[1].FS;
  HTS221_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  HTS221_Init_Param.subSensorActive[1] = tempSensor->sensorStatus.subSensorStatus[1].isActive;
  
  tempSensor = COM_GetSensor(iis2dh_com_id);
  IIS2DH_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  IIS2DH_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2DH_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  
  tempSensor = COM_GetSensor(iis2mdc_com_id);
  IIS2MDC_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  IIS2MDC_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2MDC_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  tempSensor = COM_GetSensor(imp34dt05_com_id);
  IMP34DT05_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  IMP34DT05_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  IMP34DT05_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  tempSensor = COM_GetSensor(ism330dhcx_com_id);
  ISM330DHCX_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  ISM330DHCX_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  ISM330DHCX_Init_Param.FS[1] = tempSensor->sensorStatus.subSensorStatus[1].FS;
  ISM330DHCX_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  ISM330DHCX_Init_Param.subSensorActive[1] = tempSensor->sensorStatus.subSensorStatus[1].isActive;
  
  tempSensor = COM_GetSensor(lps22hh_com_id);
  LPS22HH_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  LPS22HH_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  LPS22HH_Init_Param.FS[1] = tempSensor->sensorStatus.subSensorStatus[1].FS;
  LPS22HH_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;
  LPS22HH_Init_Param.subSensorActive[1] = tempSensor->sensorStatus.subSensorStatus[1].isActive; 
  
  tempSensor = COM_GetSensor(mp23abs1_com_id);
  MP23ABS1_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  MP23ABS1_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  MP23ABS1_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  tempSensor = COM_GetSensor(stts751_com_id);
  STTS751_Init_Param.ODR = tempSensor->sensorStatus.ODR;
  STTS751_Init_Param.FS[0] = tempSensor->sensorStatus.subSensorStatus[0].FS;
  STTS751_Init_Param.subSensorActive[0] = tempSensor->sensorStatus.subSensorStatus[0].isActive;  
  
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
