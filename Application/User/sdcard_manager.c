/**
******************************************************************************
* @file    sdcard_manager.c
* @author  SRA - Central Labs
* @version v2.1.1
* @date    26-Feb-2020
* @brief   This file provides a set of functions to handle SDcard communication
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
#include "sdcard_manager.h"
#include "main.h"
#include "com_manager.h"
#include "cmsis_os.h"
#include "string.h"
#include <math.h>

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"


#define LOG_DIR_PREFIX    "STWIN_"

/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL FileHandler[COM_MAX_SENSORS];
FIL FileConfigHandler;
FIL FileLogError;
FIL FileConfigJSON;
char SDPath[4]; /* SD card logical drive path */

SD_HandleTypeDef hsd1;

osThreadId SDM_Thread_Id;
static void SDM_Thread(void const *argument);

static void _Error_Handler( void );

osSemaphoreId sdioSem_id;
osSemaphoreDef(sdioSem);

osMessageQId sdThreadQueue_id;
osMessageQDef(sdThreadQueue, 200, int);

extern uint8_t iis3dwb_com_id;
extern uint8_t hts221_com_id;
extern uint8_t iis2dh_com_id;
extern uint8_t iis2mdc_com_id;
extern uint8_t imp34dt05_com_id;
extern uint8_t mp23abs1_com_id;
extern uint8_t ism330dhcx_com_id;
extern uint8_t lps22hh_com_id;
extern uint8_t stts751_com_id;

/* Private function prototypes -----------------------------------------------*/
static void Enable_Sensors(void);
static void Activate_Sensor(uint32_t id);

uint8_t SDM_Memory_Init(void);
uint8_t SDM_Memory_Deinit(void);
uint32_t SDM_GetLastDirNumber(void);

uint8_t batt_error(void);
uint8_t log_error(float timeError);

uint8_t *SD_WriteBuffer[COM_MAX_SENSORS];
volatile uint32_t SD_WriteBufferIdx[COM_MAX_SENSORS] = {0};

uint8_t SD_Logging_Active = 0;
uint8_t SD_present = 0;
uint8_t init_SD_peripheral = 0;

extern uint8_t sensor_first_dataReady[];

extern uint32_t maxWriteTimeSensor[];
float writeTimeError = 0;
uint32_t t_start = 0;
uint32_t oldTime[COM_MAX_SENSORS] = {0};
uint32_t currentTime[COM_MAX_SENSORS], writeTime[COM_MAX_SENSORS] = {0};
uint32_t maxWriteTime[COM_MAX_SENSORS] = {0};
uint32_t testTime = 0;

volatile uint8_t BatteryLow = 0; 

#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT)
uint32_t SD_Logging_Time_Start=0;
uint32_t SD_Logging_Enabled = 0;
#endif

/*----------------------------------------------------------------------------*/
void Enable_Sensors(void)
{  
  /* Comment or uncomment each of the following lines
  * to chose which sensor you want to log.         */
  
  Activate_Sensor(iis3dwb_com_id);
  Activate_Sensor(hts221_com_id);
  Activate_Sensor(iis2dh_com_id);
  Activate_Sensor(iis2mdc_com_id);
  Activate_Sensor(imp34dt05_com_id);
  Activate_Sensor(mp23abs1_com_id);
  Activate_Sensor(ism330dhcx_com_id);
  Activate_Sensor(lps22hh_com_id);
  Activate_Sensor(stts751_com_id);
}


void Activate_Sensor(uint32_t id)
{
  COM_SensorStatus_t * sensor_status = COM_GetSensorStatus(id);  
  sensor_status->isActive = 1;
}



/*----------------------------------------------------------------------------*/
static void SDM_Thread(void const *argument)
{
  (void)argument;
  osEvent evt;
  DIR dir;
  static FILINFO fno;
  int isJSON = 0;
  
  if (BSP_SD_IsDetected())
  {        
    if (init_SD_peripheral != 1)
    {
      SDM_SD_Init();
      init_SD_peripheral = 1;
    }
    
    /* Check if a custom configuration JSON is available in the root folder of the SD Card*/   
    (void)f_opendir(&dir, "/"); /* Open the root directory */
    
    for (;;) 
    {
      (void)f_readdir(&dir, &fno); /* Read files in root folder */
      if (fno.fname[0] == 0) break;
      if (fno.fattrib & AM_ARC) /* It is a file. */
      {
        isJSON = strcmp(".json",fno.fname);
        if (isJSON)
        {
          if(f_open(&FileConfigJSON, fno.fname, FA_OPEN_EXISTING | FA_READ) == FR_OK)
          {  
            char* config_JSON_string = NULL;
            int sizeFile;
            uint32_t br;  
            sizeFile = f_size(&FileConfigJSON)+1;
            config_JSON_string = HSD_malloc(sizeFile);
            f_read (&FileConfigJSON, config_JSON_string, sizeFile, &br);
            SDM_ReadJSON(config_JSON_string);
            HSD_JSON_free(config_JSON_string);
            config_JSON_string = NULL;
            f_close(&FileConfigJSON);
          }
        }
        break; 
      }
    }
    f_closedir(&dir);
    
    if (isJSON == 0)
    {
      Enable_Sensors();    
    }
    
    if (init_SD_peripheral != 0)
    {
      SDM_SD_DeInit();
      init_SD_peripheral = 0;
    }   
  }
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_SDM_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_SDM_DEBUG_PIN );
#endif
  for (;;)
  {
    BSP_LED_Off(LED1); 
    
    /* If the battery is too low close the file and turn off the system */
    if(BatteryLow == 1)
    {
#ifdef LOG_ERROR
      batt_error();
#endif      
      SM_TIM_Stop();
      if(SDM_CloseFiles() == 0)
      {
        SD_Logging_Active = 0;
      }
      SDM_Memory_Deinit();
      
      if (init_SD_peripheral != 0)
      {
        SDM_SD_DeInit();
        init_SD_peripheral = 0;
      }
      BSP_BC_CmdSend(SHIPPING_MODE_ON);
    }      
    
    evt = osMessageGet(sdThreadQueue_id, osWaitForever);  // wait for message      
    
    if (com_status == HS_DATALOG_IDLE || com_status == HS_DATALOG_SD_STARTED )
    {      
      BSP_LED_On(LED1);
      
      if (evt.status == osEventMessage)
      {
        if(evt.value.v == SDM_START_STOP)
        {
          if(SD_Logging_Active == 0)
          {
            com_status = HS_DATALOG_SD_STARTED;            
            SM_TIM_Start();
            if (BSP_SD_IsDetected())
            {              
              if (init_SD_peripheral != 1)
              {
                SDM_SD_Init();
                init_SD_peripheral = 1;
              }
              SD_present = 1;
              if(SDM_InitFiles() == 0)
              {
                SD_Logging_Active = 1;
                BSP_LED_Off(LED_ORANGE);
              }
              testTime = HAL_GetTick();
            }
            else
            {
              SD_present = 0;
            }
          }
          else if (SD_Logging_Active == 1)
          {      
            SM_TIM_Stop();
            
            if(SDM_CloseFiles() == 0)
            {
              SD_Logging_Active = 0;
            }
            SDM_Memory_Deinit();
            
            if (init_SD_peripheral != 0)
            {
              SDM_SD_DeInit();
              init_SD_peripheral = 0;
            }
            com_status = HS_DATALOG_IDLE;
          }
        }
        else
        {
          if(evt.value.v & SDM_DATA_READY_MASK)
          {
            COM_SensorStatus_t * sensor_status; 
            uint32_t buf_size;
            uint8_t sensor_id = (uint8_t)(evt.value.v & SDM_SENSOR_ID_MASK);
            
#ifdef LOG_ERROR
            if (SD_Logging_Active != 0)
            {
              oldTime[sensor_id] = HAL_GetTick();
            }
#endif          
            
            sensor_status = COM_GetSensorStatus(sensor_id);
            buf_size = sensor_status->sdWriteBufferSize;
            
            if(evt.value.v & SDM_DATA_FIRST_HALF_MASK) // Data available on first half of the circular buffer
            {
              SDM_WriteBuffer(sensor_id, SD_WriteBuffer[sensor_id], buf_size);
            }
            else // Data available on second half of the circular buffer
            {
              SDM_WriteBuffer(sensor_id, (uint8_t *)(SD_WriteBuffer[sensor_id]+buf_size), buf_size);
            }
            
#ifdef LOG_ERROR
            if (SD_Logging_Active != 0)
            {
              currentTime[sensor_id] = HAL_GetTick();
              writeTime[sensor_id] = currentTime[sensor_id] - oldTime[sensor_id];
              if (writeTime[sensor_id] > maxWriteTime[sensor_id]  || maxWriteTime[sensor_id] == 0)
              {
                maxWriteTime[sensor_id] = writeTime[sensor_id];
              } 
            }
            
            if ( (writeTime[sensor_id] > maxWriteTimeSensor[sensor_id]) || (BatteryLow == 1) )
            {
              /* Write in 'log_error' file when ther is an SD_card write error */
              BSP_LED_On(LED2);
              writeTimeError = (float) (currentTime[sensor_id] - testTime);
              writeTimeError = writeTimeError/1000.0f;
              log_error(writeTimeError);
            }
            else
            {
              BSP_LED_Off(LED2);
            }
#endif
            
          }
        }
      }
    }
  }
}



/**
* @brief  PWR PVD interrupt callback
* @param  None 
* @retval None
*/
void HAL_PWR_PVDCallback(void)
{
  BatteryLow = 1;
}


uint8_t batt_error(void)
{
  uint32_t byteswritten;
  
  if(f_write(&FileLogError, "\nRECHARGE THE BATTERY", 22, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }
  return 0;
}


uint8_t log_error(float timeError)
{
  uint32_t byteswritten;
  char error[25];
  
  sprintf(error, "\nDATA CORRUPTED at %.3f", timeError);
  if(f_write(&FileLogError, error, 27, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }
  return 0;
}

/**
* @brief  SD Card Manager memory initialization. Performs the dinamic allocation for
*         the SD_WriteBuffer associated to each active sensor.
* @param  
* @retval 1: no error
*/
uint8_t SDM_Memory_Init(void)
{
  COM_SensorStatus_t * sensor_status; 
  COM_DeviceDescriptor_t * device_descriptor;
  uint32_t i;
  
  device_descriptor = COM_GetDeviceDescriptor();
  
  for(i=0;i<device_descriptor->nSensor;i++)
  {
    sensor_status = COM_GetSensorStatus(i);
    if(sensor_status->isActive)
    {
      SD_WriteBuffer[i] = HSD_malloc(sensor_status->sdWriteBufferSize*2);
      if(!SD_WriteBuffer[i])
      {
        _Error_Handler();
      }
    }
    else
    {
      SD_WriteBuffer[i] = 0;
    }
  }
  return 1;
}

/**
* @brief  SD Card Manager memory De-initialization.
* @param  
* @retval 1: no error
*/
uint8_t SDM_Memory_Deinit(void)
{
  COM_SensorStatus_t * sensor_status; 
  COM_DeviceDescriptor_t * device_descriptor;
  uint32_t i;
  
  device_descriptor = COM_GetDeviceDescriptor();
  
  for(i=0;i<device_descriptor->nSensor;i++)
  {
    sensor_status = COM_GetSensorStatus(i);
    if(sensor_status->isActive && SD_WriteBuffer[i]!=0)
    {
      HSD_free(SD_WriteBuffer[i]);
      SD_WriteBuffer[i] = NULL;
    }
  }
  return 1;
}


void SDM_Peripheral_Init(void)
{
  BSP_SD_Detect_Init();   
}

/**
* @brief  Initialize SD Card Manager thread and queue
* @param  None
* @retval None
*/
void SDM_OS_Init(void)
{
  sdioSem_id = osSemaphoreCreate(osSemaphore(sdioSem), 1);
  osSemaphoreWait(sdioSem_id, osWaitForever);
  
  sdThreadQueue_id = osMessageCreate(osMessageQ(sdThreadQueue), NULL);
  
  vQueueAddToRegistry( sdThreadQueue_id, "sdThreadQueue_id" );
  
  /* Thread definition: read data */
  osThreadDef(SDM_On_Off_Thread, SDM_Thread, SD_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE*4);
  /* Start thread 1 */
  SDM_Thread_Id = osThreadCreate(osThread(SDM_On_Off_Thread), NULL);
}


/**
* @brief  Initialize SD Card and file system
* @param  None
* @retval None
*/
void SDM_SD_Init(void)
{
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}


/**
* @brief  Deinitialize SD Card and file system
* @param  None
* @retval None
*/
void SDM_SD_DeInit(void)
{
  if(FATFS_UnLinkDriver(SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(NULL, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}

uint8_t SDM_OpenLogErrorFile(const char *name)
{
  uint32_t byteswritten;
  
  if(f_open(&FileLogError, (char const*)name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  } 
  
  if(f_write(&FileLogError, "WARNING: possible data loss at samples [seconds]: ", 50, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }
  return 0; // OK
}


uint8_t SDM_OpenFile(uint32_t id, const char *sensorName)
{
  char file_name[50];
  
  sprintf(file_name, "%s%s", sensorName, ".dat");
  
  if(f_open(&FileHandler[id], (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }
  
  return 0; // OK
}


uint8_t SDM_CloseFile(uint32_t id)
{
  return f_close(&FileHandler[id]);
}


/**
* @brief  Scan SD Card file system to find the latest directory number that includes to the LOG_DIR_PREFIX
* @param  None
* @retval 
*/
uint32_t SDM_GetLastDirNumber(void)
{
  FRESULT fr;     /* Return value */
  DIR dj;         /* Directory search object */
  FILINFO fno;    /* File information */
  int dir_n = 0, tmp;
  char dir_name[sizeof(LOG_DIR_PREFIX)+1] = LOG_DIR_PREFIX;
  
  dir_name[sizeof(LOG_DIR_PREFIX)-1] = '*';  /* wildcard */
  dir_name[sizeof(LOG_DIR_PREFIX)] = 0;
  
  fr = f_findfirst(&dj, &fno, "", dir_name);  /* Start to search for matching directories */
  if(fno.fname[0])
  {
    tmp = strtol(&fno.fname[sizeof(LOG_DIR_PREFIX)],NULL,10);
    if(dir_n<tmp)
    {
      dir_n = tmp;
    }
  }
  
  /* Repeat while an item is found */
  while (fr == FR_OK && fno.fname[0])
  {
    fr = f_findnext(&dj, &fno);   /* Search for next item */
    if(fno.fname[0])
    {
      tmp = strtol(&fno.fname[sizeof(LOG_DIR_PREFIX)],NULL,10);
      if(tmp > dir_n)
      {
        dir_n = tmp;
      }
    }
  }
  
  f_closedir(&dj);
  return (uint32_t)dir_n;
}


/**
* @brief  Open one file for each sensor to store raw data and a JSON file with the device configuration
* @param  None
* @retval None
*/
uint8_t SDM_InitFiles(void)
{
  COM_SensorStatus_t * sensor_status; 
  COM_DeviceDescriptor_t * device_descriptor;
  COM_SensorDescriptor_t * sensor_descriptor; 
  
  uint32_t i = 0, dir_n = 0;
  char dir_name[sizeof(LOG_DIR_PREFIX)+4];
  char file_name[50];
  
  device_descriptor = COM_GetDeviceDescriptor();  
  dir_n = SDM_GetLastDirNumber();
  dir_n++;
  
  sprintf(dir_name, "%s%03ld", LOG_DIR_PREFIX, dir_n);
  
  FRESULT test = f_mkdir(dir_name);
  if(test != FR_OK)
  {
    return 1;
  }
  
#ifdef LOG_ERROR
  sprintf(file_name, "%s/log_error.txt", dir_name);
  if (SDM_OpenLogErrorFile(file_name)!=0)
  {
    return 1;    
  }
#endif
  
  for(i=0;i<device_descriptor->nSensor;i++)
  {
    sensor_status = COM_GetSensorStatus(i);
    
    if(sensor_status->isActive)
    {
      sensor_descriptor = COM_GetSensorDescriptor(i);
      sprintf(file_name, "%s/%s", dir_name, sensor_descriptor->name);
      
      if(SDM_OpenFile(i, file_name)!=0)
      {
        return 1;
      }
    }
  }
  
  SDM_Memory_Init();
  
  for(i=0;i<device_descriptor->nSensor;i++)
  {
    sensor_status = COM_GetSensorStatus(i);
    
    if(sensor_status->isActive)
    {
      SDM_StartSensorThread(i);
    }
  }  
  
  return 0;
}

uint8_t SDM_CloseFiles(void)
{
  COM_SensorStatus_t * sensor_status; 
  COM_DeviceDescriptor_t * device_descriptor;
  uint32_t id = 0, dir_n = 0;
  char dir_name[sizeof(LOG_DIR_PREFIX)+4];
  char file_name[50];
  char* JSON_string = NULL;
  
  device_descriptor = COM_GetDeviceDescriptor();
  
  /* Put all the sensors in "SUSPENDED" mode */
  for(id=0;id<device_descriptor->nSensor;id++)
  {
    sensor_status = COM_GetSensorStatus(id);
    
    if(sensor_status->isActive)
    {
      SDM_StopSensorThread(id);
    }
  }
  
  /* Flush remaining data and close the files  */
  for(id=0;id<device_descriptor->nSensor;id++)
  {
    sensor_status = COM_GetSensorStatus(id);
    
    if(sensor_status->isActive)
    {
      SDM_Flush_Buffer(id);
      if(SDM_CloseFile(id)!=0)
      {
        return 1;
      }
    }
  } 
  
#ifdef LOG_ERROR
  if (f_close(&FileLogError)!= FR_OK)
  {
    return 1;
  }  
#endif
  
  dir_n = SDM_GetLastDirNumber();
  sprintf(dir_name, "%s%03ld", LOG_DIR_PREFIX, dir_n);
  sprintf(file_name, "%s/DeviceConfig.json", dir_name);
  
  if(f_open(&FileConfigHandler, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }
  
  (void)SDM_CreateJSON(&JSON_string);
  SDM_WriteConfigBuffer((uint8_t*)JSON_string, strlen(JSON_string));  
  
  if (f_close(&FileConfigHandler)!= FR_OK)
  {
    return 1;
  }
  
  HSD_JSON_free(JSON_string);
  JSON_string = NULL;
  
  return 0;
}


uint8_t SDM_WriteConfigBuffer(uint8_t *buffer, uint32_t size)
{
  uint32_t byteswritten;
  FRESULT returnWrite;
  
  returnWrite = f_write(&FileConfigHandler, buffer, size, (void *)&byteswritten);
  if(returnWrite != FR_OK)
  {
    return 0;
  }  
  return 1;
}


uint8_t SDM_WriteBuffer(uint32_t id, uint8_t *buffer, uint32_t size)
{
  uint32_t byteswritten;
  
  if(f_write(&FileHandler[id], buffer, size, (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }  
  return 1;
}

/* Write remaining data to file */
uint8_t SDM_Flush_Buffer(uint32_t id)
{
  uint8_t ret = 0;
  uint32_t buf_size;
  COM_SensorStatus_t * sensor_status;   
  
  sensor_status = COM_GetSensorStatus(id);
  buf_size = sensor_status->sdWriteBufferSize;
  
  if(SD_WriteBufferIdx[id]>0 && SD_WriteBufferIdx[id]<(buf_size-1))
  {
    /* flush from the beginning */
    ret = SDM_WriteBuffer(id, SD_WriteBuffer[id], SD_WriteBufferIdx[id]+1);
  }
  else if (SD_WriteBufferIdx[id]>(buf_size-1) && SD_WriteBufferIdx[id]<(buf_size*2-1))
  {
    /* flush from half buffer */
    ret =  SDM_WriteBuffer(id, (uint8_t *)(SD_WriteBuffer[id]+buf_size), SD_WriteBufferIdx[id]+1-buf_size);
  }
  
  SD_WriteBufferIdx[id] = 0;
  return ret;
}

/* Fill SD buffer with new data */
uint8_t SDM_Fill_Buffer(uint8_t id, uint8_t *src, uint16_t srcSize)
{
  uint8_t *dst;
  uint32_t dstP, srcP=0;
  COM_SensorStatus_t * sensor_status; 
  uint32_t dstSize, sdBufSize;
  
  
  sensor_status = COM_GetSensorStatus(id);
  dst = SD_WriteBuffer[id];
  dstP = SD_WriteBufferIdx[id];
  sdBufSize = sensor_status->sdWriteBufferSize;
  dstSize = sdBufSize*2;
  
  /* byte per byte copy */
  while(srcP < srcSize)
  {
    dst[dstP] = src[srcP];
    dstP++;
    srcP++;
    if(dstP>=dstSize)
    {
      dstP=0;
    }
  }
  
  if(SD_WriteBufferIdx[id]<(dstSize/2) && dstP>=(dstSize/2)) // first half full
  {           
    // unlock write task
    if(osMessagePut(sdThreadQueue_id, id|SDM_DATA_READY_MASK|SDM_DATA_FIRST_HALF_MASK, 0) != osOK)
    {
      _Error_Handler();
    }
    
    // check for buffer consistency
  }
  else if(dstP<SD_WriteBufferIdx[id])  // second half full
  {
    if(osMessagePut(sdThreadQueue_id, id|SDM_DATA_READY_MASK|SDM_DATA_SECOND_HALF_MASK, 0) != osOK)
    {
      _Error_Handler();
    }
  }
  SD_WriteBufferIdx[id] = dstP;
  return 0;
}



/**
* @brief  Start sensor thread
* @param  id: Sensor id
* @retval 0: no error
*/
uint8_t SDM_StartSensorThread(uint32_t id)
{
  if(id == iis3dwb_com_id)
  {
    IIS3DWB_Start();
  }
  else if(id == hts221_com_id)
  {
    HTS221_Start();
  }
  else if(id == iis2dh_com_id)
  {
    IIS2DH_Start();
  }
  else if(id == iis2mdc_com_id)
  {
    IIS2MDC_Start();
  }
  else if(id == imp34dt05_com_id)
  {
    IMP34DT05_Start();
  }
  else if(id == mp23abs1_com_id)
  {
    MP23ABS1_Start();
  }
  else if(id == ism330dhcx_com_id)
  {
    ISM330DHCX_Start();
  }
  else if(id == lps22hh_com_id)
  {
    LPS22HH_Start();
  }
  else if(id == stts751_com_id)
  {
    STTS751_Start();
  }
  sensor_first_dataReady[id] = 1;
  
  return 0;
}

/**
* @brief  Stop sensor thread
* @param  id: Sensor id
* @retval 0: no error
*/
uint8_t SDM_StopSensorThread(uint32_t id)
{
  if(id == iis3dwb_com_id)
  {
    IIS3DWB_Stop();
  }
  else if(id == hts221_com_id)
  {
    HTS221_Stop();
  }
  else if(id == iis2dh_com_id)
  {
    IIS2DH_Stop();
  }
  else if(id == iis2mdc_com_id)
  {
    IIS2MDC_Stop();
  }
  else if(id == imp34dt05_com_id)
  {
    IMP34DT05_Stop();
  }
  else if(id == mp23abs1_com_id)
  {
    MP23ABS1_Stop();
  }
  else if(id == ism330dhcx_com_id)
  {
    ISM330DHCX_Stop();
  }
  else if(id == lps22hh_com_id)
  {
    LPS22HH_Stop();
  }
  else if(id == stts751_com_id)
  {
    STTS751_Stop();
  }
  sensor_first_dataReady[id] = 0;
  return 0;
}



uint32_t SDM_ReadJSON(char *serialized_string)
{  
  static COM_Device_t JSON_device; 
  COM_Device_t *local_device; 
  uint8_t ii;
  uint32_t size;
  
  local_device = COM_GetDevice();  
  size = sizeof(COM_Device_t);
  
  memcpy(&JSON_device, local_device, size);       
  HSD_JSON_parse_Device(serialized_string, &JSON_device);      
  
  for (ii = 0; ii < JSON_device.deviceDescriptor.nSensor; ii++)
  {
    update_sensorStatus(&local_device->sensors[ii]->sensorStatus, &JSON_device.sensors[ii]->sensorStatus, ii);
  }
  
  update_sensors_config();  
  
  return 0;  
}

uint32_t SDM_CreateJSON(char **serialized_string)
{  
  COM_Device_t *device; 
  uint32_t size;
  
  device = COM_GetDevice();  
  size = HSD_JSON_serialize_Device(device, serialized_string, PRETTY_JSON);
  
  return size;
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
  case USER_BUTTON_PIN:
    if( HAL_GetTick() - t_start > 1000 )
    {
      if (com_status == HS_DATALOG_IDLE || com_status == HS_DATALOG_SD_STARTED )
      {  		
        // Cannot wait since we are in an ISR
        if(osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
        {
          _Error_Handler();
        }
        
        t_start = HAL_GetTick();
        
#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT)
        t_start = SD_Logging_Time_Start = HAL_GetTick();
        
        if (SD_Logging_Enabled == 1)
          SD_Logging_Enabled = 0;
        else 
          SD_Logging_Enabled = 1;
#endif        
      }
    }
    
  default:
    break;
  }
}


#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT)
void SDM_AutosaveFile(void)
{
  if (SD_Logging_Active)
  {
    if( (HAL_GetTick() - SD_Logging_Time_Start) > HSD_LOGGING_TIME_SECONDS_ACTIVE*1000 )
    {
      // Cannot wait since we are in an ISR
      if(osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
      {
        _Error_Handler();
      }      
      t_start = SD_Logging_Time_Start = HAL_GetTick();
    }
  }
  else
  {
    if( (HAL_GetTick() - SD_Logging_Time_Start) > HSD_LOGGING_TIME_SECONDS_IDLE*1000 )
    {
      // Cannot wait since we are in an ISR
      if(osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
      {
        _Error_Handler();
      }      
      t_start = SD_Logging_Time_Start = HAL_GetTick();
    }
  }
}
#endif    


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
