/**
******************************************************************************
* @file    main.c
* @author  SRA - Central Labs
* @version v2.1.1
* @date    26-Feb-2020
* @brief   Main program body
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
#include "main.h"
#include "ff.h"			//Declaration of FatFs API

/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
extern volatile COM_Device_t COM_device;

SD_HandleTypeDef hsd1;

uint8_t iis3dwb_com_id;
uint8_t hts221_com_id;
uint8_t iis2dh_com_id;
uint8_t iis2mdc_com_id;
uint8_t imp34dt05_com_id;
uint8_t mp23abs1_com_id;
uint8_t ism330dhcx_com_id;
uint8_t lps22hh_com_id;
uint8_t stts751_com_id;

volatile float iis3dwb_sampling_freq;
volatile float hts221_sampling_freq;
volatile float iis2dh_sampling_freq;
volatile float iis2mdc_sampling_freq;
volatile float imp34dt05_sampling_freq;
volatile float mp23abs1_sampling_freq;
volatile float ism330dhcx_sampling_freq;
volatile float lps22hh_sampling_freq;
volatile float stts751_sampling_freq;
volatile float imp34dt05_sampling_freq1;

float sensor_n_samples_acc[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
double old_time_stamp[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

uint8_t sensor_first_dataReady[] =  {1,1,1,1,1,1,1,1,1,1,1};
uint16_t sensor_n_samples_to_timestamp[] =  {0,0,0,0,0,0,0,0,0,0,0,0};

/* ****** */

extern uint8_t SD_Logging_Active;
extern uint8_t SD_present;
uint32_t maxWriteTimeSensor[COM_MAX_SENSORS];

EXTI_HandleTypeDef BC_exti;
volatile uint32_t t_stwin = 0;

extern uint32_t t_start;
UART_HandleTypeDef huart2;
SD_HandleTypeDef hsd1;
uint8_t * p = 0;

#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT)
extern uint32_t SD_Logging_Time_Start;
extern uint32_t SD_Logging_Enabled;
#endif

/* Private function prototypes -----------------------------------------------*/
static void Peripheral_MSP_Init_All(void);
static void Peripheral_OS_Init_All(void);
static void BattChrg_Init(void);
static void BC_Int_Callback(void);
static void Error_Handler(void);
static void MX_USART2_Init(void);
void PVD_Config(void);
void SystemClock_Config(void);
static void MX_SDMMC1_SD_Init(void);

//Battery SOC var
uint32_t mvLevel = 0;
uint32_t batteryLevel = 0;

/**
* @brief  Main program
* @param  None
* @retval None
*/  
int main(void)
{



	HAL_Init();

	SystemClock_Config();
    
	/* Enable Power Clock for low power modes*/
	__HAL_RCC_PWR_CLK_ENABLE();
	MX_SDMMC1_SD_Init();
	//HAL_PWREx_EnableVddUSB();		/* USB */
	//HAL_PWREx_EnableVddIO2();		/* VddIO2 is for Port G, since we are not using it */
	//BSP_Enable_DCDC2();				//Only for I2C 3, WiFi
  
	/* Configure the Battery Charger */
	BattChrg_Init();
  
	/* Configure Power Voltage Detector(PVD) to detect if battery voltage is low */
	PVD_Config();
  
	/* Configure DEBUG PIN and LED */
	//BSP_DEBUG_PIN_Init_All();
	BSP_LED_Init(LED1);				//Green
	BSP_LED_Init(LED2);				//Orange
	__HAL_RCC_GPIOA_CLK_ENABLE();		//PA13, 14 are for DEBUG
   
	HSD_JSON_set_allocation_functions(HSD_malloc, HSD_free);
  
  /* Start USB */  
  //MX_USB_DEVICE_Init();			//Dont know why this is disabled
  
  /* Set default device description */
  set_default_description();
  
  /* USER Button initialization */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  BSP_PB_PWR_Init();
  
  /* Sensor Manager initilization, only using PID (Platform Indepentent Drivers) */
  SM_Peripheral_Init();
  SM_OS_Init();
  
  MX_USART2_Init();
  Peripheral_MSP_Init_All();  
  SDM_Peripheral_Init();
  
  Peripheral_OS_Init_All();  
  SDM_OS_Init();
  
  //Show reset
  char wakeUp [] = {"MCU WAKE UP"};
  HAL_UART_Transmit(&huart2, (uint8_t *) wakeUp, sizeof(wakeUp), HAL_MAX_DELAY);

  //Get SOC
  BSP_BC_GetVoltageAndLevel(&mvLevel, &batteryLevel);

  //Show SOC
  char soc[21] = {"SOC in % :"};
  char batVal[5];
  itoa(batteryLevel, batVal,10);
  strcat(soc, batVal);
  HAL_UART_Transmit(&huart2, (uint8_t *) soc, sizeof(soc), HAL_MAX_DELAY);

  /* Start scheduler */
  osKernelStart();
  
  while(1);  
}


/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}


//UART2 INIT
void MX_USART2_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

}


void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
 // UNUSED(huart);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_USART2_CLK_ENABLE();
    //__HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD6     ------> USART2_RX
    PD4     ------> USART2_RTS
    PD5     ------> USART2_TX
    */
    GPIO_InitStruct.Pin = USART_CR2_RXINV|USART2_RTS_Pin|USART2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_MspInit can be implemented in the user file
   */
}

/**
* Init USB device Library, add supported class and start the library
* @retval None
*/
void MX_USB_DEVICE_Init(void)
{
  USBD_Init(&USBD_Device, &WCID_STREAMING_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_WCID_STREAMING_CLASS);
  /* Add Interface callbacks for AUDIO and CDC Class */
  USBD_WCID_STREAMING_RegisterInterface(&USBD_Device, &USBD_WCID_STREAMING_fops);
  /* Start Device Process */  
  USBD_Start(&USBD_Device);
}


/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;	//Changed
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;					//Changed
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;				//Changed
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
#if 0
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C2
    |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB|RCC_PERIPHCLK_SDMMC1
      |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_DFSDM1AUDIO;
#else
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
#endif

#if 0
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
#endif

  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;

#if 0
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.Dfsdm1AudioClockSelection = RCC_DFSDM1AUDIOCLKSOURCE_SAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
#endif

  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLP;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

#if 1
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 5;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 96;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV25;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
#endif

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }  
}


void BattChrg_Init(void)
{
  BSP_BC_Init();
  BSP_BC_BatMS_Init();
  BSP_BC_CmdSend(BATMS_ON);  
  
  HAL_EXTI_GetHandle(&BC_exti, EXTI_LINE_10);  
  HAL_EXTI_RegisterCallback(&BC_exti,  HAL_EXTI_COMMON_CB_ID, BC_Int_Callback);
  
  t_stwin = HAL_GetTick(); 
}

/**
* @brief  Battery Charger Interrupt callback
* @param  None
* @retval None
*/
void BC_Int_Callback(void)
{
  if(HAL_GetTick() - t_stwin > 4000)
  {
    BSP_BC_CmdSend(SHIPPING_MODE_ON);
  }
}


/**
* @brief  Configures the PVD resources.
* @param  None
* @retval None
*/
void PVD_Config(void)
{
  PWR_PVDTypeDef sConfigPVD;
  
  /*##-1- Enable Power Clock #################################################*/
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /*##-2- Configure the NVIC for PVD #########################################*/
  HAL_NVIC_SetPriority(PVD_PVM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);
  
  /* Configure the PVD Level to 6 and generate an interrupt on falling
  edge(PVD detection level set to 2.9V, refer to the electrical characteristics
  of you device datasheet for more details) */
  sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;
  sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING;
  HAL_PWR_ConfigPVD(&sConfigPVD);
  
  /* Enable the PVD Output */
  HAL_PWR_EnablePVD();
}


void vApplicationIdleHook( void )
{
	//Starts measurement manually


#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT) 
	//If SD_Logging enabled start measurement
  if(SD_Logging_Enabled)
  {
    SDM_AutosaveFile();
  }
#endif
  
  if (!SD_Logging_Active)
  {
    if(com_status == HS_DATALOG_USB_STARTED)
    {
        if(!(HAL_GetTick()%100)) 
        {
          BSP_LED_On(LED_ORANGE);
        }
        else 
        {
          if(!(HAL_GetTick()%50)) 
          {
            BSP_LED_Off(LED_ORANGE);
          }
        }        
    }
    else
    {
      if (!BSP_SD_IsDetected())
      {
        if(!(HAL_GetTick()%200)) 
        {
          BSP_LED_On(LED_ORANGE);
        }
        else 
        {
          if(!(HAL_GetTick()%100)) 
          {
            BSP_LED_Off(LED_ORANGE);
          }
        }    
      }
      else
      {
        if(!(HAL_GetTick()%1000)) 
        {
          BSP_LED_On(LED_ORANGE);
        }
        else 
        {
          if(!(HAL_GetTick()%50)) 
          {
            BSP_LED_Off(LED_ORANGE);
          }
        }
      }
    }
    //Start Measurement manually
    //Show reset
    SDM_StartMeasurements();
  }
}

//RTC
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{

}


/**
* @brief  Sensor Data Ready generic callback. Takes the latest data coming from
*         a sensor and send it to the active communication interface.
* @param  sId: Sensor Id
* @param  buf: input data buffer
* @param  size: input data buffer size [bytes]
* @param  timeStamp: timestamp of the latest sample in the input buffer
* @retval 
*/
void SENSOR_Generic_Data_Ready(uint16_t sId, uint8_t *buf, uint16_t size, double timeStamp)
{ 
  COM_SensorStatus_t * tempStatus = COM_GetSensorStatus(sId);    
  COM_SensorDescriptor_t * tempDescriptor = COM_GetSensorDescriptor(sId); 
  COM_SubSensorDescriptor_t *tempSubSensorDescriptor; 
  uint16_t toSend = 0, nBytesPerSample = 0;
  
  if (sensor_first_dataReady[sId]) // Discard first set of sensor data
  {
    sensor_first_dataReady[sId] = 0;    
    sensor_n_samples_acc[sId] = 0.0f;    
    tempStatus->initialOffset = (float)timeStamp;
    old_time_stamp[sId] = timeStamp;
    sensor_n_samples_to_timestamp[sId] = tempStatus->samplesPerTimestamp;
  }
  else
  {    
    if(tempDescriptor->dataType == DATA_TYPE_FLOAT || tempDescriptor->dataType == DATA_TYPE_INT32 || tempDescriptor->dataType == DATA_TYPE_UINT32) 
    {
      nBytesPerSample = 4;
    }
    else if(tempDescriptor->dataType == DATA_TYPE_UINT16 || tempDescriptor->dataType == DATA_TYPE_INT16) 
    {
      nBytesPerSample = 2;
    }
    else if(tempDescriptor->dataType == DATA_TYPE_UINT8 || tempDescriptor->dataType == DATA_TYPE_INT8) 
    {
      nBytesPerSample = 1;
    }
    
    uint8_t totalDataPerSample = 0;
    
    for (int i = 0; i < tempDescriptor->nSubSensors; i++)
    {
      if (tempStatus->subSensorStatus[i].isActive)
      {
        tempSubSensorDescriptor = COM_GetSubSensorDescriptor(sId, i);
        totalDataPerSample += tempSubSensorDescriptor->dataPerSample;     
      }
    }
    
    nBytesPerSample *= totalDataPerSample;
    sensor_n_samples_acc[sId] = (float)(size / nBytesPerSample);
    tempStatus->measuredODR = sensor_n_samples_acc[sId]/(timeStamp - old_time_stamp[sId]);
    old_time_stamp[sId] = timeStamp;
    toSend = size/nBytesPerSample;
    
    while(toSend > 0)
    {
      if(toSend < sensor_n_samples_to_timestamp[sId] || sensor_n_samples_to_timestamp[sId] == 0)
      {
        if(com_status == HS_DATALOG_SD_STARTED)
        {
          SDM_Fill_Buffer(sId , (uint8_t *)buf, toSend * nBytesPerSample);
        }
        else if (com_status == HS_DATALOG_USB_STARTED )
        {
          USBD_WCID_STREAMING_FillTxDataBuffer(&USBD_Device, tempStatus->comChannelNumber  , (uint8_t *)buf, toSend * nBytesPerSample);
        }
        if(sensor_n_samples_to_timestamp[sId] != 0)
        {
          sensor_n_samples_to_timestamp[sId] -= toSend;
        }        
        toSend = 0;
      }
      else
      {
        if(com_status == HS_DATALOG_SD_STARTED)
        {
          SDM_Fill_Buffer(sId, (uint8_t *)buf, sensor_n_samples_to_timestamp[sId] * nBytesPerSample);
        }
        else if (com_status == HS_DATALOG_USB_STARTED )
        {
          USBD_WCID_STREAMING_FillTxDataBuffer(&USBD_Device, tempStatus->comChannelNumber, (uint8_t *)buf, sensor_n_samples_to_timestamp[sId] * nBytesPerSample);
        }

        buf+=sensor_n_samples_to_timestamp[sId] * nBytesPerSample;
        toSend -= sensor_n_samples_to_timestamp[sId];

        double newTS = timeStamp - ((1.0 / (double)tempStatus->measuredODR) * toSend);   
        if(com_status == HS_DATALOG_SD_STARTED)
        {
          SDM_Fill_Buffer(sId, (uint8_t *)&newTS, 8);
        }
        else if (com_status == HS_DATALOG_USB_STARTED )
        {
          USBD_WCID_STREAMING_FillTxDataBuffer(&USBD_Device, tempStatus->comChannelNumber, (uint8_t *)&newTS, 8);
        }
        sensor_n_samples_to_timestamp[sId] = tempStatus->samplesPerTimestamp;  
      }      
    }    
  }    
}


/*  ---------- Sensors data ready functions ----------- */
void IIS3DWB_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{  
  SENSOR_Generic_Data_Ready(iis3dwb_com_id, buf, size, timeStamp);
}

void HTS221_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(hts221_com_id, buf, size, timeStamp);
}

void IIS2DH_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(iis2dh_com_id, buf, size, timeStamp);
}

void IIS2MDC_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(iis2mdc_com_id, buf, size, timeStamp);
}

void IMP34DT05_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(imp34dt05_com_id, buf, size, timeStamp);
}

void ISM330DHCX_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(ism330dhcx_com_id, buf, size, timeStamp);
}

void LPS22HH_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(lps22hh_com_id, buf, size, timeStamp);
}

void MP23ABS1_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(mp23abs1_com_id, buf, size, timeStamp);
}

void STTS751_Data_Ready(uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(stts751_com_id, buf, size, timeStamp);
}



void Peripheral_MSP_Init_All(void)
{  
#if 0
  IIS2MDC_Peripheral_Init();
  STTS751_Peripheral_Init();
  LPS22HH_Peripheral_Init();
  HTS221_Peripheral_Init();
  IMP34DT05_Peripheral_Init();
  MP23ABS1_Peripheral_Init();
  ISM330DHCX_Peripheral_Init();
#endif
  IIS3DWB_Peripheral_Init();
#if 0
  IIS2DH_Peripheral_Init();
#endif
}


void Peripheral_OS_Init_All(void)
{
#if 0
  IIS2MDC_OS_Init();
  STTS751_OS_Init();
  LPS22HH_OS_Init();
  HTS221_OS_Init();
  IMP34DT05_OS_Init();
  MP23ABS1_OS_Init();
  ISM330DHCX_OS_Init();
#endif
  IIS3DWB_OS_Init();
#if 0
  IIS2DH_OS_Init();
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
	  case USER_BUTTON_PIN:
		  userButtonCallback(GPIO_Pin);
	  case  IIS3DWB_INT1_Pin:
		  UpdateTimerValue();

	  default:
		break;
  }
}

/**
* @brief  This function is executed in case of error occurrence
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
  while(1);
}



#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
