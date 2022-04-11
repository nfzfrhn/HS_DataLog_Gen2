################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/HSD_json.c \
../Application/User/com_manager.c \
../Application/User/device_description.c \
../Application/User/hts221_app.c \
../Application/User/iis2dh_app.c \
../Application/User/iis2mdc_app.c \
../Application/User/iis3dwb_app.c \
../Application/User/imp34dt05_app.c \
../Application/User/ism330dhcx_app.c \
../Application/User/lps22hh_app.c \
../Application/User/main.c \
../Application/User/mp23abs1_app.c \
../Application/User/rtc.c \
../Application/User/sd_diskio.c \
../Application/User/sdcard_manager.c \
../Application/User/sensors_manager.c \
../Application/User/stm32l4xx_it.c \
../Application/User/stts751_app.c \
../Application/User/system_stm32l4xx.c \
../Application/User/uart.c \
../Application/User/usbd_conf.c \
../Application/User/usbd_desc.c \
../Application/User/usbd_wcid_interface.c 

OBJS += \
./Application/User/HSD_json.o \
./Application/User/com_manager.o \
./Application/User/device_description.o \
./Application/User/hts221_app.o \
./Application/User/iis2dh_app.o \
./Application/User/iis2mdc_app.o \
./Application/User/iis3dwb_app.o \
./Application/User/imp34dt05_app.o \
./Application/User/ism330dhcx_app.o \
./Application/User/lps22hh_app.o \
./Application/User/main.o \
./Application/User/mp23abs1_app.o \
./Application/User/rtc.o \
./Application/User/sd_diskio.o \
./Application/User/sdcard_manager.o \
./Application/User/sensors_manager.o \
./Application/User/stm32l4xx_it.o \
./Application/User/stts751_app.o \
./Application/User/system_stm32l4xx.o \
./Application/User/uart.o \
./Application/User/usbd_conf.o \
./Application/User/usbd_desc.o \
./Application/User/usbd_wcid_interface.o 

C_DEPS += \
./Application/User/HSD_json.d \
./Application/User/com_manager.d \
./Application/User/device_description.d \
./Application/User/hts221_app.d \
./Application/User/iis2dh_app.d \
./Application/User/iis2mdc_app.d \
./Application/User/iis3dwb_app.d \
./Application/User/imp34dt05_app.d \
./Application/User/ism330dhcx_app.d \
./Application/User/lps22hh_app.d \
./Application/User/main.d \
./Application/User/mp23abs1_app.d \
./Application/User/rtc.d \
./Application/User/sd_diskio.d \
./Application/User/sdcard_manager.d \
./Application/User/sensors_manager.d \
./Application/User/stm32l4xx_it.d \
./Application/User/stts751_app.d \
./Application/User/system_stm32l4xx.d \
./Application/User/uart.d \
./Application/User/usbd_conf.d \
./Application/User/usbd_desc.d \
./Application/User/usbd_wcid_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/%.o Application/User/%.su: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DUSE_HAL_DFSDM_REGISTER_CALLBACKS -DUSE_HAL_ADC_REGISTER_CALLBACKS -DUSE_HAL_SAI_REGISTER_CALLBACKS -DUSE_HAL_SPI_REGISTER_CALLBACKS=1 -DUSE_HAL_I2C_REGISTER_CALLBACKS -DSTM32L4R9ZIJx -DSTM32 -DSTM32L4PLUS -DSTEVAL_STWINKT1 -c -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Application/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/stts751" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/hts221" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/lps22hh" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis2mdc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/ism330dhcx" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis2dh" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis3dwb" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/Common" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/STWIN" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/ST/STM32_USB_Device_Library/Class/SensorStreaming_WCID/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FatFs/src" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FatFs/src/drivers" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/parson" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/Include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/HSD_json.d ./Application/User/HSD_json.o ./Application/User/HSD_json.su ./Application/User/com_manager.d ./Application/User/com_manager.o ./Application/User/com_manager.su ./Application/User/device_description.d ./Application/User/device_description.o ./Application/User/device_description.su ./Application/User/hts221_app.d ./Application/User/hts221_app.o ./Application/User/hts221_app.su ./Application/User/iis2dh_app.d ./Application/User/iis2dh_app.o ./Application/User/iis2dh_app.su ./Application/User/iis2mdc_app.d ./Application/User/iis2mdc_app.o ./Application/User/iis2mdc_app.su ./Application/User/iis3dwb_app.d ./Application/User/iis3dwb_app.o ./Application/User/iis3dwb_app.su ./Application/User/imp34dt05_app.d ./Application/User/imp34dt05_app.o ./Application/User/imp34dt05_app.su ./Application/User/ism330dhcx_app.d ./Application/User/ism330dhcx_app.o ./Application/User/ism330dhcx_app.su ./Application/User/lps22hh_app.d ./Application/User/lps22hh_app.o ./Application/User/lps22hh_app.su ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/mp23abs1_app.d ./Application/User/mp23abs1_app.o ./Application/User/mp23abs1_app.su ./Application/User/rtc.d ./Application/User/rtc.o ./Application/User/rtc.su ./Application/User/sd_diskio.d ./Application/User/sd_diskio.o ./Application/User/sd_diskio.su ./Application/User/sdcard_manager.d ./Application/User/sdcard_manager.o ./Application/User/sdcard_manager.su ./Application/User/sensors_manager.d ./Application/User/sensors_manager.o ./Application/User/sensors_manager.su ./Application/User/stm32l4xx_it.d ./Application/User/stm32l4xx_it.o ./Application/User/stm32l4xx_it.su ./Application/User/stts751_app.d ./Application/User/stts751_app.o ./Application/User/stts751_app.su ./Application/User/system_stm32l4xx.d ./Application/User/system_stm32l4xx.o ./Application/User/system_stm32l4xx.su ./Application/User/uart.d ./Application/User/uart.o ./Application/User/uart.su ./Application/User/usbd_conf.d ./Application/User/usbd_conf.o ./Application/User/usbd_conf.su ./Application/User/usbd_desc.d ./Application/User/usbd_desc.o ./Application/User/usbd_desc.su ./Application/User/usbd_wcid_interface.d ./Application/User/usbd_wcid_interface.o ./Application/User/usbd_wcid_interface.su

.PHONY: clean-Application-2f-User

