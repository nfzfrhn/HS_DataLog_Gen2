################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c 

OBJS += \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.o 

C_DEPS += \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32L4xx_HAL_Driver/Src/%.o: ../Drivers/STM32L4xx_HAL_Driver/Src/%.c Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DUSE_HAL_DFSDM_REGISTER_CALLBACKS -DUSE_HAL_ADC_REGISTER_CALLBACKS -DUSE_HAL_SAI_REGISTER_CALLBACKS -DUSE_HAL_SPI_REGISTER_CALLBACKS=1 -DUSE_HAL_I2C_REGISTER_CALLBACKS -DSTM32L4R9ZIJx -DSTM32 -DSTM32L4PLUS -DSTEVAL_STWINKT1 -c -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Application/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/stts751" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/hts221" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/lps22hh" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis2mdc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/ism330dhcx" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis2dh" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis3dwb" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/Common" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/STWIN" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/ST/STM32_USB_Device_Library/Class/SensorStreaming_WCID/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FatFs/src" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FatFs/src/drivers" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/parson" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/Include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32L4xx_HAL_Driver-2f-Src

clean-Drivers-2f-STM32L4xx_HAL_Driver-2f-Src:
	-$(RM) ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.o ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.d ./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.o

.PHONY: clean-Drivers-2f-STM32L4xx_HAL_Driver-2f-Src

