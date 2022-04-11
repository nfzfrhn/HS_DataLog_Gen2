################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/iis2dh/iis2dh.c \
../Drivers/BSP/Components/iis2dh/iis2dh_reg.c 

OBJS += \
./Drivers/BSP/Components/iis2dh/iis2dh.o \
./Drivers/BSP/Components/iis2dh/iis2dh_reg.o 

C_DEPS += \
./Drivers/BSP/Components/iis2dh/iis2dh.d \
./Drivers/BSP/Components/iis2dh/iis2dh_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/iis2dh/%.o Drivers/BSP/Components/iis2dh/%.su: ../Drivers/BSP/Components/iis2dh/%.c Drivers/BSP/Components/iis2dh/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DUSE_HAL_DFSDM_REGISTER_CALLBACKS -DUSE_HAL_ADC_REGISTER_CALLBACKS -DUSE_HAL_SAI_REGISTER_CALLBACKS -DUSE_HAL_SPI_REGISTER_CALLBACKS=1 -DUSE_HAL_I2C_REGISTER_CALLBACKS -DSTM32L4R9ZIJx -DSTM32 -DSTM32L4PLUS -DSTEVAL_STWINKT1 -c -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Application/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/stts751" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/hts221" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/lps22hh" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis2mdc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/ism330dhcx" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis2dh" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/iis3dwb" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/Components/Common" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/BSP/STWIN" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/ST/STM32_USB_Device_Library/Class/SensorStreaming_WCID/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FatFs/src" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FatFs/src/drivers" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/FreeRTOS/Source" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Middlewares/Third_Party/parson" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/Include" -I"C:/Users/Nafiz/Documents/STM_Cube_Workspace/HS_DataLog_Gen2/Drivers/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-iis2dh

clean-Drivers-2f-BSP-2f-Components-2f-iis2dh:
	-$(RM) ./Drivers/BSP/Components/iis2dh/iis2dh.d ./Drivers/BSP/Components/iis2dh/iis2dh.o ./Drivers/BSP/Components/iis2dh/iis2dh.su ./Drivers/BSP/Components/iis2dh/iis2dh_reg.d ./Drivers/BSP/Components/iis2dh/iis2dh_reg.o ./Drivers/BSP/Components/iis2dh/iis2dh_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-iis2dh

