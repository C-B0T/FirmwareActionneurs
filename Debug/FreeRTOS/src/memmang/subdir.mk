################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/src/memmang/heap_4.c 

OBJS += \
./FreeRTOS/src/memmang/heap_4.o 

C_DEPS += \
./FreeRTOS/src/memmang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/src/memmang/%.o: ../FreeRTOS/src/memmang/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DDEBUG -DUSE_STDPERIPH_DRIVER -DUSE_FULL_ASSERT -DSTM32F446xx -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/CMSIS/core" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/FreeRTOS/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/STM32_Driver/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/FreeRTOS/inc/portable" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/HardwareAbstraction/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/Utils/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/Application/inc" -O0 -g3 -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


