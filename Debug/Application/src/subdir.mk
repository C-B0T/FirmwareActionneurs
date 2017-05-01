################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/src/syscalls.c 

CPP_SRCS += \
../Application/src/main.cpp 

OBJS += \
./Application/src/main.o \
./Application/src/syscalls.o 

C_DEPS += \
./Application/src/syscalls.d 

CPP_DEPS += \
./Application/src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Application/src/%.o: ../Application/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo %cd%
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c++0x -DDEBUG -DUSE_STDPERIPH_DRIVER -DUSE_FULL_ASSERT -DSTM32F446xx -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/CMSIS/core" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/FreeRTOS/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/STM32_Driver/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/FreeRTOS/inc/portable" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/HardwareAbstraction/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/Utils/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/Application/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/src/%.o: ../Application/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DDEBUG -DUSE_STDPERIPH_DRIVER -DUSE_FULL_ASSERT -DSTM32F446xx -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/CMSIS/core" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/FreeRTOS/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/STM32_Driver/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/FreeRTOS/inc/portable" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/HardwareAbstraction/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/Utils/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/Application/inc" -O0 -g3 -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


