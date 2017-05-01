################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../HardwareAbstraction/src/Encoder.cpp \
../HardwareAbstraction/src/GPIO.cpp \
../HardwareAbstraction/src/PWM.cpp \
../HardwareAbstraction/src/Serial.cpp \
../HardwareAbstraction/src/Timer.cpp 

OBJS += \
./HardwareAbstraction/src/Encoder.o \
./HardwareAbstraction/src/GPIO.o \
./HardwareAbstraction/src/PWM.o \
./HardwareAbstraction/src/Serial.o \
./HardwareAbstraction/src/Timer.o 

CPP_DEPS += \
./HardwareAbstraction/src/Encoder.d \
./HardwareAbstraction/src/GPIO.d \
./HardwareAbstraction/src/PWM.d \
./HardwareAbstraction/src/Serial.d \
./HardwareAbstraction/src/Timer.d 


# Each subdirectory must supply rules for building sources it contributes
HardwareAbstraction/src/%.o: ../HardwareAbstraction/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo %cd%
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c++0x -DDEBUG -DUSE_STDPERIPH_DRIVER -DUSE_FULL_ASSERT -DSTM32F446xx -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/CMSIS/core" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/FreeRTOS/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/STM32_Driver/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/FreeRTOS/inc/portable" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/HardwareAbstraction/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/Utils/inc" -I"C:/Users/Bureau/STM32Workspace/FirmwareActionneurs/Application/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


