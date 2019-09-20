################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adxl345.c \
../Src/dma.c \
../Src/freertos.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_tim.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system.c \
../Src/system_stm32f4xx.c \
../Src/usart.c 

OBJS += \
./Src/adxl345.o \
./Src/dma.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_tim.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system.o \
./Src/system_stm32f4xx.o \
./Src/usart.o 

C_DEPS += \
./Src/adxl345.d \
./Src/dma.d \
./Src/freertos.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_tim.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system.d \
./Src/system_stm32f4xx.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F429xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -I"/home/edxian/Desktop/integrate/Inc" -I"/home/edxian/Desktop/integrate/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/edxian/Desktop/integrate/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/edxian/Desktop/integrate/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/edxian/Desktop/integrate/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/edxian/Desktop/integrate/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/edxian/Desktop/integrate/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/edxian/Desktop/integrate/Drivers/CMSIS/Include" -I"/home/edxian/Desktop/integrate/Middlewares/Third_Party/c_library_v1-master/c_library_v1-master/common" -I"/home/edxian/Desktop/integrate/Middlewares/Third_Party/c_library_v1-master/c_library_v1-master"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


