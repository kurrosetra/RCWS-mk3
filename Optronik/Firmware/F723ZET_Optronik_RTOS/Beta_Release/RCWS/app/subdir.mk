################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RCWS/app/bus.c \
../RCWS/app/camera.c \
../RCWS/app/lrf.c \
../RCWS/app/manager.c 

OBJS += \
./RCWS/app/bus.o \
./RCWS/app/camera.o \
./RCWS/app/lrf.o \
./RCWS/app/manager.o 

C_DEPS += \
./RCWS/app/bus.d \
./RCWS/app/camera.d \
./RCWS/app/lrf.d \
./RCWS/app/manager.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/app/%.o RCWS/app/%.su: ../RCWS/app/%.c RCWS/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F723xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Optronik/Firmware/F723ZET_Optronik_RTOS/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-app

clean-RCWS-2f-app:
	-$(RM) ./RCWS/app/bus.d ./RCWS/app/bus.o ./RCWS/app/bus.su ./RCWS/app/camera.d ./RCWS/app/camera.o ./RCWS/app/camera.su ./RCWS/app/lrf.d ./RCWS/app/lrf.o ./RCWS/app/lrf.su ./RCWS/app/manager.d ./RCWS/app/manager.o ./RCWS/app/manager.su

.PHONY: clean-RCWS-2f-app

