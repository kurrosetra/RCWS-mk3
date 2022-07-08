################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Optronik/app/bus.c \
../Optronik/app/camera.c \
../Optronik/app/lrf.c \
../Optronik/app/manager.c 

OBJS += \
./Optronik/app/bus.o \
./Optronik/app/camera.o \
./Optronik/app/lrf.o \
./Optronik/app/manager.o 

C_DEPS += \
./Optronik/app/bus.d \
./Optronik/app/camera.d \
./Optronik/app/lrf.d \
./Optronik/app/manager.d 


# Each subdirectory must supply rules for building sources it contributes
Optronik/app/%.o: ../Optronik/app/%.c Optronik/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Optronik -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan" -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Optronik-2f-app

clean-Optronik-2f-app:
	-$(RM) ./Optronik/app/bus.d ./Optronik/app/bus.o ./Optronik/app/camera.d ./Optronik/app/camera.o ./Optronik/app/lrf.d ./Optronik/app/lrf.o ./Optronik/app/manager.d ./Optronik/app/manager.o

.PHONY: clean-Optronik-2f-app

