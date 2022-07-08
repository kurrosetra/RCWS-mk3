################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Optronik/hal/cam_day.c \
../Optronik/hal/cam_thermal.c \
../Optronik/hal/hal_bus.c \
../Optronik/hal/lrf_hal.c 

OBJS += \
./Optronik/hal/cam_day.o \
./Optronik/hal/cam_thermal.o \
./Optronik/hal/hal_bus.o \
./Optronik/hal/lrf_hal.o 

C_DEPS += \
./Optronik/hal/cam_day.d \
./Optronik/hal/cam_thermal.d \
./Optronik/hal/hal_bus.d \
./Optronik/hal/lrf_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Optronik/hal/%.o: ../Optronik/hal/%.c Optronik/hal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Optronik -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan" -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Optronik-2f-hal

clean-Optronik-2f-hal:
	-$(RM) ./Optronik/hal/cam_day.d ./Optronik/hal/cam_day.o ./Optronik/hal/cam_thermal.d ./Optronik/hal/cam_thermal.o ./Optronik/hal/hal_bus.d ./Optronik/hal/hal_bus.o ./Optronik/hal/lrf_hal.d ./Optronik/hal/lrf_hal.o

.PHONY: clean-Optronik-2f-hal

