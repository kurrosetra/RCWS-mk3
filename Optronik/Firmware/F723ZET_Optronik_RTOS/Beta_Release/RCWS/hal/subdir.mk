################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RCWS/hal/cam_day.c \
../RCWS/hal/cam_thermal.c \
../RCWS/hal/hal_bus.c \
../RCWS/hal/lrf_hal.c 

OBJS += \
./RCWS/hal/cam_day.o \
./RCWS/hal/cam_thermal.o \
./RCWS/hal/hal_bus.o \
./RCWS/hal/lrf_hal.o 

C_DEPS += \
./RCWS/hal/cam_day.d \
./RCWS/hal/cam_thermal.d \
./RCWS/hal/hal_bus.d \
./RCWS/hal/lrf_hal.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/hal/%.o RCWS/hal/%.su: ../RCWS/hal/%.c RCWS/hal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F723xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Optronik/Firmware/F723ZET_Optronik_RTOS/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-hal

clean-RCWS-2f-hal:
	-$(RM) ./RCWS/hal/cam_day.d ./RCWS/hal/cam_day.o ./RCWS/hal/cam_day.su ./RCWS/hal/cam_thermal.d ./RCWS/hal/cam_thermal.o ./RCWS/hal/cam_thermal.su ./RCWS/hal/hal_bus.d ./RCWS/hal/hal_bus.o ./RCWS/hal/hal_bus.su ./RCWS/hal/lrf_hal.d ./RCWS/hal/lrf_hal.o ./RCWS/hal/lrf_hal.su

.PHONY: clean-RCWS-2f-hal

