################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RCWS/driver/bus_can/bus.c \
../RCWS/driver/bus_can/bus_ll_fdcan.c 

OBJS += \
./RCWS/driver/bus_can/bus.o \
./RCWS/driver/bus_can/bus_ll_fdcan.o 

C_DEPS += \
./RCWS/driver/bus_can/bus.d \
./RCWS/driver/bus_can/bus_ll_fdcan.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/driver/bus_can/%.o RCWS/driver/bus_can/%.su: ../RCWS/driver/bus_can/%.c RCWS/driver/bus_can/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../RCWS -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-driver-2f-bus_can

clean-RCWS-2f-driver-2f-bus_can:
	-$(RM) ./RCWS/driver/bus_can/bus.d ./RCWS/driver/bus_can/bus.o ./RCWS/driver/bus_can/bus.su ./RCWS/driver/bus_can/bus_ll_fdcan.d ./RCWS/driver/bus_can/bus_ll_fdcan.o ./RCWS/driver/bus_can/bus_ll_fdcan.su

.PHONY: clean-RCWS-2f-driver-2f-bus_can

