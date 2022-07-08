################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan/bus_fdcan.c 

OBJS += \
./bus_fdcan/bus_fdcan.o 

C_DEPS += \
./bus_fdcan/bus_fdcan.d 


# Each subdirectory must supply rules for building sources it contributes
bus_fdcan/bus_fdcan.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan/bus_fdcan.c bus_fdcan/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../MovementMode -I../busPanel -I../Core/Inc -I../PC -I../Button -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bus_fdcan

clean-bus_fdcan:
	-$(RM) ./bus_fdcan/bus_fdcan.d ./bus_fdcan/bus_fdcan.o ./bus_fdcan/bus_fdcan.su

.PHONY: clean-bus_fdcan

