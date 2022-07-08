################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/retarget.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/ring_buffer.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/stm_hal_serial.c 

OBJS += \
./uart/retarget.o \
./uart/ring_buffer.o \
./uart/stm_hal_serial.o 

C_DEPS += \
./uart/retarget.d \
./uart/ring_buffer.d \
./uart/stm_hal_serial.d 


# Each subdirectory must supply rules for building sources it contributes
uart/retarget.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/retarget.c uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../MovementMode -I../busPanel -I../Core/Inc -I../PC -I../Button -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
uart/ring_buffer.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/ring_buffer.c uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../MovementMode -I../busPanel -I../Core/Inc -I../PC -I../Button -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
uart/stm_hal_serial.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/stm_hal_serial.c uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../MovementMode -I../busPanel -I../Core/Inc -I../PC -I../Button -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-uart

clean-uart:
	-$(RM) ./uart/retarget.d ./uart/retarget.o ./uart/retarget.su ./uart/ring_buffer.d ./uart/ring_buffer.o ./uart/ring_buffer.su ./uart/stm_hal_serial.d ./uart/stm_hal_serial.o ./uart/stm_hal_serial.su

.PHONY: clean-uart

