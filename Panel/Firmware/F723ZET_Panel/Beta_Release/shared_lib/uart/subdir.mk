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
./shared_lib/uart/retarget.o \
./shared_lib/uart/ring_buffer.o \
./shared_lib/uart/stm_hal_serial.o 

C_DEPS += \
./shared_lib/uart/retarget.d \
./shared_lib/uart/ring_buffer.d \
./shared_lib/uart/stm_hal_serial.d 


# Each subdirectory must supply rules for building sources it contributes
shared_lib/uart/retarget.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/retarget.c shared_lib/uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g1 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F723xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/bus_can" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/button" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/sla" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
shared_lib/uart/ring_buffer.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/ring_buffer.c shared_lib/uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g1 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F723xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/bus_can" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/button" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/sla" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
shared_lib/uart/stm_hal_serial.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart/stm_hal_serial.c shared_lib/uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g1 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F723xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/bus_can" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/button" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/sla" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-shared_lib-2f-uart

clean-shared_lib-2f-uart:
	-$(RM) ./shared_lib/uart/retarget.d ./shared_lib/uart/retarget.o ./shared_lib/uart/retarget.su ./shared_lib/uart/ring_buffer.d ./shared_lib/uart/ring_buffer.o ./shared_lib/uart/ring_buffer.su ./shared_lib/uart/stm_hal_serial.d ./shared_lib/uart/stm_hal_serial.o ./shared_lib/uart/stm_hal_serial.su

.PHONY: clean-shared_lib-2f-uart

