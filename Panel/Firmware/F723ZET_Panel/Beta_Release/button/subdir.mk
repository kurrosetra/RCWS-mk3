################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../button/button.c 

OBJS += \
./button/button.o 

C_DEPS += \
./button/button.d 


# Each subdirectory must supply rules for building sources it contributes
button/%.o button/%.su: ../button/%.c button/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g1 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F723xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/uart" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/bus_can" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/button" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Panel/Firmware/F723ZET_Panel/sla" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-button

clean-button:
	-$(RM) ./button/button.d ./button/button.o ./button/button.su

.PHONY: clean-button

