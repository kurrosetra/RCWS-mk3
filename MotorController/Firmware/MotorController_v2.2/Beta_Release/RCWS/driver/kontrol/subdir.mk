################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/driver/kontrol/track_control.c 

OBJS += \
./RCWS/driver/kontrol/track_control.o 

C_DEPS += \
./RCWS/driver/kontrol/track_control.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/driver/kontrol/track_control.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/driver/kontrol/track_control.c RCWS/driver/kontrol/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-driver-2f-kontrol

clean-RCWS-2f-driver-2f-kontrol:
	-$(RM) ./RCWS/driver/kontrol/track_control.d ./RCWS/driver/kontrol/track_control.o ./RCWS/driver/kontrol/track_control.su

.PHONY: clean-RCWS-2f-driver-2f-kontrol

