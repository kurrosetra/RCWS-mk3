################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/driver/weapon/kokang.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/driver/weapon/munisi.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/driver/weapon/trigger.c 

OBJS += \
./RCWS/driver/weapon/kokang.o \
./RCWS/driver/weapon/munisi.o \
./RCWS/driver/weapon/trigger.o 

C_DEPS += \
./RCWS/driver/weapon/kokang.d \
./RCWS/driver/weapon/munisi.d \
./RCWS/driver/weapon/trigger.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/driver/weapon/kokang.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/driver/weapon/kokang.c RCWS/driver/weapon/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
RCWS/driver/weapon/munisi.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/driver/weapon/munisi.c RCWS/driver/weapon/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
RCWS/driver/weapon/trigger.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/driver/weapon/trigger.c RCWS/driver/weapon/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-driver-2f-weapon

clean-RCWS-2f-driver-2f-weapon:
	-$(RM) ./RCWS/driver/weapon/kokang.d ./RCWS/driver/weapon/kokang.o ./RCWS/driver/weapon/kokang.su ./RCWS/driver/weapon/munisi.d ./RCWS/driver/weapon/munisi.o ./RCWS/driver/weapon/munisi.su ./RCWS/driver/weapon/trigger.d ./RCWS/driver/weapon/trigger.o ./RCWS/driver/weapon/trigger.su

.PHONY: clean-RCWS-2f-driver-2f-weapon

