################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/homing.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/manual.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/memory.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/stab.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/t_motor.c \
D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/track.c 

OBJS += \
./RCWS/app/movement/homing.o \
./RCWS/app/movement/manual.o \
./RCWS/app/movement/memory.o \
./RCWS/app/movement/stab.o \
./RCWS/app/movement/t_motor.o \
./RCWS/app/movement/track.o 

C_DEPS += \
./RCWS/app/movement/homing.d \
./RCWS/app/movement/manual.d \
./RCWS/app/movement/memory.d \
./RCWS/app/movement/stab.d \
./RCWS/app/movement/t_motor.d \
./RCWS/app/movement/track.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/app/movement/homing.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/homing.c RCWS/app/movement/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
RCWS/app/movement/manual.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/manual.c RCWS/app/movement/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
RCWS/app/movement/memory.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/memory.c RCWS/app/movement/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
RCWS/app/movement/stab.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/stab.c RCWS/app/movement/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
RCWS/app/movement/t_motor.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/t_motor.c RCWS/app/movement/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
RCWS/app/movement/track.o: D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS/app/movement/track.c RCWS/app/movement/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/MotorController/Firmware/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-app-2f-movement

clean-RCWS-2f-app-2f-movement:
	-$(RM) ./RCWS/app/movement/homing.d ./RCWS/app/movement/homing.o ./RCWS/app/movement/homing.su ./RCWS/app/movement/manual.d ./RCWS/app/movement/manual.o ./RCWS/app/movement/manual.su ./RCWS/app/movement/memory.d ./RCWS/app/movement/memory.o ./RCWS/app/movement/memory.su ./RCWS/app/movement/stab.d ./RCWS/app/movement/stab.o ./RCWS/app/movement/stab.su ./RCWS/app/movement/t_motor.d ./RCWS/app/movement/t_motor.o ./RCWS/app/movement/t_motor.su ./RCWS/app/movement/track.d ./RCWS/app/movement/track.o ./RCWS/app/movement/track.su

.PHONY: clean-RCWS-2f-app-2f-movement

