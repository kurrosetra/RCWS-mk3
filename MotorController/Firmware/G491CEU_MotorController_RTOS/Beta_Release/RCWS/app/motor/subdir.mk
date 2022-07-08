################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RCWS/app/motor/homing.c \
../RCWS/app/motor/manual.c \
../RCWS/app/motor/memory.c \
../RCWS/app/motor/stab.c \
../RCWS/app/motor/t_motor.c \
../RCWS/app/motor/track.c 

OBJS += \
./RCWS/app/motor/homing.o \
./RCWS/app/motor/manual.o \
./RCWS/app/motor/memory.o \
./RCWS/app/motor/stab.o \
./RCWS/app/motor/t_motor.o \
./RCWS/app/motor/track.o 

C_DEPS += \
./RCWS/app/motor/homing.d \
./RCWS/app/motor/manual.d \
./RCWS/app/motor/memory.d \
./RCWS/app/motor/stab.d \
./RCWS/app/motor/t_motor.d \
./RCWS/app/motor/track.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/app/motor/%.o RCWS/app/motor/%.su: ../RCWS/app/motor/%.c RCWS/app/motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../RCWS -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-app-2f-motor

clean-RCWS-2f-app-2f-motor:
	-$(RM) ./RCWS/app/motor/homing.d ./RCWS/app/motor/homing.o ./RCWS/app/motor/homing.su ./RCWS/app/motor/manual.d ./RCWS/app/motor/manual.o ./RCWS/app/motor/manual.su ./RCWS/app/motor/memory.d ./RCWS/app/motor/memory.o ./RCWS/app/motor/memory.su ./RCWS/app/motor/stab.d ./RCWS/app/motor/stab.o ./RCWS/app/motor/stab.su ./RCWS/app/motor/t_motor.d ./RCWS/app/motor/t_motor.o ./RCWS/app/motor/t_motor.su ./RCWS/app/motor/track.d ./RCWS/app/motor/track.o ./RCWS/app/motor/track.su

.PHONY: clean-RCWS-2f-app-2f-motor

