################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Optronik/BusOptronik.c \
../Optronik/Camera.c \
../Optronik/Lrf.c \
../Optronik/TaskManager.c 

OBJS += \
./Optronik/BusOptronik.o \
./Optronik/Camera.o \
./Optronik/Lrf.o \
./Optronik/TaskManager.o 

C_DEPS += \
./Optronik/BusOptronik.d \
./Optronik/Camera.d \
./Optronik/Lrf.d \
./Optronik/TaskManager.d 


# Each subdirectory must supply rules for building sources it contributes
Optronik/%.o: ../Optronik/%.c Optronik/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Optronik -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/bus_fdcan" -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Optronik

clean-Optronik:
	-$(RM) ./Optronik/BusOptronik.d ./Optronik/BusOptronik.o ./Optronik/Camera.d ./Optronik/Camera.o ./Optronik/Lrf.d ./Optronik/Lrf.o ./Optronik/TaskManager.d ./Optronik/TaskManager.o

.PHONY: clean-Optronik

