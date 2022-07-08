################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RCWS/driver/ingenia/ingenia.c \
../RCWS/driver/ingenia/ingenia_buffer.c \
../RCWS/driver/ingenia/ingenia_ll_fdcan.c 

OBJS += \
./RCWS/driver/ingenia/ingenia.o \
./RCWS/driver/ingenia/ingenia_buffer.o \
./RCWS/driver/ingenia/ingenia_ll_fdcan.o 

C_DEPS += \
./RCWS/driver/ingenia/ingenia.d \
./RCWS/driver/ingenia/ingenia_buffer.d \
./RCWS/driver/ingenia/ingenia_ll_fdcan.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/driver/ingenia/%.o RCWS/driver/ingenia/%.su: ../RCWS/driver/ingenia/%.c RCWS/driver/ingenia/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../RCWS -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O0 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-driver-2f-ingenia

clean-RCWS-2f-driver-2f-ingenia:
	-$(RM) ./RCWS/driver/ingenia/ingenia.d ./RCWS/driver/ingenia/ingenia.o ./RCWS/driver/ingenia/ingenia.su ./RCWS/driver/ingenia/ingenia_buffer.d ./RCWS/driver/ingenia/ingenia_buffer.o ./RCWS/driver/ingenia/ingenia_buffer.su ./RCWS/driver/ingenia/ingenia_ll_fdcan.d ./RCWS/driver/ingenia/ingenia_ll_fdcan.o ./RCWS/driver/ingenia/ingenia_ll_fdcan.su

.PHONY: clean-RCWS-2f-driver-2f-ingenia

