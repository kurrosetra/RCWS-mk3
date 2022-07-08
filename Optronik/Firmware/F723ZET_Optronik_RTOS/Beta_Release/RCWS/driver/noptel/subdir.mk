################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RCWS/driver/noptel/noptel.c \
../RCWS/driver/noptel/noptel_ll_comm.c 

OBJS += \
./RCWS/driver/noptel/noptel.o \
./RCWS/driver/noptel/noptel_ll_comm.o 

C_DEPS += \
./RCWS/driver/noptel/noptel.d \
./RCWS/driver/noptel/noptel_ll_comm.d 


# Each subdirectory must supply rules for building sources it contributes
RCWS/driver/noptel/%.o RCWS/driver/noptel/%.su: ../RCWS/driver/noptel/%.c RCWS/driver/noptel/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F723xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/Optronik/Firmware/F723ZET_Optronik_RTOS/RCWS" -I"D:/Cloud/NextCloud/stm32_workspace/RCWS-mk3/share_lib/config" -O3 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RCWS-2f-driver-2f-noptel

clean-RCWS-2f-driver-2f-noptel:
	-$(RM) ./RCWS/driver/noptel/noptel.d ./RCWS/driver/noptel/noptel.o ./RCWS/driver/noptel/noptel.su ./RCWS/driver/noptel/noptel_ll_comm.d ./RCWS/driver/noptel/noptel_ll_comm.o ./RCWS/driver/noptel/noptel_ll_comm.su

.PHONY: clean-RCWS-2f-driver-2f-noptel

