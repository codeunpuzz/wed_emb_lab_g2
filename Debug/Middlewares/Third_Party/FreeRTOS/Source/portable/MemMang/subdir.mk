################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Inc" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Middlewares/Third_Party/FreeRTOS/Source/include" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"D:/H4o5o1h0e4i7n1z/Temp/stm32/workspace/wed_emb_lab_g2/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


