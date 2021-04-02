################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../startup/startup_stm32f0xx.S 

OBJS += \
./startup/startup_stm32f0xx.o 

S_UPPER_DEPS += \
./startup/startup_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F0 -DSTM32F051R8Tx -DSTM32F0DISCOVERY -DDEBUG -DSTM32F051 -DUSE_STDPERIPH_DRIVER -I"C:/Users/acton/OneDrive/Windows/Desktop/Junior Spring/ECE 362/lab10/Utilities" -I"C:/Users/acton/OneDrive/Windows/Desktop/Junior Spring/ECE 362/lab10/StdPeriph_Driver/inc" -I"C:/Users/acton/OneDrive/Windows/Desktop/Junior Spring/ECE 362/lab10/inc" -I"C:/Users/acton/OneDrive/Windows/Desktop/Junior Spring/ECE 362/lab10/CMSIS/device" -I"C:/Users/acton/OneDrive/Windows/Desktop/Junior Spring/ECE 362/lab10/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


