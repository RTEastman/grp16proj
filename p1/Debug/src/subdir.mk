################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/fifo.c \
../src/main.c \
../src/screen.c \
../src/snake.c \
../src/syscalls.c \
../src/system_stm32f0xx.c \
../src/tty.c \
../src/uart.c 

OBJS += \
./src/fifo.o \
./src/main.o \
./src/screen.o \
./src/snake.o \
./src/syscalls.o \
./src/system_stm32f0xx.o \
./src/tty.o \
./src/uart.o 

C_DEPS += \
./src/fifo.d \
./src/main.d \
./src/screen.d \
./src/snake.d \
./src/syscalls.d \
./src/system_stm32f0xx.d \
./src/tty.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F0 -DSTM32F051R8Tx -DSTM32F0DISCOVERY -DDEBUG -DSTM32F051 -DUSE_STDPERIPH_DRIVER -I"C:/Users/acton/OneDrive/Windows/Desktop/Spring2021/ECE 477/p1/Utilities" -I"C:/Users/acton/OneDrive/Windows/Desktop/Spring2021/ECE 477/p1/StdPeriph_Driver/inc" -I"C:/Users/acton/OneDrive/Windows/Desktop/Spring2021/ECE 477/p1/inc" -I"C:/Users/acton/OneDrive/Windows/Desktop/Spring2021/ECE 477/p1/CMSIS/device" -I"C:/Users/acton/OneDrive/Windows/Desktop/Spring2021/ECE 477/p1/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


