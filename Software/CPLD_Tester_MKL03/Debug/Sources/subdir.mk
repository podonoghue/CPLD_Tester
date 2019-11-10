################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Sources/delay.cpp \
../Sources/ftfa.cpp \
../Sources/hardware.cpp \
../Sources/i2c.cpp \
../Sources/main.cpp \
../Sources/spi.cpp \
../Sources/usbdmError.cpp 

OBJS += \
./Sources/delay.o \
./Sources/ftfa.o \
./Sources/hardware.o \
./Sources/i2c.o \
./Sources/main.o \
./Sources/spi.o \
./Sources/usbdmError.o 

CPP_DEPS += \
./Sources/delay.d \
./Sources/ftfa.d \
./Sources/hardware.d \
./Sources/i2c.d \
./Sources/main.d \
./Sources/spi.d \
./Sources/usbdmError.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m0plus -mthumb -g3 -O0 -ffunction-sections -fdata-sections -fno-rtti -Wall -Wextra -DDEBUG_BUILD -DCPU_MKL03Z8VFG4 -I"C:/Users/podonoghue/Documents/Development/CPLD_Tester/Software/CPLD_Tester_MKL03/Sources" -I"C:/Users/podonoghue/Documents/Development/CPLD_Tester/Software/CPLD_Tester_MKL03/Project_Headers" -fno-exceptions -std=gnu++17 -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)"  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '


