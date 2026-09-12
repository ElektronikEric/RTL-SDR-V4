################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/ADS-B_Decoder.c 

C_DEPS += \
./src/ADS-B_Decoder.d 

OBJS += \
./src/ADS-B_Decoder.o 


# Each subdirectory must supply rules for building sources it contributes
src/ADS-B_Decoder.o: ../src/ADS-B_Decoder.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C Compiler'
	gcc -I"C:\msys64\usr\include" -I"C:\msys64\usr\include\w32api" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/ADS-B_Decoder.d ./src/ADS-B_Decoder.o

.PHONY: clean-src

