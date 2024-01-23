################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32l07xx_gpio_driver.c 

OBJS += \
./drivers/Src/stm32l07xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/stm32l07xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_L073RZ -DSTM32 -DSTM32L073RZTx -DSTM32L0 -c -I../Inc -I"C:/Users/andrew.purves/dev/mcu_course/drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32l07xx_gpio_driver.cyclo ./drivers/Src/stm32l07xx_gpio_driver.d ./drivers/Src/stm32l07xx_gpio_driver.o ./drivers/Src/stm32l07xx_gpio_driver.su

.PHONY: clean-drivers-2f-Src

