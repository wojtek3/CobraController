################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ahrs.c \
../Core/Src/control.c \
../Core/Src/crsf.c \
../Core/Src/dma.c \
../Core/Src/dps310.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/icm42688.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/memorymap.c \
../Core/Src/mixer.c \
../Core/Src/pid.c \
../Core/Src/pwm.c \
../Core/Src/spi.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_hal_timebase_tim.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

C_DEPS += \
./Core/Src/ahrs.d \
./Core/Src/control.d \
./Core/Src/crsf.d \
./Core/Src/dma.d \
./Core/Src/dps310.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/icm42688.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/memorymap.d \
./Core/Src/mixer.d \
./Core/Src/pid.d \
./Core/Src/pwm.d \
./Core/Src/spi.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_hal_timebase_tim.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 

OBJS += \
./Core/Src/ahrs.o \
./Core/Src/control.o \
./Core/Src/crsf.o \
./Core/Src/dma.o \
./Core/Src/dps310.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/icm42688.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/memorymap.o \
./Core/Src/mixer.o \
./Core/Src/pid.o \
./Core/Src/pwm.o \
./Core/Src/spi.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_hal_timebase_tim.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc -gdwarf-4 "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ahrs.cyclo ./Core/Src/ahrs.d ./Core/Src/ahrs.o ./Core/Src/ahrs.su ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/crsf.cyclo ./Core/Src/crsf.d ./Core/Src/crsf.o ./Core/Src/crsf.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/dps310.cyclo ./Core/Src/dps310.d ./Core/Src/dps310.o ./Core/Src/dps310.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/icm42688.cyclo ./Core/Src/icm42688.d ./Core/Src/icm42688.o ./Core/Src/icm42688.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/memorymap.cyclo ./Core/Src/memorymap.d ./Core/Src/memorymap.o ./Core/Src/memorymap.su ./Core/Src/mixer.cyclo ./Core/Src/mixer.d ./Core/Src/mixer.o ./Core/Src/mixer.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pwm.cyclo ./Core/Src/pwm.d ./Core/Src/pwm.o ./Core/Src/pwm.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_hal_timebase_tim.cyclo ./Core/Src/stm32h7xx_hal_timebase_tim.d ./Core/Src/stm32h7xx_hal_timebase_tim.o ./Core/Src/stm32h7xx_hal_timebase_tim.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

