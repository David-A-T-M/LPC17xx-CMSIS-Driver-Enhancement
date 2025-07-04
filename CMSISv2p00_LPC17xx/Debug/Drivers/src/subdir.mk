################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/debug_frmwrk.c \
../Drivers/src/lpc17xx_adc.c \
../Drivers/src/lpc17xx_can.c \
../Drivers/src/lpc17xx_clkpwr.c \
../Drivers/src/lpc17xx_dac.c \
../Drivers/src/lpc17xx_emac.c \
../Drivers/src/lpc17xx_exti.c \
../Drivers/src/lpc17xx_gpio.c \
../Drivers/src/lpc17xx_i2c.c \
../Drivers/src/lpc17xx_i2s.c \
../Drivers/src/lpc17xx_libcfg_default.c \
../Drivers/src/lpc17xx_mcpwm.c \
../Drivers/src/lpc17xx_nvic.c \
../Drivers/src/lpc17xx_pinsel.c \
../Drivers/src/lpc17xx_pwm.c \
../Drivers/src/lpc17xx_qei.c \
../Drivers/src/lpc17xx_rit.c \
../Drivers/src/lpc17xx_rtc.c \
../Drivers/src/lpc17xx_spi.c \
../Drivers/src/lpc17xx_ssp.c \
../Drivers/src/lpc17xx_systick.c \
../Drivers/src/lpc17xx_timer.c \
../Drivers/src/lpc17xx_uart.c \
../Drivers/src/lpc17xx_wdt.c 

OBJS += \
./Drivers/src/debug_frmwrk.o \
./Drivers/src/lpc17xx_adc.o \
./Drivers/src/lpc17xx_can.o \
./Drivers/src/lpc17xx_clkpwr.o \
./Drivers/src/lpc17xx_dac.o \
./Drivers/src/lpc17xx_emac.o \
./Drivers/src/lpc17xx_exti.o \
./Drivers/src/lpc17xx_gpio.o \
./Drivers/src/lpc17xx_i2c.o \
./Drivers/src/lpc17xx_i2s.o \
./Drivers/src/lpc17xx_libcfg_default.o \
./Drivers/src/lpc17xx_mcpwm.o \
./Drivers/src/lpc17xx_nvic.o \
./Drivers/src/lpc17xx_pinsel.o \
./Drivers/src/lpc17xx_pwm.o \
./Drivers/src/lpc17xx_qei.o \
./Drivers/src/lpc17xx_rit.o \
./Drivers/src/lpc17xx_rtc.o \
./Drivers/src/lpc17xx_spi.o \
./Drivers/src/lpc17xx_ssp.o \
./Drivers/src/lpc17xx_systick.o \
./Drivers/src/lpc17xx_timer.o \
./Drivers/src/lpc17xx_uart.o \
./Drivers/src/lpc17xx_wdt.o 

C_DEPS += \
./Drivers/src/debug_frmwrk.d \
./Drivers/src/lpc17xx_adc.d \
./Drivers/src/lpc17xx_can.d \
./Drivers/src/lpc17xx_clkpwr.d \
./Drivers/src/lpc17xx_dac.d \
./Drivers/src/lpc17xx_emac.d \
./Drivers/src/lpc17xx_exti.d \
./Drivers/src/lpc17xx_gpio.d \
./Drivers/src/lpc17xx_i2c.d \
./Drivers/src/lpc17xx_i2s.d \
./Drivers/src/lpc17xx_libcfg_default.d \
./Drivers/src/lpc17xx_mcpwm.d \
./Drivers/src/lpc17xx_nvic.d \
./Drivers/src/lpc17xx_pinsel.d \
./Drivers/src/lpc17xx_pwm.d \
./Drivers/src/lpc17xx_qei.d \
./Drivers/src/lpc17xx_rit.d \
./Drivers/src/lpc17xx_rtc.d \
./Drivers/src/lpc17xx_spi.d \
./Drivers/src/lpc17xx_ssp.d \
./Drivers/src/lpc17xx_systick.d \
./Drivers/src/lpc17xx_timer.d \
./Drivers/src/lpc17xx_uart.d \
./Drivers/src/lpc17xx_wdt.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o: ../Drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -I"D:\Users\Gabriel Valenzuela\Documents\MCUXpressoIDE_11.0.0_2516\workspace\CMSISv2p00_LPC17xx\Drivers\inc" -I"../inc" -O0 -Os -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


