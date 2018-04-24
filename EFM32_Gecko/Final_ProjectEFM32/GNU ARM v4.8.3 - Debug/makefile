################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include Drivers/subdir.mk
-include CMSIS/efm32lg/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Final_ProjectEFM32.axf

# Tool invocations
Final_ProjectEFM32.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb -T "Final_ProjectEFM32.ld" -Xlinker --gc-sections -Xlinker -Map="Final_ProjectEFM32.map" --specs=nano.specs -o Final_ProjectEFM32.axf "./src/InitDevice.o" "./src/assignment.o" "./src/circbuff.o" "./src/i2c.o" "./src/leuart.o" "./emlib/dmactrl.o" "./emlib/em_acmp.o" "./emlib/em_adc.o" "./emlib/em_aes.o" "./emlib/em_assert.o" "./emlib/em_burtc.o" "./emlib/em_cmu.o" "./emlib/em_cryotimer.o" "./emlib/em_crypto.o" "./emlib/em_dac.o" "./emlib/em_dbg.o" "./emlib/em_dma.o" "./emlib/em_ebi.o" "./emlib/em_emu.o" "./emlib/em_gpcrc.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_idac.o" "./emlib/em_int.o" "./emlib/em_lcd.o" "./emlib/em_ldma.o" "./emlib/em_lesense.o" "./emlib/em_letimer.o" "./emlib/em_leuart.o" "./emlib/em_mpu.o" "./emlib/em_msc.o" "./emlib/em_opamp.o" "./emlib/em_pcnt.o" "./emlib/em_prs.o" "./emlib/em_rmu.o" "./emlib/em_rtc.o" "./emlib/em_rtcc.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./emlib/em_vcmp.o" "./emlib/em_wdog.o" "./Drivers/lesense_letouch.o" "./Drivers/segmentlcd.o" "./CMSIS/efm32lg/startup_gcc_efm32lg.o" "./CMSIS/efm32lg/system_efm32lg.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: Final_ProjectEFM32.hex'
	arm-none-eabi-objcopy -O ihex "Final_ProjectEFM32.axf" "Final_ProjectEFM32.hex"
	@echo ' '

	@echo 'Building bin file: Final_ProjectEFM32.bin'
	arm-none-eabi-objcopy -O binary "Final_ProjectEFM32.axf" "Final_ProjectEFM32.bin"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "Final_ProjectEFM32.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) Final_ProjectEFM32.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
