################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/STM32F429I-Discovery/subdir.mk
-include Drivers/BSP/Components/wm8994/subdir.mk
-include Drivers/BSP/Components/ts3510/subdir.mk
-include Drivers/BSP/Components/stmpe811/subdir.mk
-include Drivers/BSP/Components/stmpe1600/subdir.mk
-include Drivers/BSP/Components/st7789h2/subdir.mk
-include Drivers/BSP/Components/st7735/subdir.mk
-include Drivers/BSP/Components/s5k5cag/subdir.mk
-include Drivers/BSP/Components/ov2640/subdir.mk
-include Drivers/BSP/Components/otm8009a/subdir.mk
-include Drivers/BSP/Components/mfxstm32l152/subdir.mk
-include Drivers/BSP/Components/lsm303dlhc/subdir.mk
-include Drivers/BSP/Components/ls016b8uy/subdir.mk
-include Drivers/BSP/Components/lis3dsh/subdir.mk
-include Drivers/BSP/Components/lis302dl/subdir.mk
-include Drivers/BSP/Components/l3gd20/subdir.mk
-include Drivers/BSP/Components/ili9341/subdir.mk
-include Drivers/BSP/Components/ili9325/subdir.mk
-include Drivers/BSP/Components/ft6x06/subdir.mk
-include Drivers/BSP/Components/exc7200/subdir.mk
-include Drivers/BSP/Components/cs43l22/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
JAKUB_KURCZYNSKI_PROJEKT_CS_GO.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
JAKUB_KURCZYNSKI_PROJEKT_CS_GO.list \

OBJCOPY_HEX += \
JAKUB_KURCZYNSKI_PROJEKT_CS_GO.hex \

OBJCOPY_BIN += \
JAKUB_KURCZYNSKI_PROJEKT_CS_GO.bin \


# All Target
all: JAKUB_KURCZYNSKI_PROJEKT_CS_GO.elf secondary-outputs

# Tool invocations
JAKUB_KURCZYNSKI_PROJEKT_CS_GO.elf: $(OBJS) $(USER_OBJS) C:\Users\Jakub\Desktop\PROJEKT\ STRZELNICA\ po\ zajeciach\Na_PC\PROJEKT_z_ZDJECIAMI\STM32F429ZITX_FLASH.ld
	arm-none-eabi-gcc -o "JAKUB_KURCZYNSKI_PROJEKT_CS_GO.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Jakub\Desktop\PROJEKT STRZELNICA po zajeciach\Na_PC\PROJEKT_z_ZDJECIAMI\STM32F429ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="JAKUB_KURCZYNSKI_PROJEKT_CS_GO.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

JAKUB_KURCZYNSKI_PROJEKT_CS_GO.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "JAKUB_KURCZYNSKI_PROJEKT_CS_GO.list"
	@echo 'Finished building: $@'
	@echo ' '

JAKUB_KURCZYNSKI_PROJEKT_CS_GO.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "JAKUB_KURCZYNSKI_PROJEKT_CS_GO.hex"
	@echo 'Finished building: $@'
	@echo ' '

JAKUB_KURCZYNSKI_PROJEKT_CS_GO.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "JAKUB_KURCZYNSKI_PROJEKT_CS_GO.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
