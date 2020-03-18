################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
DigitalOTATremolo_SW.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
DigitalOTATremolo_SW.list \

OBJCOPY_BIN += \
DigitalOTATremolo_SW.bin \


# All Target
all: DigitalOTATremolo_SW.elf secondary-outputs

# Tool invocations
DigitalOTATremolo_SW.elf: $(OBJS) $(USER_OBJS) /home/chris/Projects/Embedded/DigitalOTATremolo_SW/DigitalOTATremolo_SW/STM32F072RBTX_FLASH.ld
	arm-none-eabi-gcc -o "DigitalOTATremolo_SW.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/home/chris/Projects/Embedded/DigitalOTATremolo_SW/DigitalOTATremolo_SW/STM32F072RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="DigitalOTATremolo_SW.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

DigitalOTATremolo_SW.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "DigitalOTATremolo_SW.list"
	@echo 'Finished building: $@'
	@echo ' '

DigitalOTATremolo_SW.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "DigitalOTATremolo_SW.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
