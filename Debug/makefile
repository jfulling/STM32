################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
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
Indy1.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Indy1.list \

OBJCOPY_BIN += \
Indy1.bin \


# All Target
all: Indy1.elf secondary-outputs

# Tool invocations
Indy1.elf: $(OBJS) $(USER_OBJS) C:\Users\Gunther\STM32CubeIDE\workspace_1.3.0\Indy1\STM32L4R9AIIX_FLASH.ld
	arm-none-eabi-gcc -o "Indy1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Gunther\STM32CubeIDE\workspace_1.3.0\Indy1\STM32L4R9AIIX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Indy1.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Indy1.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Indy1.list"
	@echo 'Finished building: $@'
	@echo ' '

Indy1.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Indy1.bin"
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
