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
touch_sensor_DMA_UART.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
touch_sensor_DMA_UART.list \

OBJCOPY_BIN += \
touch_sensor_DMA_UART.bin \


# All Target
all: touch_sensor_DMA_UART.elf secondary-outputs

# Tool invocations
touch_sensor_DMA_UART.elf: $(OBJS) $(USER_OBJS) /home/miirho3ein/STM32CubeIDE/workspace_1.3.0/touch_sensor_DMA_UART/STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "touch_sensor_DMA_UART.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/miirho3ein/STM32CubeIDE/workspace_1.3.0/touch_sensor_DMA_UART/STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="touch_sensor_DMA_UART.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

touch_sensor_DMA_UART.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "touch_sensor_DMA_UART.list"
	@echo 'Finished building: $@'
	@echo ' '

touch_sensor_DMA_UART.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "touch_sensor_DMA_UART.bin"
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
