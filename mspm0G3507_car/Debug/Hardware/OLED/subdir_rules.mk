################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Hardware/OLED/%.o: ../Hardware/OLED/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccstheia140/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car" -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car/Hardware" -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car/Hardware/MENU" -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car/Hardware/delay" -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car/Hardware/Encoder" -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car/Hardware/KEY" -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car/Hardware/MPU6050" -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car/Hardware/OLED" -I"C:/Users/33534/workspace_ccstheia/mspm0G3507_car/Debug" -I"C:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"Hardware/OLED/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


