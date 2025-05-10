################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccstheia140/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/MPU6050" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/delay" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/Debug" -I"C:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1284711103: ../main.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccstheia140/ccs/utils/sysconfig_1.20.0/sysconfig_cli.bat" --script "C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/main.syscfg" -o "." -s "C:/ti/mspm0_sdk_2_01_00_03/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1284711103 ../main.syscfg
device.opt: build-1284711103
device.cmd.genlibs: build-1284711103
ti_msp_dl_config.c: build-1284711103
ti_msp_dl_config.h: build-1284711103
Event.dot: build-1284711103

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccstheia140/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/MPU6050" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/delay" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/Debug" -I"C:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0l130x_ticlang.o: C:/ti/mspm0_sdk_2_01_00_03/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0l130x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccstheia140/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/MPU6050" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/delay" -I"C:/Users/33534/workspace_ccstheia/empty_LP_MSPM0L1306_nortos_ticlang/Debug" -I"C:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


