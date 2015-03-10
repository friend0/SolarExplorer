################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
ADC_SOC_Cnf.obj: C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/C/ADC_SOC_Cnf.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="ADC_SOC_Cnf.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2803x_CodeStartBranch.obj: C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="DSP2803x_CodeStartBranch.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2803x_GlobalVariableDefs.obj: C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/source/DSP2803x_GlobalVariableDefs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="DSP2803x_GlobalVariableDefs.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2803x_usDelay.obj: C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/source/DSP2803x_usDelay.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="DSP2803x_usDelay.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

PWM_1ch_UpDwnCntCompl_Cnf.obj: C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/C/PWM_1ch_UpDwnCntCompl_Cnf.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="PWM_1ch_UpDwnCntCompl_Cnf.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

SolarExplorer-DPL-ISR.obj: ../SolarExplorer-DPL-ISR.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="SolarExplorer-DPL-ISR.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

SolarExplorer-DevInit_F2803x.obj: ../SolarExplorer-DevInit_F2803x.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="SolarExplorer-DevInit_F2803x.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

SolarExplorer-Main.obj: ../SolarExplorer-Main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="SolarExplorer-Main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Util_DLOG4CHC.obj: C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x/Util_DLOG4CHC.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" --include_path="Y:/Documents/GitHub/SolarExplorer/" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="Util_DLOG4CHC.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


