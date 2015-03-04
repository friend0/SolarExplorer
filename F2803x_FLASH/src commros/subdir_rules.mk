################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
src\ commros/Commros_user.obj: ../src\ commros/Commros_user.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.7/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="Y:/Documents/GitHub/SolarExplorer/src commros" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/include" --include_path="C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/asm" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.0/IQ" --include_path="C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x" --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="Y:/Documents/GitHub/SolarExplorer/fsmFramework" -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 --preproc_with_compile --preproc_dependency="src commros/Commros_user.pp" --obj_directory="src commros" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


