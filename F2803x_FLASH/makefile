################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.3

ORDERED_OBJS += \
"./ADC_SOC_Cnf.obj" \
"./DSP2803x_CodeStartBranch.obj" \
"./DSP2803x_GlobalVariableDefs.obj" \
"./DSP2803x_usDelay.obj" \
"./InverterISR.obj" \
"./PWM_1ch_UpDwnCntCompl_Cnf.obj" \
"./SciCommsGui_32bit.obj" \
"./SolarExplorer-DPL.obj" \
"./SolarExplorer-DevInit_F2803x.obj" \
"./SolarExplorer-Main.obj" \
"./Util_DLOG4CHC.obj" \
"./inverterVariables.obj" \
"./src commros/Commros_user.obj" \
"./fsmFramework/hBridgeFSM.obj" \
"./fsmFramework/inverterFSM.obj" \
"./fsmFramework/mpptFSM.obj" \
"./fsmFramework/panelFSM.obj" \
"C:/ti/controlSUITE/libs/dsp/SGEN/v101/lib/C28x_SGEN_Lib_fixed.lib" \
"C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/cmd/DSP2803x_Headers_nonBIOS.cmd" \
"../F28035_FLASH_SolarExplorer.CMD" \
"C:/ti/controlSUITE/libs/math/IQmath/v160/lib/IQmath.lib" \
"../commros_28xx_c_regular.lib" \
$(GEN_CMDS__FLAG) \
-l"libc.a" \
-l"rts2800_ml.lib" \
-l"IQmath.lib" \
-l"SFRA_IQ_Lib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include src\ commros/subdir_vars.mk
-include fsmFramework/subdir_vars.mk
-include subdir_rules.mk
-include src\ commros/subdir_rules.mk
-include fsmFramework/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
SolarExplorer_PVInverter.out \

EXE_OUTPUTS__QUOTED += \
"SolarExplorer_PVInverter.out" \

BIN_OUTPUTS += \
SolarExplorer_PVInverter.hex \

BIN_OUTPUTS__QUOTED += \
"SolarExplorer_PVInverter.hex" \


# All Target
all: SolarExplorer_PVInverter.out

# Tool invocations
SolarExplorer_PVInverter.out: $(OBJS) $(CMD_SRCS) $(CMD_UPPER_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.3/bin/cl2000" -v28 -ml -mt --cla_support=cla0 -g --define="_DEBUG" --define="FLASH" --define="LARGE_MODEL" --diag_warning=225 -z -m"ProjectName.map" --stack_size=0x380 --warn_sections -i"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.3/lib" -i"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.3/include" -i"C:/ti/controlSUITE/libs/math/IQmath/v160/lib" -i"C:/ti/controlSUITE/libs/app_libs/solar/v1.2/IQ/lib" -i"C:/ti/controlSUITE/libs/app_libs/SFRA/v1_10_00_00/IQ/lib" --reread_libs --xml_link_info="SolarExplorer_PVInverter_linkInfo.xml" --rom_model -o "SolarExplorer_PVInverter.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

SolarExplorer_PVInverter.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C2000 Hex Utility'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.3/bin/hex2000"  -o "SolarExplorer_PVInverter.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "ADC_SOC_Cnf.pp" "DSP2803x_GlobalVariableDefs.pp" "InverterISR.pp" "PWM_1ch_UpDwnCntCompl_Cnf.pp" "SciCommsGui_32bit.pp" "SolarExplorer-DevInit_F2803x.pp" "SolarExplorer-Main.pp" "inverterVariables.pp" "src commros\Commros_user.pp" "fsmFramework\hBridgeFSM.pp" "fsmFramework\inverterFSM.pp" "fsmFramework\mpptFSM.pp" "fsmFramework\panelFSM.pp" 
	-$(RM) "ADC_SOC_Cnf.obj" "DSP2803x_CodeStartBranch.obj" "DSP2803x_GlobalVariableDefs.obj" "DSP2803x_usDelay.obj" "InverterISR.obj" "PWM_1ch_UpDwnCntCompl_Cnf.obj" "SciCommsGui_32bit.obj" "SolarExplorer-DPL.obj" "SolarExplorer-DevInit_F2803x.obj" "SolarExplorer-Main.obj" "Util_DLOG4CHC.obj" "inverterVariables.obj" "src commros\Commros_user.obj" "fsmFramework\hBridgeFSM.obj" "fsmFramework\inverterFSM.obj" "fsmFramework\mpptFSM.obj" "fsmFramework\panelFSM.obj" 
	-$(RM) "DSP2803x_CodeStartBranch.pp" "DSP2803x_usDelay.pp" "SolarExplorer-DPL.pp" "Util_DLOG4CHC.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

