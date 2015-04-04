################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/cmd/DSP2803x_Headers_nonBIOS.cmd 

CMD_UPPER_SRCS += \
../F28035_FLASH_SolarExplorer.CMD 

LIB_SRCS += \
C:/ti/controlSUITE/libs/dsp/SGEN/v101/lib/C28x_SGEN_Lib_fixed.lib \
C:/ti/controlSUITE/libs/math/IQmath/v160/lib/IQmath.lib \
../commros_28xx_c_regular.lib 

ASM_SRCS += \
C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm \
C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/source/DSP2803x_usDelay.asm \
../SolarExplorer-DPL-ISR.asm \
C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x/Util_DLOG4CHC.asm 

C_SRCS += \
C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/C/ADC_SOC_Cnf.c \
C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/source/DSP2803x_GlobalVariableDefs.c \
../InverterISR.c \
C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/C/PWM_1ch_UpDwnCntCompl_Cnf.c \
../SolarExplorer-DevInit_F2803x.c \
../SolarExplorer-Main.c \
../inverterVariables.c 

OBJS += \
./ADC_SOC_Cnf.obj \
./DSP2803x_CodeStartBranch.obj \
./DSP2803x_GlobalVariableDefs.obj \
./DSP2803x_usDelay.obj \
./InverterISR.obj \
./PWM_1ch_UpDwnCntCompl_Cnf.obj \
./SolarExplorer-DPL-ISR.obj \
./SolarExplorer-DevInit_F2803x.obj \
./SolarExplorer-Main.obj \
./Util_DLOG4CHC.obj \
./inverterVariables.obj 

ASM_DEPS += \
./DSP2803x_CodeStartBranch.pp \
./DSP2803x_usDelay.pp \
./SolarExplorer-DPL-ISR.pp \
./Util_DLOG4CHC.pp 

C_DEPS += \
./ADC_SOC_Cnf.pp \
./DSP2803x_GlobalVariableDefs.pp \
./InverterISR.pp \
./PWM_1ch_UpDwnCntCompl_Cnf.pp \
./SolarExplorer-DevInit_F2803x.pp \
./SolarExplorer-Main.pp \
./inverterVariables.pp 

C_DEPS__QUOTED += \
"ADC_SOC_Cnf.pp" \
"DSP2803x_GlobalVariableDefs.pp" \
"InverterISR.pp" \
"PWM_1ch_UpDwnCntCompl_Cnf.pp" \
"SolarExplorer-DevInit_F2803x.pp" \
"SolarExplorer-Main.pp" \
"inverterVariables.pp" 

OBJS__QUOTED += \
"ADC_SOC_Cnf.obj" \
"DSP2803x_CodeStartBranch.obj" \
"DSP2803x_GlobalVariableDefs.obj" \
"DSP2803x_usDelay.obj" \
"InverterISR.obj" \
"PWM_1ch_UpDwnCntCompl_Cnf.obj" \
"SolarExplorer-DPL-ISR.obj" \
"SolarExplorer-DevInit_F2803x.obj" \
"SolarExplorer-Main.obj" \
"Util_DLOG4CHC.obj" \
"inverterVariables.obj" 

ASM_DEPS__QUOTED += \
"DSP2803x_CodeStartBranch.pp" \
"DSP2803x_usDelay.pp" \
"SolarExplorer-DPL-ISR.pp" \
"Util_DLOG4CHC.pp" 

C_SRCS__QUOTED += \
"C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/C/ADC_SOC_Cnf.c" \
"C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/source/DSP2803x_GlobalVariableDefs.c" \
"../InverterISR.c" \
"C:/ti/controlSUITE/libs/app_libs/digital_power/f2803x_v3.3/C/PWM_1ch_UpDwnCntCompl_Cnf.c" \
"../SolarExplorer-DevInit_F2803x.c" \
"../SolarExplorer-Main.c" \
"../inverterVariables.c" 

ASM_SRCS__QUOTED += \
"C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm" \
"C:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/source/DSP2803x_usDelay.asm" \
"../SolarExplorer-DPL-ISR.asm" \
"C:/ti/controlSUITE/libs/app_libs/drivers/v1.0/F2803x/Util_DLOG4CHC.asm" 


