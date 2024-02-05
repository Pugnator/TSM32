##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.2.0-B44] date: [Mon Jan 29 16:13:14 CET 2024] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

JLINK:=jlink.exe
CPUTYPE:=STM32F103C8

######################################
# target
######################################
TARGET = TSM_103

######################################
# versioning
######################################
version := $(subst -, ,$(shell git describe --long --tags))
COMMIT := $(strip $(word 3, $(version)))
COMMITS_PAST := $(strip $(word 2, $(version)))
DIRTY := $(strip $(word 4, $(version)))
ifneq ($(COMMITS_PAST), 0)
	BUILD_INFO_COMMITS := "."$(COMMITS_PAST)
endif

export BUILD_TAG :=$(strip $(word 1, $(version)))
export BUILD_INFO := $(COMMIT)$(BUILD_INFO_COMMITS)$(BUILD_INFO_DIRTY)
BUILD_DATE := $(shell python -c "from datetime import datetime; print(datetime.now().strftime('%d/%m/%Y, %H:%M'))")
BUILD_MACHINE := $(shell echo %username%)@$(shell hostname)


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization

ifeq ($(DEBUG), 1)
OPT = -Og -g
CFLAGS += -DDEBUG
CXXFLAGS += -DDEBUG
else
OPT = -O3
CFLAGS += -DRELEASE
CXXFLAGS += -DRELEASE
endif

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/gpio.c \
Core/Src/adc.c \
Core/Src/tim.c \
Core/Src/dma.c \
Core/Src/crc.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_crc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Core/Src/system_stm32f1xx.c \
Core/Src/spi.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c

#CPP sources
CPP_SOURCES =  \
	Core/Src/bulb_ctrl.cc \
	Core/Src/starter_ctrl.cc \
	Core/Src/turn_ctrl.cc \
	Core/Src/settings.cc \
	Core/Src/switch_ctrl.cc \
	Core/Src/j1850vpw.cc \
	Core/Src/j1850parser.cc \
	Core/Src/tsm.cc \
	Core/Src/sys.cc \
	Core/Src/printf.cc \
	Core/Src/trace.cc\
	Core/Src/uniqueid.cc\
	Core/Src/vmmu.cc\
	Core/Src/ahrs/ahrs.cc\
	Core/Src/ahrs/antijam.cc\
	Core/Src/ahrs/impl/mpu9250/imu.cc\
	Core/Src/ahrs/impl/mpu9250/imu_spi.cc\
	Core/Src/ahrs/impl/mpu9250/imu_i2c.cc\
	Core/Src/ahrs/impl/mpu9250/io_spi.cc\
	Core/Src/ahrs/impl/mpu9250/io_i2c.cc\
	Core/Src/ahrs/impl/mpu9250/dmp.cc\
	Core/Src/ahrs/impl/mpu9250/firmware.cc\
	Core/Src/ahrs/fusion.cc\
	Core/Src/ahrs/mag.cc\
	Core/Src/ahrs/gyro.cc\
	Core/Src/ahrs/selftest.cc\
	Core/Src/ahrs/accel.cc

# ASM sources
ASM_SOURCES =  \
startup_stm32f103xb.s

RTTSRC:=\
	Drivers/SEGGER_RTT/RTT/SEGGER_RTT.c \
	Drivers/SEGGER_RTT/RTT/SEGGER_RTT_printf.c \
	Drivers/SEGGER_RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.c

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CPP = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CPP = $(PREFIX)g++
AS = $(PREFIX)gcc
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3
FPU:=

# float-abi
FLOAT-ABI:=

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F103xB


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/CMSIS/Include \
-IDrivers/SEGGER_RTT/Config \
-IDrivers/SEGGER_RTT/RTT \
-ICore/Src/ahrs/inc/ \
-ICore/Src/ahrs/inc/impl/mpu9250/ \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include

CXXERRORS_TO_SKIP := -Wno-volatile

CXXSTD := c++17 -pedantic -pedantic-errors $(CXXERRORS_TO_SKIP) #-Werror
CSTD := c18 -pedantic
# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS+= $(MCU) --std=$(CSTD) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -flto -fdata-sections -ffunction-sections -fno-strict-aliasing -mtpcs-frame -mtpcs-leaf-frame  -fno-omit-frame-pointer
CXXFLAGS+= $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -flto -fno-exceptions -fno-rtti -fno-strict-aliasing -mtpcs-frame -mtpcs-leaf-frame  -fno-omit-frame-pointer
CXXFLAGS += --std=$(CXXSTD) -Wall -D_GNU_SOURCE


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

CFLAGS += -DVERSION_BUILD_DATE=\""$(BUILD_DATE)\"" \
          -DVERSION_TAG=\""$(BUILD_TAG)\"" \
          -DVERSION_BUILD=\""$(BUILD_INFO)\""

CXXFLAGS += -DVERSION_BUILD_DATE=\""$(BUILD_DATE)\"" \
          -DVERSION_TAG=\""$(BUILD_TAG)\"" \
          -DVERSION_BUILD=\""$(BUILD_INFO)\""

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103C8Tx_FLASH.ld

# libraries
LIBS = -lm -lstdc++
LIBDIR = 
LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -flto -specs=nano.specs -specs=nosys.specs

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cc=.o)))
vpath %.cc $(sort $(dir $(CPP_SOURCES)))
# list of RTT objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(RTTSRC:.c=.o)))
vpath %.c $(sort $(dir $(RTTSRC)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cc Makefile | $(BUILD_DIR) 
	$(CPP) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cc=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		


flash:
	@echo -e "device $(CPUTYPE)\r\nerase\r\nloadbin build/$(TARGET).bin, 0x08000000\r\nr\r\ng\r\nq" > __temp.jlink
	$(JLINK) -if SWD -speed 4000 -CommanderScript __temp.jlink
	-rm -f __temp.jlink

reset:
	@echo -e "device $(CPUTYPE)\r\nr\r\ng\r\nq" > __temp.jlink
	$(JLINK) -if SWD -speed 4000 -CommanderScript __temp.jlink
	$(RM) -f __temp.jlink

erase:
	@echo -e "device $(CPUTYPE)\r\nerase\r\nq" > __temp.jlink
	$(JLINK) -if SWD -speed 4000 -CommanderScript __temp.jlink
	$(RM) -f __temp.jlink
#######################################
# clean up
#######################################
clean:
	-rm -Rf $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
