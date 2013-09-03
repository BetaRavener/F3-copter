PROJ_NAME = F3-copter

###################################################
# Set toolchain
TC = arm-none-eabi
RM = rm -f

# Set Tools
CC			= $(TC)-gcc
CP          = $(TC)-g++
AR			= $(TC)-ar
OBJCOPY		= $(TC)-objcopy
OBJDUMP		= $(TC)-objdump
SIZE		= $(TC)-size

###################################################
# Set Sources
LIB_SRCS	= $(wildcard D:/STM32/Libraries/STM32F30x_StdPeriph_Driver/src/*.c) \
			  $(wildcard D:/STM32/Utilities/STM32F3_Discovery/*.c) 
CFG_SRCS    = $(wildcard Config/src/*.c) 
USER_SRCS	= $(wildcard src/*.cpp)

# Set Objects
LIB_OBJS	= $(LIB_SRCS:.c=.o)
CFG_OBJS    = $(CFG_SRCS:.c=.o) Config/src/startup_stm32f30x.o
USER_OBJS	= $(USER_SRCS:.cpp=.o)

# Set Include Paths
INCLUDES 	= -ID:/STM32/Libraries/CMSIS/Include \
            -ID:/STM32/Libraries/CMSIS/Device/ST/STM32F30x/Include \
			-ID:/STM32/Libraries/STM32F30x_StdPeriph_Driver/inc \
			-ID:/STM32/Utilities/STM32F3_Discovery \
			-Iinc \
            -IConfig/inc 
			
# Set Libraries
LIBS		= -lm -lc -lstdc++

###################################################
# Set Board
MCU 		= -mthumb -mcpu=cortex-m4
FPU 		= -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFINES 	= -DSTM32F3XX -DUSE_STDPERIPH_DRIVER

# Set Compilation and Linking Flags
CFLAGS 		= $(MCU) $(FPU) $(DEFINES) $(INCLUDES) \
			-g -Wall -std=gnu90 -O0 -ffunction-sections -fdata-sections
CPFLAGS     = $(MCU) $(FPU) $(DEFINES) $(INCLUDES) \
            -g -Wall -std=c++11 -O0 -ffunction-sections -fdata-sections
ASFLAGS 	= $(MCU) $(FPU) -g -Wa,--warn -x assembler-with-cpp
LDFLAGS 	= $(MCU) $(FPU) -g -gdwarf-2 \
			-Tstm32f30_flash.ld \
			-Xlinker --gc-sections -Wl,-Map=$(PROJ_NAME).map \
			$(LIBS) \
			-o $(PROJ_NAME).elf

###################################################
# Default Target
all: $(PROJ_NAME).bin info

# elf Target
$(PROJ_NAME).elf: $(LIB_OBJS) $(CFG_OBJS) $(USER_OBJS)
	@$(CC) $(LIB_OBJS) $(CFG_OBJS) $(USER_OBJS) $(LDFLAGS)
	@echo $@

# bin Target
$(PROJ_NAME).bin: $(PROJ_NAME).elf
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	@echo $@

#$(PROJ_NAME).hex: $(PROJ_NAME).elf
#	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
#	@echo $@

#$(PROJ_NAME).lst: $(PROJ_NAME).elf
#	@$(OBJDUMP) -h -S $(PROJ_NAME).elf > $(PROJ_NAME).lst
#	@echo $@

# Display Memory Usage Info
info: $(PROJ_NAME).elf
	@$(SIZE) --format=berkeley $(PROJ_NAME).elf

# Rule for .c files
.c.o:
	@$(CC) $(CFLAGS) -c -o $@ $<
	@echo $@

# Rule for .cpp files
.cpp.o:
	@$(CP) $(CPFLAGS) -c -o $@ $<
	@echo $@

# Rule for .s files
.s.o:
	@$(CC) $(ASFLAGS) -c -o $@ $<
	@echo $@

# Clean Target
clean:
	#$(RM) $(LIB_OBJS)
	$(RM) $(CFG_OBJS)
	$(RM) $(USER_OBJS)
	$(RM) $(PROJ_NAME).elf
	$(RM) $(PROJ_NAME).bin
	$(RM) $(PROJ_NAME).map
