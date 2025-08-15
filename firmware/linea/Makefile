# Linea Library Makefile
# Adapted for STM32WB55

# Compiler and flags
CC = arm-none-eabi-gcc
CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
          -Wall -Wextra -std=c99 -O2 -g3 \
          -DSTM32WB55xx -DUSE_HAL_DRIVER

# Include directories
INCLUDES = -I./inc \
           -I../Core/Inc \
           -I../Drivers/STM32WBxx_HAL_Driver/Inc \
           -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include \
           -I../Drivers/CMSIS/Include

# Source files
SRC_FILES = src/linea.c \
            src/linea_bsp_linea_v1.c \
            src/linea_math.c \
            src/BCG.c \
            src/linea_passive_pen.c

# Object files
OBJ_FILES = $(SRC_FILES:.c=.o)

# Library name
LIB_NAME = liblinea.a

# Default target
all: $(LIB_NAME)

# Build library
$(LIB_NAME): $(OBJ_FILES)
	arm-none-eabi-ar rcs $@ $^

# Compile source files
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Clean
clean:
	rm -f $(OBJ_FILES) $(LIB_NAME)

# Install (copy to project)
install: $(LIB_NAME)
	cp $(LIB_NAME) ../
	cp -r inc/* ../Core/Inc/

.PHONY: all clean install
