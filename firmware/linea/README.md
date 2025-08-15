# Linea Library

A digitizer library adapted from the Patchouli project for STM32WB55 microcontrollers.

## Overview

The Linea library provides functionality for electromagnetic pen digitizer systems, adapted specifically for STM32WB55 microcontrollers. It includes coil selection, frequency generation, ADC sampling, and position tracking capabilities.

## Features

- **Coil Selection**: Supports up to 34 X-coils and 45 Y-coils
- **Frequency Generation**: Configurable frequency range from 100kHz to 1MHz
- **ADC Sampling**: Multi-sample ADC reading with correlated double sampling (CDS)
- **Position Tracking**: Real-time pen position detection and tracking
- **State Machine**: Built-in finite state machine for system control
- **Mathematical Functions**: Comprehensive math library for signal processing
- **Background Processing**: BCG (Background) processing for noise reduction

## Pin Configuration

The library is configured for the following STM32WB55 pin assignments:

### Selection Pins

- **S0**: PC10 (Selection bit 0)
- **S1**: PA15 (Selection bit 1)
- **S2**: PA14 (Selection bit 2)

### Enable Pins

- **E0N**: PB5 (Enable 0, active low)
- **E1N**: PB4 (Enable 1, active low)
- **E2N**: PB3 (Enable 2, active low)
- **E3N**: PD1 (Enable 3, active low)
- **E4N**: PD0 (Enable 4, active low)
- **E5N**: PC12 (Enable 5, active low)

### Control Pins

- **EXCITER**: PB6 (Pen excitation signal)
- **DISCHARGE**: PA0 (ADC discharge control)

### ADC Input

- **ADC1_IN3**: PC2 (Analog input for pen detection)

## Usage

### Basic Initialization

```c
#include "linea.h"

int main(void) {
    // Initialize system
    HAL_Init();
    SystemClock_Config();

    // Initialize Linea library
    linea_init();

    // Main loop
    while (1) {
        linea_cycle();
        HAL_Delay(1);
    }
}
```

### Configuration

The library can be configured by modifying `linea_config.h`:

```c
// Frequency settings
#define LINEA_TIM_F_MIN      (100.0E3f)  // 100kHz minimum
#define LINEA_TIM_F_MAX      (1.0E6f)    // 1MHz maximum
#define LINEA_TIM_F_CENTER   (500.0E3f)  // 500kHz center

// Number of coils
#define LINEA_N_X_COIL    34u
#define LINEA_N_Y_COIL    45u

// Debug level
#define LINEA_DEBUG_LEVEL 1
```

### API Functions

#### Core Functions

- `linea_init()` - Initialize the Linea system
- `linea_cycle()` - Main processing cycle
- `linea_set_mode()` - Set debug mode

#### BSP Functions

- `linea_coil_select()` - Select specific coil
- `linea_TIM_init()` - Initialize timers
- `linea_ADC_init()` - Initialize ADC
- `linea_set_freq()` - Set excitation frequency
- `linea_simple_take_sample()` - Take ADC sample

#### Math Functions

- `linea_mean()`, `linea_variance()`, `linea_std()` - Statistical functions
- `linea_lowpass_filter()`, `linea_highpass_filter()` - Filtering functions
- `linea_clamp()`, `linea_map()` - Utility functions

## Building

### Include Paths

Add the following to your project's include paths:

- `linea/inc/`

### Source Files

Include the following source files in your build:

- `linea/src/linea.c`
- `linea/src/linea_bsp_linea_v1.c`
- `linea/src/linea_math.c`
- `linea/src/BCG.c`
- `linea/src/linea_passive_pen.c`

### Dependencies

The library requires:

- STM32WB55 HAL drivers
- Standard C library (math.h, string.h)
- printf support for debug output

## State Machine

The library implements a three-state finite state machine:

1. **LINEA_FSM_INIT**: System initialization
2. **LINEA_FSM_SEARCH**: Scanning for pen presence
3. **LINEA_FSM_LOCKED**: Tracking pen position

## Coil Mapping

The library supports a 6x8 coil matrix configuration:

- **U1**: Coils 0-7 (X: 0-7, Y: 0-5)
- **U2**: Coils 8-15 (X: 8-15, Y: 6-13)
- **U3**: Coils 16-23 (X: 16-23, Y: 14-21)
- **U4**: Coils 24-31 (X: 24-31, Y: 22-29)
- **U5**: Coils 32-33 (X: 32-33, Y: 30-37)
- **U6**: Coils 34-44 (Y: 38-44)

## License

This project is licensed under the GNU General Public License v3.0.

## Credits

Original Patchouli project by Anhang Li (thelithcore@gmail.com)
Adapted for STM32WB55 by Nathan
