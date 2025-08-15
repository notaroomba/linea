/*
* Project Linea
* Copyright (C) 2024 Anhang Li (thelithcore@gmail.com)
* Adapted for STM32WB55 by Nathan
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __LINEA_H
#define __LINEA_H

#include <stdio.h>
#include <stdbool.h>
#include "linea_config.h"
#include <stdint.h>
#include <stdbool.h>

// A temporary workaround for the signs
#define LINEA_DATA_OFFSET 100.0f

/* Debug Macros */
#if (LINEA_DEBUG_LEVEL > 0)
#define LINEA_UsrLog(...)    do{printf("[INFO]: ");printf(__VA_ARGS__);printf("\n");}while (0)
#else
#define LINEA_UsrLog(...)
#endif

#if (LINEA_DEBUG_LEVEL > 1)
#define LINEA_ErrLog(...)    do{printf("[ ERR]: ");printf(__VA_ARGS__);printf("\n");}while (0)
#else
#define LINEA_ErrLog(...)
#endif

#if (LINEA_DEBUG_LEVEL > 2)
#define LINEA_DbgLog(...)    do{printf("[ DBG]: ");printf(__VA_ARGS__);printf("\n");}while (0)
#else
#define LINEA_DbgLog(...)
#endif

// Linea State Machine
typedef enum {
    LINEA_FSM_INIT = 0,     // Initializing
    LINEA_FSM_SEARCH,       // Scan for Pen
    LINEA_FSM_LOCKED,       // The pen is found and locked
} linea_FSM_t;

typedef enum {
    LINEA_DEBUG_NONE = 0,  // No debug
    LINEA_DEBUG_XSCAN,  // X Scan
    LINEA_DEBUG_YSCAN,  // Y Scan
    LINEA_DEBUG_PSCAN,  // Pressure Scan
    LINEA_DEBUG_XYTRACK
} linea_debug_t;

typedef struct {
    int   idx;      // Last Coil
    float val;      // Sample memory
    bool  x_yn;     // X or Y
} linea_position_t;

// Pen Readout Report 
typedef struct {
    uint16_t   xpos;	// Absolute X Position
    uint16_t   ypos;	// Absolute Y Position
    uint16_t   xtilt;	// X Tilt
    uint16_t   ytilt;	// Y Tilt
    uint16_t   rotate;	// Z Rotation
    uint16_t   tip;		// Tip Pressure
    uint16_t   tail;	// Eraser Pressure
    uint16_t   keys;	// All digitizer switched (excluding Keyboard HID keys)
} linea_report_t;

linea_position_t linea_get_position(uint8_t fstep, int lastpos, bool x_yn);
void linea_init();
void linea_cycle();
void linea_set_mode(linea_debug_t mode);

#endif /* __LINEA_H */
