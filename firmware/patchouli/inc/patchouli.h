/*
* Project Patchouli
* Copyright (C) 2024 Anhang Li (thelithcore@gmail.com)
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

#ifndef __PATCHOULI_H
#define __PATCHOULI_H

#include <stdio.h>
#include <stdbool.h>
#include "patchouli_config.h"


// A temporary workaround for the signs
#define PATCHOULI_DATA_OFFSET 100.0f

// Helper function for debug output
void patchouli_debug_output(const char* prefix, const char* format, ...);

/* Debug Macros */
#if (PATCHOULI_DEBUG_LEVEL > 0)
#define PATCHOULI_UsrLog(...)    patchouli_debug_output("[INFO]: ", __VA_ARGS__)
#else
#define PATCHOULI_UsrLog(...)
#endif

#if (PATCHOULI_DEBUG_LEVEL > 1)
#define PATCHOULI_ErrLog(...)    patchouli_debug_output("[ ERR]: ", __VA_ARGS__)
#else
#define PATCHOULI_ErrLog(...)
#endif

#if (PATCHOULI_DEBUG_LEVEL > 2)
#define PATCHOULI_DbgLog(...)    patchouli_debug_output("[ DBG]: ", __VA_ARGS__)
#else
#define PATCHOULI_DbgLog(...)
#endif

// Patchouli State Machine
typedef enum {
    PATCHOULI_FSM_INIT = 0,     // Initializing
    PATCHOULI_FSM_SEARCH,       // Scan for Pen
    PATCHOULI_FSM_LOCKED,       // The pen is found and locked
} patchouli_FSM_t;

typedef enum {
    PATCHOULI_DEBUG_NONE = 0,  // No debug
    PATCHOULI_DEBUG_XSCAN,  // X Scan
    PATCHOULI_DEBUG_YSCAN,  // Y Scan
    PATCHOULI_DEBUG_PSCAN,  // Pressure Scan
    PATCHOULI_DEBUG_XYTRACK, // X-Y Tracking
    PATCHOULI_DEBUG_PEN_SCAN // Passive pen parameter scan
} patchouli_debug_t;

typedef struct {
    int   idx;      // Last Coil
    float val;      // Sample memory
    bool  x_yn;     // X or Y
} patchouli_position_t;

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
} patchouli_report_t;

patchouli_position_t patchouli_get_position(uint8_t fstep, int lastpos, bool x_yn);
void patchouli_init();
void patchouli_cycle();
void patchouli_set_mode(patchouli_debug_t mode);

#endif /* __PATCHOULI_H */
