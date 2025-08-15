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

#include "linea_config.h"
#ifdef LINEA_PCB_LINEA_V1

#include "linea_bsp_linea_v1.h"
#include "main.h"
#include "math.h"
#include "linea_math.h"
#include "linea.h"
#include "BCG.h"
#include <stdint.h>
#include <stdbool.h>

// Function prototypes
static void linea_scan_for_pen(void);
static void linea_track_pen(void);

// Linea State Machine
static linea_FSM_t linea_fsm = LINEA_FSM_INIT;
static linea_debug_t linea_debug_mode = LINEA_DEBUG_NONE;

// Position tracking
static linea_position_t linea_pos_x = {0, 0.0f, true};
static linea_position_t linea_pos_y = {0, 0.0f, false};

// Frequency step tracking
static uint8_t linea_fstep = 0;
static float linea_freq = LINEA_TIM_F_CENTER;

// Initialize Linea system
void linea_init() {
    LINEA_UsrLog("Initializing Linea system...");
    
    // Initialize BSP
    linea_TIM_init();
    linea_ADC_init();
    
    // Initialize position tracking
    linea_pos_x.idx = 0;
    linea_pos_x.val = 0.0f;
    linea_pos_x.x_yn = true;
    
    linea_pos_y.idx = 0;
    linea_pos_y.val = 0.0f;
    linea_pos_y.x_yn = false;
    
    // Set initial frequency
    linea_set_freq(LINEA_TIM_F_CENTER);
    
    // Set initial state
    linea_fsm = LINEA_FSM_SEARCH;
    
    LINEA_UsrLog("Linea system initialized successfully");
}

// Main Linea cycle function
void linea_cycle() {
    switch(linea_fsm) {
        case LINEA_FSM_INIT:
            linea_init();
            break;
            
        case LINEA_FSM_SEARCH:
            // Scan for pen
            linea_scan_for_pen();
            break;
            
        case LINEA_FSM_LOCKED:
            // Track pen position
            linea_track_pen();
            break;
            
        default:
            LINEA_ErrLog("Invalid FSM state: %d", linea_fsm);
            linea_fsm = LINEA_FSM_INIT;
            break;
    }
}

// Set debug mode
void linea_set_mode(linea_debug_t mode) {
    linea_debug_mode = mode;
    LINEA_UsrLog("Debug mode set to: %d", mode);
}

// Get position for a specific frequency step
linea_position_t linea_get_position(uint8_t fstep, int lastpos, bool x_yn) {
    linea_position_t pos;
    
    if(x_yn) {
        pos = linea_pos_x;
    } else {
        pos = linea_pos_y;
    }
    
    // Update frequency step
    linea_fstep = fstep;
    linea_freq = LINEA_TIM_F_MIN + (fstep * LINEA_TIM_F_STEP);
    
    // Set frequency
    linea_set_freq(linea_freq);
    
    // Take sample
    uint16_t sample = linea_simple_take_sample(false, fstep);
    
    // Update position
    pos.val = (float)sample;
    
    return pos;
}

// Scan for pen
static void linea_scan_for_pen() {
    static uint8_t scan_step = 0;
    static bool scanning_x = true;
    
    // Select coil
    if(scanning_x) {
        linea_coil_select(scan_step, true);
        linea_pos_x.idx = scan_step;
    } else {
        linea_coil_select(scan_step, false);
        linea_pos_y.idx = scan_step;
    }
    
    // Take sample
    uint16_t sample = linea_simple_take_sample(false, linea_fstep);
    
    // Check if pen is detected (threshold-based detection)
    if(sample > 100) { // Adjust threshold as needed
        LINEA_UsrLog("Pen detected! X:%d, Y:%d, Sample:%d", 
                     scanning_x ? scan_step : 0, 
                     scanning_x ? 0 : scan_step, 
                     sample);
        linea_fsm = LINEA_FSM_LOCKED;
        return;
    }
    
    // Move to next step
    scan_step++;
    
    if(scanning_x && scan_step >= LINEA_N_X_COIL) {
        scanning_x = false;
        scan_step = 0;
    } else if(!scanning_x && scan_step >= LINEA_N_Y_COIL) {
        scanning_x = true;
        scan_step = 0;
    }
}

// Track pen position
static void linea_track_pen() {
    // Get current position
    linea_position_t x_pos = linea_get_position(linea_fstep, linea_pos_x.idx, true);
    linea_position_t y_pos = linea_get_position(linea_fstep, linea_pos_y.idx, false);
    
    // Update stored positions
    linea_pos_x = x_pos;
    linea_pos_y = y_pos;
    
    // Create report
    linea_report_t report;
    report.xpos = (uint16_t)(x_pos.val * 10); // Scale as needed
    report.ypos = (uint16_t)(y_pos.val * 10); // Scale as needed
    report.tip = 1000; // Default pressure
    report.xtilt = 0;
    report.ytilt = 0;
    report.rotate = 0;
    report.tail = 0;
    report.keys = 0;
    
    // Transmit report
    linea_transmit(&report);
    
    // Check if pen is still detected
    if(x_pos.val < 50 && y_pos.val < 50) { // Adjust threshold as needed
        LINEA_UsrLog("Pen lost, returning to search mode");
        linea_fsm = LINEA_FSM_SEARCH;
    }
}

#endif /* LINEA_PCB_LINEA_V1 */
