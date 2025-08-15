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

#include "linea_passive_pen.h"
#include "linea_config.h"
#include <stdint.h>
#include <stdbool.h>

// Global pen transmission object
linea_tx_t linea_pen_pw100;

// Initialize pen transmission functions
void linea_pen_init() {
    // Initialize the function pointer table
    for (int i = 0; i < LINEA_TIM_N_STEP; i++) {
        linea_pen_pw100.tx_fptr_table[i] = linea_pen_default_tx;
    }
}

// Default pen transmission function
void linea_pen_default_tx() {
    // Default transmission behavior
    // This can be customized for different pen types
}

// Set specific transmission function for a frequency step
void linea_pen_set_tx_function(uint8_t fstep, linea_tx_fptr_t func) {
    if (fstep < LINEA_TIM_N_STEP && func != NULL) {
        linea_pen_pw100.tx_fptr_table[fstep] = func;
    }
}

// Get transmission function for a frequency step
linea_tx_fptr_t linea_pen_get_tx_function(uint8_t fstep) {
    if (fstep < LINEA_TIM_N_STEP) {
        return linea_pen_pw100.tx_fptr_table[fstep];
    }
    return NULL;
}
