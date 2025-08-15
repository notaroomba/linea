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

#include "BCG.h"
#include <string.h>

// Initialize BCG structure
void bcg_init(bcg_t* bcg) {
    if (!bcg) return;
    
    memset(bcg->buffer, 0, sizeof(bcg->buffer));
    bcg->index = 0;
    bcg->count = 0;
    bcg->mean = 0.0f;
    bcg->variance = 0.0f;
}

// Add a new sample to the BCG buffer
void bcg_add_sample(bcg_t* bcg, uint16_t sample) {
    if (!bcg) return;
    
    // Add sample to buffer
    bcg->buffer[bcg->index] = sample;
    bcg->index = (bcg->index + 1) % BCG_BUFFER_SIZE;
    
    // Update count
    if (bcg->count < BCG_BUFFER_SIZE) {
        bcg->count++;
    }
    
    // Update running statistics
    if (bcg->count == 1) {
        bcg->mean = (float)sample;
        bcg->variance = 0.0f;
    } else {
        float old_mean = bcg->mean;
        bcg->mean = old_mean + ((float)sample - old_mean) / bcg->count;
        bcg->variance = bcg->variance + ((float)sample - old_mean) * ((float)sample - bcg->mean);
    }
}

// Get the current mean value
float bcg_get_mean(bcg_t* bcg) {
    if (!bcg) return 0.0f;
    return bcg->mean;
}

// Get the current variance
float bcg_get_variance(bcg_t* bcg) {
    if (!bcg || bcg->count <= 1) return 0.0f;
    return bcg->variance / (bcg->count - 1);
}

// Reset BCG structure
void bcg_reset(bcg_t* bcg) {
    bcg_init(bcg);
}
