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

#ifndef __BCG_H
#define __BCG_H

#include <stdint.h>

// BCG (Background) related definitions
#define BCG_BUFFER_SIZE 256
#define BCG_SAMPLE_RATE 1000  // Hz

typedef struct {
    uint16_t buffer[BCG_BUFFER_SIZE];
    uint16_t index;
    uint16_t count;
    float mean;
    float variance;
} bcg_t;

void bcg_init(bcg_t* bcg);
void bcg_add_sample(bcg_t* bcg, uint16_t sample);
float bcg_get_mean(bcg_t* bcg);
float bcg_get_variance(bcg_t* bcg);
void bcg_reset(bcg_t* bcg);

#endif /* __BCG_H */
