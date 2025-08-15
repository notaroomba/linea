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

#ifndef __LINEA_PASSIVE_PEN_H
#define __LINEA_PASSIVE_PEN_H

#include "linea.h"

typedef void (*linea_tx_fptr_t)(void);

typedef struct {
    linea_tx_fptr_t tx_fptr_table[LINEA_TIM_N_STEP];
} linea_tx_t;

extern linea_tx_t linea_pen_pw100;

// Function declarations
void linea_pen_default_tx(void);
void linea_pen_init(void);
void linea_pen_set_tx_function(uint8_t fstep, linea_tx_fptr_t func);
linea_tx_fptr_t linea_pen_get_tx_function(uint8_t fstep);

#endif /* __LINEA_PASSIVE_PEN_H */
