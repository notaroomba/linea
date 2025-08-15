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

#ifndef __LINEA_BSP_H
#define __LINEA_BSP_H

#pragma message("Including linea_bsp_linea_v1.h")

#include "stdbool.h"
#include "stdint.h"
#include "linea.h"
#include "stm32wbxx_hal.h"

#define LINEA_N_X_COIL    34u
#define LINEA_N_Y_COIL    45u

#include "linea_passive_pen.h"

// External HAL handles (declared in main.c)
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

// Subtracting base level when sampling
#define LINEA_CDS

// From the sysclk settings for STM32WB55
// 64MHz Fast Clock for Pulse Generation TIM1
#define LINEA_TIM_FAST_CLK (64.0E6f)
// 64MHz Slow Clock for all the other timers
#define LINEA_TIM_SLOW_CLK (64.0E6f)
// Number of samples
#define LINEA_ADC_NSAMPLE (3u)

// Frequency settings
#define LINEA_TIM_F_MIN     (100.0E3f)  // 100kHz minimum
#define LINEA_TIM_F_MAX     (1.0E6f)    // 1MHz maximum
#define LINEA_TIM_F_CENTER  (500.0E3f)  // 500kHz center
#define LINEA_TIM_F_STEP    (50.0E3f)   // 50kHz step
#define LINEA_TIM_N_STEP    (19u)       // Number of frequency steps

void     linea_coil_select(int id, bool x_yn);
void     linea_TIM_init();
void     linea_TIM_trigger();
void     linea_ADC_init();
void     linea_set_freq(float freq);
uint16_t linea_simple_take_sample(bool back_side, uint8_t fstep);
void     linea_led_on();
void     linea_led_off();
bool     linea_transmit(linea_report_t* report);

// Pin control macros for STM32WB55
#define LINEA_DISCHARGE_ENABLE()  do{LINEA_DISCHARGE_GPIO_Port->BSRR = LINEA_DISCHARGE_Pin;      }while(0)
#define LINEA_DISCHARGE_DISABLE() do{LINEA_DISCHARGE_GPIO_Port->BSRR = LINEA_DISCHARGE_Pin << 16;}while(0)
#define LINEA_EXCITER_HIGH()      do{LINEA_EXCITER_GPIO_Port->BSRR = LINEA_EXCITER_Pin;           }while(0)
#define LINEA_EXCITER_LOW()       do{LINEA_EXCITER_GPIO_Port->BSRR = LINEA_EXCITER_Pin << 16;    }while(0)

#endif /* __LINEA_BSP_H */
