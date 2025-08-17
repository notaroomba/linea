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

#ifndef __PATCHOULI_BSP_H
#define __PATCHOULI_BSP_H

#pragma message("Including patchouli_bsp_glider_addon_v1.h")

#include <stdbool.h>
#include <stdint.h>
#include "patchouli.h"

#define PATCHOULI_N_X_COIL    28u
#define PATCHOULI_N_Y_COIL    18u

#include "patchouli_passive_pen.h"

// Subtracting base level when sampling
#define PATCHOULI_CDS

// From the sysclk settings
// 96MHz Fast Clock for Pulse Generation TIM1
#define PATCHOULI_TIM_FAST_CLK (64.0E6f)
// 48MHz Slow Clock for all the other timers
#define PATCHOULI_TIM_SLOW_CLK (64.0E6f)
// Number of samples
#define PATCHOULI_ADC_NSAMPLE (3u)
// Number of pulses in the pulse train
// #define PATCHOULI_TIM_N_PULSE  (60u)

void     patchouli_coil_select(int id, bool x_yn);
void     patchouli_TIM_init();
void     patchouli_TIM_trigger();
void     patchouli_ADC_init();
void     patchouli_set_freq(float freq);
uint16_t patchouli_simple_take_sample(bool back_side, uint8_t fstep);
void     patchouli_led_on();
void     patchouli_led_off();
bool     patchouli_transmit(patchouli_report_t* report);

#define PATCHOULI_DISCHARGE_ENABLE()  do{DISCHARGE_GPIO_Port->BSRR = DISCHARGE_Pin;      }while(0)
#define PATCHOULI_DISCHARGE_DISABLE() do{DISCHARGE_GPIO_Port->BSRR = DISCHARGE_Pin << 16;}while(0)
#define PATCHOULI_TX_HIGH()           do{GPIOB->BSRR = GPIO_PIN_6;                      }while(0)
#define PATCHOULI_TX_LOW()            do{GPIOB->BSRR = GPIO_PIN_6 << 16;                }while(0)
#define PATCHOULI_TX_TRISTATE()       do{GPIOB->MODER &= ~GPIO_MODER_MODE6_Msk;         }while(0)
#define PATCHOULI_TX_PP()             do{GPIOB->MODER |=  GPIO_MODER_MODE6_0;           }while(0)

#endif /* __PATCHOULI_BSP_H */
