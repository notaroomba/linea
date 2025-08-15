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

#ifndef __LINEA_CONFIG_H
#define __LINEA_CONFIG_H

// Define the PCB type for STM32WB55 Linea
#define LINEA_PCB_LINEA_V1

// Debug level (0=none, 1=info, 2=error, 3=debug)
#define LINEA_DEBUG_LEVEL 1

// Frequency settings for STM32WB55
#define LINEA_TIM_F_MIN      (100.0E3f)  // 100kHz minimum
#define LINEA_TIM_F_MAX      (1.0E6f)    // 1MHz maximum
#define LINEA_TIM_F_CENTER   (500.0E3f)  // 500kHz center
#define LINEA_TIM_F_STEP     (50.0E3f)   // 50kHz step
#define LINEA_TIM_N_STEP     (19u)       // Number of frequency steps

// Number of coils
#define LINEA_N_X_COIL    34u
#define LINEA_N_Y_COIL    45u

// Clock frequencies for STM32WB55
#define LINEA_TIM_FAST_CLK (64.0E6f)  // 64MHz for TIM1
#define LINEA_TIM_SLOW_CLK (64.0E6f)  // 64MHz for other timers

// ADC settings
#define LINEA_ADC_NSAMPLE (3u)        // Number of samples for averaging

// Pin definitions for STM32WB55
#define LINEA_S0_Pin       S0_Pin
#define LINEA_S0_GPIO_Port S0_GPIO_Port
#define LINEA_S1_Pin       S1_Pin
#define LINEA_S1_GPIO_Port S1_GPIO_Port
#define LINEA_S2_Pin       S2_Pin
#define LINEA_S2_GPIO_Port S2_GPIO_Port

#define LINEA_E0N_Pin      E0N_Pin
#define LINEA_E0N_GPIO_Port E0N_GPIO_Port
#define LINEA_E1N_Pin      E1N_Pin
#define LINEA_E1N_GPIO_Port E1N_GPIO_Port
#define LINEA_E2N_Pin      E2N_Pin
#define LINEA_E2N_GPIO_Port E2N_GPIO_Port
#define LINEA_E3N_Pin      E3N_Pin
#define LINEA_E3N_GPIO_Port E3N_GPIO_Port
#define LINEA_E4N_Pin      E4N_Pin
#define LINEA_E4N_GPIO_Port E4N_GPIO_Port
#define LINEA_E5N_Pin      E5N_Pin
#define LINEA_E5N_GPIO_Port E5N_GPIO_Port

#define LINEA_EXCITER_Pin      EXCITER_Pin
#define LINEA_EXCITER_GPIO_Port EXCITER_GPIO_Port
#define LINEA_DISCHARGE_Pin    DISCHARGE_Pin
#define LINEA_DISCHARGE_GPIO_Port DISCHARGE_GPIO_Port

// Enable CDS (Correlated Double Sampling)
#define LINEA_CDS

#endif /* __LINEA_CONFIG_H */
