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

// ExN List for STM32WB55 Linea board
#define LINEA_N_EXN (6u)
GPIO_TypeDef* ExN_GPIO_Port[LINEA_N_EXN] = {
    E0N_GPIO_Port, E1N_GPIO_Port, E2N_GPIO_Port,
    E3N_GPIO_Port, E4N_GPIO_Port, E5N_GPIO_Port
};
uint16_t ExN_Pin[LINEA_N_EXN] = {
    E0N_Pin, E1N_Pin, E2N_Pin,
    E3N_Pin, E4N_Pin, E5N_Pin
};

// Coil selection function for STM32WB55 Linea board
void linea_coil_select(int id, bool x_yn){
    uint8_t S = 0;
    GPIO_PinState ExN[LINEA_N_EXN];
    
    // Default state is high (disabled)
    for (int i = 0; i < LINEA_N_EXN; i++) {
        ExN[i] = GPIO_PIN_SET;
    }

    if(x_yn) {
        // X Coils
        if(id >= LINEA_N_X_COIL) {
            LINEA_ErrLog("Invalid X Coil ID: %d", id);
            return;
        }
        
        // X coil mapping for STM32WB55 Linea board
        switch(id) {
            case 0:  ExN[0] = GPIO_PIN_RESET; S = 0; break; // U1
            case 1:  ExN[0] = GPIO_PIN_RESET; S = 1; break;
            case 2:  ExN[0] = GPIO_PIN_RESET; S = 2; break;
            case 3:  ExN[0] = GPIO_PIN_RESET; S = 3; break;
            case 4:  ExN[0] = GPIO_PIN_RESET; S = 4; break;
            case 5:  ExN[0] = GPIO_PIN_RESET; S = 5; break;
            case 6:  ExN[0] = GPIO_PIN_RESET; S = 6; break;
            case 7:  ExN[0] = GPIO_PIN_RESET; S = 7; break;
            case 8:  ExN[1] = GPIO_PIN_RESET; S = 0; break; // U2
            case 9:  ExN[1] = GPIO_PIN_RESET; S = 1; break;
            case 10: ExN[1] = GPIO_PIN_RESET; S = 2; break;
            case 11: ExN[1] = GPIO_PIN_RESET; S = 3; break;
            case 12: ExN[1] = GPIO_PIN_RESET; S = 4; break;
            case 13: ExN[1] = GPIO_PIN_RESET; S = 5; break;
            case 14: ExN[1] = GPIO_PIN_RESET; S = 6; break;
            case 15: ExN[1] = GPIO_PIN_RESET; S = 7; break;
            case 16: ExN[2] = GPIO_PIN_RESET; S = 0; break; // U3
            case 17: ExN[2] = GPIO_PIN_RESET; S = 1; break;
            case 18: ExN[2] = GPIO_PIN_RESET; S = 2; break;
            case 19: ExN[2] = GPIO_PIN_RESET; S = 3; break;
            case 20: ExN[2] = GPIO_PIN_RESET; S = 4; break;
            case 21: ExN[2] = GPIO_PIN_RESET; S = 5; break;
            case 22: ExN[2] = GPIO_PIN_RESET; S = 6; break;
            case 23: ExN[2] = GPIO_PIN_RESET; S = 7; break;
            case 24: ExN[3] = GPIO_PIN_RESET; S = 0; break; // U4
            case 25: ExN[3] = GPIO_PIN_RESET; S = 1; break;
            case 26: ExN[3] = GPIO_PIN_RESET; S = 2; break;
            case 27: ExN[3] = GPIO_PIN_RESET; S = 3; break;
            case 28: ExN[3] = GPIO_PIN_RESET; S = 4; break;
            case 29: ExN[3] = GPIO_PIN_RESET; S = 5; break;
            case 30: ExN[3] = GPIO_PIN_RESET; S = 6; break;
            case 31: ExN[3] = GPIO_PIN_RESET; S = 7; break;
            case 32: ExN[4] = GPIO_PIN_RESET; S = 0; break; // U5
            case 33: ExN[4] = GPIO_PIN_RESET; S = 1; break;
            default: 
                LINEA_ErrLog("Invalid X Coil ID: %d", id); 
                return;
        }
    } else {
        // Y Coils
        if(id >= LINEA_N_Y_COIL) {
            LINEA_ErrLog("Invalid Y Coil ID: %d", id);
            return;
        }
        
        // Y coil mapping for STM32WB55 Linea board
        switch (id) {
            case 0:  ExN[0] = GPIO_PIN_RESET; S = 2; break; // U1
            case 1:  ExN[0] = GPIO_PIN_RESET; S = 3; break;
            case 2:  ExN[0] = GPIO_PIN_RESET; S = 4; break;
            case 3:  ExN[0] = GPIO_PIN_RESET; S = 5; break;
            case 4:  ExN[0] = GPIO_PIN_RESET; S = 6; break;
            case 5:  ExN[0] = GPIO_PIN_RESET; S = 7; break;
            case 6:  ExN[1] = GPIO_PIN_RESET; S = 0; break; // U2
            case 7:  ExN[1] = GPIO_PIN_RESET; S = 1; break;
            case 8:  ExN[1] = GPIO_PIN_RESET; S = 2; break;
            case 9:  ExN[1] = GPIO_PIN_RESET; S = 3; break;
            case 10: ExN[1] = GPIO_PIN_RESET; S = 4; break;
            case 11: ExN[1] = GPIO_PIN_RESET; S = 5; break;
            case 12: ExN[1] = GPIO_PIN_RESET; S = 6; break;
            case 13: ExN[1] = GPIO_PIN_RESET; S = 7; break;
            case 14: ExN[2] = GPIO_PIN_RESET; S = 0; break; // U3
            case 15: ExN[2] = GPIO_PIN_RESET; S = 1; break;
            case 16: ExN[2] = GPIO_PIN_RESET; S = 2; break;
            case 17: ExN[2] = GPIO_PIN_RESET; S = 3; break;
            case 18: ExN[2] = GPIO_PIN_RESET; S = 4; break;
            case 19: ExN[2] = GPIO_PIN_RESET; S = 5; break;
            case 20: ExN[2] = GPIO_PIN_RESET; S = 6; break;
            case 21: ExN[2] = GPIO_PIN_RESET; S = 7; break;
            case 22: ExN[3] = GPIO_PIN_RESET; S = 0; break; // U4
            case 23: ExN[3] = GPIO_PIN_RESET; S = 1; break;
            case 24: ExN[3] = GPIO_PIN_RESET; S = 2; break;
            case 25: ExN[3] = GPIO_PIN_RESET; S = 3; break;
            case 26: ExN[3] = GPIO_PIN_RESET; S = 4; break;
            case 27: ExN[3] = GPIO_PIN_RESET; S = 5; break;
            case 28: ExN[3] = GPIO_PIN_RESET; S = 6; break;
            case 29: ExN[3] = GPIO_PIN_RESET; S = 7; break;
            case 30: ExN[4] = GPIO_PIN_RESET; S = 0; break; // U5
            case 31: ExN[4] = GPIO_PIN_RESET; S = 1; break;
            case 32: ExN[4] = GPIO_PIN_RESET; S = 2; break;
            case 33: ExN[4] = GPIO_PIN_RESET; S = 3; break;
            case 34: ExN[4] = GPIO_PIN_RESET; S = 4; break;
            case 35: ExN[4] = GPIO_PIN_RESET; S = 5; break;
            case 36: ExN[4] = GPIO_PIN_RESET; S = 6; break;
            case 37: ExN[4] = GPIO_PIN_RESET; S = 7; break;
            case 38: ExN[5] = GPIO_PIN_RESET; S = 0; break; // U6
            case 39: ExN[5] = GPIO_PIN_RESET; S = 1; break;
            case 40: ExN[5] = GPIO_PIN_RESET; S = 2; break;
            case 41: ExN[5] = GPIO_PIN_RESET; S = 3; break;
            case 42: ExN[5] = GPIO_PIN_RESET; S = 4; break;
            case 43: ExN[5] = GPIO_PIN_RESET; S = 5; break;
            case 44: ExN[5] = GPIO_PIN_RESET; S = 6; break;
            default: 
                LINEA_ErrLog("Invalid Y Coil ID: %d", id);
                return;
        }
    }
    
    // Set selection pins (S0, S1, S2)
    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, (S & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, ((S >> 1) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, ((S >> 2) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // Set enable pins (ExN)
    for (int i = 0; i < LINEA_N_EXN; i++) {
        HAL_GPIO_WritePin(ExN_GPIO_Port[i], ExN_Pin[i], ExN[i]);
    }
}

// Timer initialization for STM32WB55
void linea_TIM_init(){
    // Start TIM1 for pulse generation
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    // Set initial frequency
    float freq = LINEA_TIM_F_CENTER;
    uint16_t period = (uint16_t)(LINEA_TIM_FAST_CLK / freq);
    
    htim1.Instance->ARR = period;
    htim1.Instance->CCR1 = period >> 1; // 50% duty cycle
    
    LINEA_UsrLog("TIM1 initialized with frequency: %f Hz", freq);
}

// Trigger timer
void linea_TIM_trigger(){
    htim1.Instance->CR1 |= TIM_CR1_CEN; // Enable TIM1
}

// ADC initialization
void linea_ADC_init(){
    // ADC is already initialized in main.c
    LINEA_UsrLog("ADC initialized");
}

// Set frequency for TIM1
void linea_set_freq(float freq){
    if(freq < LINEA_TIM_F_MIN || freq > LINEA_TIM_F_MAX) {
        LINEA_ErrLog("Frequency out of range: %f Hz", freq);
        return;
    }
    
    uint16_t period = (uint16_t)(LINEA_TIM_FAST_CLK / freq);
    htim1.Instance->ARR = period;
    htim1.Instance->CCR1 = period >> 1; // 50% duty cycle
    
    LINEA_UsrLog("Frequency set to: %f Hz", freq);
}

// Multi-sample ADC reading
uint16_t _linea_adc_multisample(){
    uint16_t sample[LINEA_ADC_NSAMPLE];
    uint16_t value;
    
    if(LINEA_ADC_NSAMPLE == 1){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        value = HAL_ADC_GetValue(&hadc1);
    } else if (LINEA_ADC_NSAMPLE == 2) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        sample[0] = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        sample[1] = HAL_ADC_GetValue(&hadc1);
        value = (sample[0] + sample[1]) / 2;
    } else {
        for(int i = 0; i < LINEA_ADC_NSAMPLE; i++) {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            sample[i] = HAL_ADC_GetValue(&hadc1);
        }
        // Simple average for now (can be replaced with median filter)
        uint32_t sum = 0;
        for(int i = 0; i < LINEA_ADC_NSAMPLE; i++) {
            sum += sample[i];
        }
        value = (uint16_t)(sum / LINEA_ADC_NSAMPLE);
    }
    
    return value;
}

// Take sample with pen excitation
uint16_t linea_simple_take_sample(bool back_side, uint8_t fstep){
    __disable_irq();
    
    // Enable exciter
    LINEA_EXCITER_HIGH();
    
    // Wait for signal to settle
    if(!back_side) {
        for (int i = 0; i < 400; i++) asm volatile ("nop");
    } else {
        for (int i = 0; i < 100; i++) asm volatile ("nop");
    }
    
    // Discharge
    LINEA_DISCHARGE_DISABLE();
    for (int i = 0; i < 5000; i++) asm volatile ("nop");
    
    // Take sample
    uint16_t value = _linea_adc_multisample();
    
#ifdef LINEA_CDS
    // Correlated Double Sampling
    LINEA_DISCHARGE_ENABLE();
    for (int i = 0; i < 100; i++) asm volatile ("nop");
    LINEA_DISCHARGE_DISABLE();
    for (int i = 0; i < 200; i++) asm volatile ("nop");
    
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t base = HAL_ADC_GetValue(&hadc1);
    
    // Return difference with offset
    value = value - base + (uint16_t)LINEA_DATA_OFFSET;
#endif

    __enable_irq();
    
    // Re-enable discharge
    LINEA_DISCHARGE_ENABLE();
    
    return value;
}

// LED control (placeholder - no LED on STM32WB55 Linea board)
void linea_led_on(){
    // No LED available on this board
}

void linea_led_off(){
    // No LED available on this board
}

// Transmit data via USB HID
bool linea_transmit(linea_report_t* report){
    // This function will be implemented when USB HID is configured
    // For now, just log the data
    LINEA_UsrLog("Transmitting: X=%d, Y=%d, Tip=%d", 
                  report->xpos, report->ypos, report->tip);
    
    return true;
}

#endif /* LINEA_PCB_LINEA_V1 */
