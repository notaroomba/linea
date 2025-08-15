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

#include "patchouli_config.h"
#ifdef PATCHOULI_PCB_DISCRETE_SST

#include "patchouli_bsp_discrete_sst.h"
#include "main.h"
#include "math.h"
#include "patchouli_math.h"
#include "patchouli.h"
#include "BCG.h"
#include "patchouli_bsp_tx.h"

void patchouli_coil_select(int id, bool x_yn){
    uint8_t S   = 0;
    GPIO_PinState    E0N = GPIO_PIN_SET;
    GPIO_PinState    E1N = GPIO_PIN_SET;
    GPIO_PinState    E2N = GPIO_PIN_SET;
    if(x_yn) {
        // X Coils
        if(id >= PATCHOULI_N_X_COIL) {
            PATCHOULI_ErrLog("Invalid X Coil ID: %d", id);
            return;
        }
        switch(id) {
            case 0:  E2N = GPIO_PIN_RESET; S = 4; break;
            case 1:  E2N = GPIO_PIN_RESET; S = 6; break;
            case 2:  E2N = GPIO_PIN_RESET; S = 7; break;
            case 3:  E2N = GPIO_PIN_RESET; S = 5; break;
            case 4:  E1N = GPIO_PIN_RESET; S = 3; break;
            case 5:  E1N = GPIO_PIN_RESET; S = 0; break;
            case 6:  E1N = GPIO_PIN_RESET; S = 1; break;
            case 7:  E1N = GPIO_PIN_RESET; S = 4; break;
            case 8:  E1N = GPIO_PIN_RESET; S = 7; break;
            case 9:  E0N = GPIO_PIN_RESET; S = 0; break;
            case 10: E0N = GPIO_PIN_RESET; S = 4; break;
            case 11: E0N = GPIO_PIN_RESET; S = 5; break;
            default: 
                PATCHOULI_ErrLog("Invalid X Coil ID: %d", id); 
                return;
        }
    } else {
        // Y Coils
        if(id >= PATCHOULI_N_Y_COIL) {
            PATCHOULI_ErrLog("Invalid Y Coil ID: %d", id);
            return;
        }
        switch (id) {
            case 0:  E0N = GPIO_PIN_RESET; S = 7; break;
            case 1:  E0N = GPIO_PIN_RESET; S = 6; break;
            case 2:  E0N = GPIO_PIN_RESET; S = 1; break;
            case 3:  E0N = GPIO_PIN_RESET; S = 2; break;
            case 4:  E1N = GPIO_PIN_RESET; S = 6; break;
            case 5:  E2N = GPIO_PIN_RESET; S = 3; break;
            case 6:  E2N = GPIO_PIN_RESET; S = 0; break;
            case 7:  E2N = GPIO_PIN_RESET; S = 1; break;
            case 8:  E2N = GPIO_PIN_RESET; S = 2; break;
            default: 
                PATCHOULI_ErrLog("Invalid Y Coil ID: %d", id);
                return;
        }
    }
    HAL_GPIO_WritePin(S0_GPIO_Port,  S0_Pin,  (S & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S1_GPIO_Port,  S1_Pin,  ((S >> 1) & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S2_GPIO_Port,  S2_Pin,  ((S >> 2) & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E0N_GPIO_Port, E0N_Pin, E0N);
    HAL_GPIO_WritePin(E1N_GPIO_Port, E1N_Pin, E1N);
    HAL_GPIO_WritePin(E2N_GPIO_Port, E2N_Pin, E2N);
    return;
}

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern float gfreq;

void patchouli_TIM_init(){
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	gfreq = PATCHOULI_TIM_F_CENTER;
	uint16_t d = (PATCHOULI_TIM_FAST_CLK/gfreq);
	htim1.Instance->ARR = d;
	htim1.Instance->CCR1 = d>>1;
	htim1.Instance->RCR = 60;
}

void patchouli_TIM_trigger(){
    htim1.Instance->CR1 |= B16(00000000,00000001); // Enable TIM1
}

extern ADC_HandleTypeDef hadc1;

uint16_t _patchouli_adc_multisample(){
	uint16_t sample[PATCHOULI_ADC_NSAMPLE];
	uint16_t value;
	if(PATCHOULI_ADC_NSAMPLE==1){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		value = HAL_ADC_GetValue(&hadc1);
	} else if (PATCHOULI_ADC_NSAMPLE==2) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		sample[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		sample[1] = HAL_ADC_GetValue(&hadc1);
		value = (sample[0]+sample[1])/2;
	} else {
		for(int i=0;i<PATCHOULI_ADC_NSAMPLE; i++) {
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			sample[i] = HAL_ADC_GetValue(&hadc1);
		}
		value = patchouli_median(sample, PATCHOULI_ADC_NSAMPLE);
	}
	return value;
}

extern patchouli_tx_t patchouli_pen_pw100;
uint16_t patchouli_simple_take_sample(bool back_side, uint8_t fstep){
	__disable_irq();
	patchouli_pen_pw100.tx_fptr_table[fstep]();
	  if(!back_side)
		  for (int i=0;i<400;i++)  asm volatile ("nop");
	  else
		  for (int i=0;i<100;i++)  asm volatile ("nop");
	  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET);
	  for (int i=0;i<5000;i++) asm volatile ("nop");
	  uint16_t value = _patchouli_adc_multisample();
# ifdef PATCHOULI_CDS
	  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_SET);
	  for (int i=0;i<100;i++) asm volatile ("nop");
	  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET);
	  for (int i=0;i<200;i++) asm volatile ("nop");
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  uint16_t base = HAL_ADC_GetValue(&hadc1);
#endif
	  __enable_irq();
	  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_SET);
#ifdef PATCHOULI_CDS
	  // return value-base;
	  return value-base+PATCHOULI_DATA_OFFSET;
#else
	  return value;
#endif
}

// Configure TIM1 to generate the expected output frequency
//
void patchouli_set_freq(float freq){
	  uint16_t t = round(PATCHOULI_TIM_FAST_CLK/freq);
	  htim1.Instance->ARR  = t;
	  htim1.Instance->CCR1 = (t/2);
}
void patchouli_freq_test(){
	  printf("Target\tAchieved\r\n");
	  float flist[PATCHOULI_TIM_N_STEP];
	  float fachieved[PATCHOULI_TIM_N_STEP];
	  for (int i=0;i<PATCHOULI_TIM_N_STEP;i++){
		  flist[i] = PATCHOULI_TIM_F_MIN + PATCHOULI_TIM_F_STEP * i;
		  uint16_t d = (PATCHOULI_TIM_FAST_CLK/flist[i]);
		  fachieved[i] = PATCHOULI_TIM_FAST_CLK/d;
		  printf("%f\t%f\r\n", flist[i]/1e3, fachieved[i]/1e3);
	  }
}
void patchouli_led_on(){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void patchouli_led_off(){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

extern UART_HandleTypeDef huart1;
extern uint8_t huart1_rxbuffer[256];
// Init all communication drivers (UART, USB, etc)
void patchouli_comms_init(){
	HAL_UART_Receive_IT(&huart1, (uint8_t *)huart1_rxbuffer, 1);

}
#include "usbd_hid.h"
extern uint8_t usbhid_txbuf[12];
extern USBD_HandleTypeDef hUsbDeviceFS;
bool patchouli_transmit(patchouli_report_t* report){
	int16_t p = report->tip;
	int16_t x = report->xpos;
	int16_t y = report->ypos;
	// printf("%d\t%d\t%d\r\n", p,x,y);
	usbhid_txbuf[0]  = 0x08;
	usbhid_txbuf[1]  = 0x80;
	usbhid_txbuf[2]  = x&0xFF;
	usbhid_txbuf[3]  = (x>>8)&0xFF;
	usbhid_txbuf[4]  = y&0xFF;
	usbhid_txbuf[5]  = (y>>8)&0xFF;
	usbhid_txbuf[6]  = p&0xFF;
	usbhid_txbuf[7]  = (p>>8)&0xFF;
	usbhid_txbuf[8]  = 0;
	usbhid_txbuf[9]  = 0;
	usbhid_txbuf[10] = 0;
	usbhid_txbuf[11] = 0;
	USBD_HID_SendReport(&hUsbDeviceFS, usbhid_txbuf, 12);
	return true;
}
// 
#endif /* PATCHOULI_PCB_DISCRETE_SST */