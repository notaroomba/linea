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
#ifdef PATCHOULI_PCB_GLIDER_ADDON_V1

#include "patchouli_bsp_glider_addon_v1.h"
#include "main.h"
#include "math.h"
#include "patchouli_math.h"
#include "patchouli.h"
#include "BCG.h"
#include "patchouli_bsp_tx.h"
#include "usbd_cdc_if.h"

// ExN List
#define PATCHOULI_N_EXN (6u)
GPIO_TypeDef* ExN_GPIO_Port[PATCHOULI_N_EXN] = {
    E0N_GPIO_Port, E1N_GPIO_Port, E2N_GPIO_Port,
    E3N_GPIO_Port, E4N_GPIO_Port, E5N_GPIO_Port
};
uint16_t ExN_Pin[PATCHOULI_N_EXN] = {
    E0N_Pin, E1N_Pin, E2N_Pin,
    E3N_Pin, E4N_Pin, E5N_Pin
};


void patchouli_coil_select(int id, bool x_yn){
    uint8_t S   = 0;
    GPIO_PinState    ExN[PATCHOULI_N_EXN];
    for (int i = 0; i < PATCHOULI_N_EXN; i++) {
        ExN[i] = GPIO_PIN_SET; // Default state is high
    }

    if(x_yn) {
        // X Coils
        if(id >= PATCHOULI_N_X_COIL) {
            PATCHOULI_ErrLog("Invalid X Coil ID: %d", id);
            // CDC_Transmit_FS((uint8_t*)"Invalid X Coil ID\n", 19);
            return;
        }
        switch(id) {
            case 0:  ExN[0] = GPIO_PIN_RESET; S = 1; break; // U7
            case 1:  ExN[0] = GPIO_PIN_RESET; S = 2; break; 
            case 2:  ExN[0] = GPIO_PIN_RESET; S = 4; break; 
            case 3:  ExN[0] = GPIO_PIN_RESET; S = 6; break; 
            case 4:  ExN[0] = GPIO_PIN_RESET; S = 7; break; 
            case 5:  ExN[0] = GPIO_PIN_RESET; S = 5; break; 
            case 6:  ExN[1] = GPIO_PIN_RESET; S = 3; break; // U8
            case 7:  ExN[1] = GPIO_PIN_RESET; S = 0; break; 
            case 8:  ExN[1] = GPIO_PIN_RESET; S = 1; break; 
            case 9:  ExN[1] = GPIO_PIN_RESET; S = 2; break; 
            case 10:  ExN[1] = GPIO_PIN_RESET; S = 4; break; 
            case 11:  ExN[1] = GPIO_PIN_RESET; S = 6; break; 
            case 12:  ExN[1] = GPIO_PIN_RESET; S = 7; break; 
            case 13:  ExN[1] = GPIO_PIN_RESET; S = 5; break; 
            case 14:  ExN[2] = GPIO_PIN_RESET; S = 3; break; // U9 
            case 15:  ExN[2] = GPIO_PIN_RESET; S = 0; break;
            case 16:  ExN[2] = GPIO_PIN_RESET; S = 1; break;
            case 17:  ExN[2] = GPIO_PIN_RESET; S = 2; break;
            case 18:  ExN[2] = GPIO_PIN_RESET; S = 4; break;
            case 19:  ExN[2] = GPIO_PIN_RESET; S = 6; break;
            case 20:  ExN[2] = GPIO_PIN_RESET; S = 7; break;
            case 21:  ExN[2] = GPIO_PIN_RESET; S = 5; break;
            case 22:  ExN[3] = GPIO_PIN_RESET; S = 3; break; // U10
            case 23:  ExN[3] = GPIO_PIN_RESET; S = 0; break;
            case 24:  ExN[3] = GPIO_PIN_RESET; S = 1; break;
            case 25:  ExN[3] = GPIO_PIN_RESET; S = 2; break;
            case 26:  ExN[3] = GPIO_PIN_RESET; S = 4; break;
            case 27:  ExN[3] = GPIO_PIN_RESET; S = 6; break;
            default: 
                PATCHOULI_ErrLog("Invalid X Coil ID: %d", id); 
                // CDC_Transmit_FS((uint8_t*)"Invalid X Coil ID2\n", 20);
                return;
        }
    } else {
        // Y Coils
        if(id >= PATCHOULI_N_Y_COIL) {
            PATCHOULI_ErrLog("Invalid Y Coil ID: %d", id);
            // CDC_Transmit_FS((uint8_t*)"Invalid Y Coil ID\n", 19);
            return;
        }
        switch (id) {
            case 0:  ExN[4] = GPIO_PIN_RESET; S = 3; break; // U11
            case 1:  ExN[4] = GPIO_PIN_RESET; S = 0; break;
            case 2:  ExN[4] = GPIO_PIN_RESET; S = 1; break;
            case 3:  ExN[4] = GPIO_PIN_RESET; S = 2; break;
            case 4:  ExN[4] = GPIO_PIN_RESET; S = 4; break;
            case 5:  ExN[4] = GPIO_PIN_RESET; S = 6; break;
            case 6:  ExN[4] = GPIO_PIN_RESET; S = 7; break;
            case 7:  ExN[4] = GPIO_PIN_RESET; S = 5; break;
            case 8:  ExN[5] = GPIO_PIN_RESET; S = 3; break; // U12
            case 9:  ExN[5] = GPIO_PIN_RESET; S = 0; break;
            case 10:  ExN[5] = GPIO_PIN_RESET; S = 1; break;
            case 11:  ExN[5] = GPIO_PIN_RESET; S = 2; break;
            case 12:  ExN[5] = GPIO_PIN_RESET; S = 4; break;
            case 13:  ExN[5] = GPIO_PIN_RESET; S = 6; break;
            case 14:  ExN[5] = GPIO_PIN_RESET; S = 7; break;
            case 15:  ExN[5] = GPIO_PIN_RESET; S = 5; break;
            case 16:  ExN[0] = GPIO_PIN_RESET; S = 3; break; // U7
            case 17:  ExN[0] = GPIO_PIN_RESET; S = 0; break;
            
            default: 
                PATCHOULI_ErrLog("Invalid Y Coil ID: %d", id);
                // CDC_Transmit_FS((uint8_t*)"Invalid Y Coil ID2\n", 20);
                return;
        }
    }
    HAL_GPIO_WritePin(S0_GPIO_Port,  S0_Pin,  (S & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S1_GPIO_Port,  S1_Pin,  ((S >> 1) & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S2_GPIO_Port,  S2_Pin,  ((S >> 2) & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
    for (int i = 0; i < PATCHOULI_N_EXN; i++) {
        HAL_GPIO_WritePin(ExN_GPIO_Port[i], ExN_Pin[i], ExN[i]);
    }
    return;
}

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim1;
extern float gfreq;

void patchouli_TIM_init(){
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
    gfreq = PATCHOULI_TIM_F_CENTER;
    uint16_t d = (PATCHOULI_TIM_FAST_CLK/gfreq);
    htim16.Instance->ARR = d;
    htim16.Instance->CCR1 = d>>1;
    htim16.Instance->RCR = 60;
}

void patchouli_TIM_trigger(){
    htim16.Instance->CR1 |= B16(00000000,00000001); // Enable TIM1
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
            CDC_Transmit_FS(sample[i], 2);
            CDC_Transmit_FS("\n", 1);
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
      //before 400 but due to clocks eedp 178
          for (int i=0;i<178;i++)  asm volatile ("nop");
      else // before 100 now 44
          for (int i=0;i<44;i++)  asm volatile ("nop");
      HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET);
      // before 5k no 2222
      for (int i=0;i<2222;i++) asm volatile ("nop");
      uint16_t value = _patchouli_adc_multisample();
    //   CDC_Transmit_FS(value, 2);
    //   CDC_Transmit_FS("\n", 1);

# ifdef PATCHOULI_CDS
      HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_SET);
      // before 100 now 
      for (int i=0;i<44;i++) asm volatile ("nop");
      HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET);
      //before 200 now 88.8
      for (int i=0;i<89;i++) asm volatile ("nop");
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
      htim16.Instance->ARR  = t;
      htim16.Instance->CCR1 = (t/2);
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
    // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void patchouli_led_off(){
    // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

extern UART_HandleTypeDef huart1;
extern uint8_t huart1_rxbuffer[256];
// Init all communication drivers (UART, USB, etc)
void patchouli_comms_init(){
    HAL_UART_Receive_IT(&huart1, (uint8_t *)huart1_rxbuffer, 1);

}
// #include "usbd_hid.h"
// extern uint8_t usbhid_txbuf[12];
// extern USBD_HandleTypeDef hUsbDeviceFS;
// bool patchouli_transmit(patchouli_report_t* report){
//     int16_t p = report->tip;
//     int16_t x = report->xpos;
//     int16_t y = report->ypos;
//     // printf("%d\t%d\t%d\r\n", p,x,y);
//     usbhid_txbuf[0]  = 0x08;
//     usbhid_txbuf[1]  = 0x80;
//     usbhid_txbuf[2]  = x&0xFF;
//     usbhid_txbuf[3]  = (x>>8)&0xFF;
//     usbhid_txbuf[4]  = y&0xFF;
//     usbhid_txbuf[5]  = (y>>8)&0xFF;
//     usbhid_txbuf[6]  = p&0xFF;
//     usbhid_txbuf[7]  = (p>>8)&0xFF;
//     usbhid_txbuf[8]  = 0;
//     usbhid_txbuf[9]  = 0;
//     usbhid_txbuf[10] = 0;
//     usbhid_txbuf[11] = 0;
//     USBD_HID_SendReport(&hUsbDeviceFS, usbhid_txbuf, 12);
//     return true;
// }
#include "usb_device.h"
bool patchouli_transmit(patchouli_report_t* report){
    // through cdc
    CDC_Transmit_FS("Transmit\n", 9);
    int16_t p = report->tip;
    int16_t x = report->xpos;
    int16_t y = report->ypos;
    char txbuf[32];
    sprintf(txbuf, "%d\t%d\t%d\n", x, y, p);
    CDC_Transmit_FS(txbuf, strlen(txbuf));
    return true;
}
#endif /* PATCHOULI_PCB_GLIDER_ADDON_V1 */