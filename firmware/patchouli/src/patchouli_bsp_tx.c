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

// This file is a workaround for the timing in the SST v2 board.
// Ideally, the TRX switching in each scanning action includes three 
// steps: Output a square wave, change the pin to tri-state mode after the
// transmission is done, and release the integrator's discharge pin after a 
// short delay.
// The timing of all three steps are critical, particularly the last two.
// Ideally we should use a timer to generate all three signals. 
// However, without an external tri-state driver, this is impossible because 
// STM32 timers cannot write directly to registers. 
// The current workaround is to use cycle-accurate assembly code to generate 
// the signals. This is not ideal because:
// 1. The code is not portable between architectures.
// 2. The clock frequency of the CPU cannot be changed.
// 3. Won't work on chips with an instruction cache.

#include "patchouli_config.h"
#include "patchouli_bsp_tx.h"
#include "main.h"

#if defined(PATCHOULI_PCB_DISCRETE_SST)
  #include "patchouli_bsp_discrete_sst.h"
#elif defined(PATCHOULI_PCB_GLIDER_ADDON_V1)
  #include "patchouli_bsp_glider_addon_v1.h"
#endif

extern uint32_t SystemCoreClock; // CMSIS System Clock

void _PW100_FSTEP_480(void);
void _PW100_FSTEP_490(void);
void _PW100_FSTEP_500(void);
void _PW100_FSTEP_510(void);
void _PW100_FSTEP_515(void);
void _PW100_FSTEP_520(void);
void _PW100_FSTEP_525(void);
void _PW100_FSTEP_530(void);
void _PW100_FSTEP_535(void);
void _PW100_FSTEP_540(void);
void _PW100_FSTEP_545(void);
void _PW100_FSTEP_550(void);

patchouli_void_fptr_t patchouli_tx_fptr_table[] = {
    _PW100_FSTEP_510,
    _PW100_FSTEP_515,
    _PW100_FSTEP_520,
    _PW100_FSTEP_525,
    _PW100_FSTEP_530,
    _PW100_FSTEP_535,
    _PW100_FSTEP_540,
    _PW100_FSTEP_545
};

patchouli_tx_t patchouli_pen_pw100 = {
    // .tx_fmin_kHz   = 470,
    // .tx_fmax_kHz   = 550,
    .tx_fmin_kHz   = 510,
    .tx_fmax_kHz   = 545,
    .tx_fstep_kHz  = 5,
    .tx_steps      = 8,
    .tx_ncycle     = 50,
    .cpu_freq      = 64.0E6f,
    .tx_fptr_table = patchouli_tx_fptr_table
};

#define _5NOP()   do{asm volatile ("nop\nnop\nnop\nnop\nnop");} while(0)
#define _10NOP()  do{_5NOP();_5NOP();} while(0)
#define _100NOP() do{_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();}while(0)

void _PW100_FSTEP_480(void){
    // 480kHz
    // need 300 cycles
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_490(void){
    // 490kHz
    // need 294 cycles
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        _5NOP();
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_500(void){
    // 500kHz
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_510(void){
    // 510kHz
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();_10NOP();
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_515(void){
    // 516.2kHz
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_520(void){
    // 520kHz
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_525(void){
    // 525.5kHz
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_530(void){
    // 530kHz
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        _5NOP();
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_535(void){
    // 535.2kHz
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_540(void){
    // 540kHz, 266.7 cycles
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_545(void){
    // 545.3kHz
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();_10NOP();
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_550(void){
    // 550kHz, 261.8 cycles
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        _100NOP();
        _10NOP();_10NOP();_10NOP();
        PATCHOULI_TX_LOW();
        _100NOP();
        _10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

// PW100
// 0   1   2   3   4   5   6   7
// 480 490 500 510 520 530 540 550
