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
//include all the high frequency steps from 650-780kHz
// void _PW100_FSTEP_650(void);
// void _PW100_FSTEP_660(void);
// void _PW100_FSTEP_670(void);
// void _PW100_FSTEP_680(void);
// void _PW100_FSTEP_690(void);
// void _PW100_FSTEP_700(void);
// void _PW100_FSTEP_710(void);
// void _PW100_FSTEP_720(void);
// void _PW100_FSTEP_730(void);
// void _PW100_FSTEP_740(void);
// void _PW100_FSTEP_750(void);
// void _PW100_FSTEP_760(void);
// void _PW100_FSTEP_770(void);
// void _PW100_FSTEP_780(void);

patchouli_void_fptr_t patchouli_tx_fptr_table[] = {
    _PW100_FSTEP_480,
    _PW100_FSTEP_490,
    _PW100_FSTEP_510,
    // _PW100_FSTEP_515,
    _PW100_FSTEP_520,
    // _PW100_FSTEP_525,
    _PW100_FSTEP_530,
    _PW100_FSTEP_535,
    _PW100_FSTEP_540,
    // _PW100_FSTEP_545,
    _PW100_FSTEP_550,

};

// patchouli_void_fptr_t patchouli_high_tx_fptr_table[] = {
//     _PW100_FSTEP_650,
//     _PW100_FSTEP_660,
//     _PW100_FSTEP_670,
//     _PW100_FSTEP_680,
//     _PW100_FSTEP_690,
//     _PW100_FSTEP_700,
//     _PW100_FSTEP_710,
//     _PW100_FSTEP_720,
//     _PW100_FSTEP_730,
//     _PW100_FSTEP_740,
//     _PW100_FSTEP_750,
//     _PW100_FSTEP_760,
//     _PW100_FSTEP_770,
//     _PW100_FSTEP_780
// };

patchouli_tx_t patchouli_pen_pw100 = {
    .tx_fmin_kHz   = 480,
    .tx_fmax_kHz   = 550,
    // .tx_fmin_kHz   = 510,
    // .tx_fmax_kHz   = 545,
    .tx_fstep_kHz  = 10,
    .tx_steps      = 8,
    .tx_ncycle     = 50,
    .cpu_freq      = 64.0E6f,
    .tx_fptr_table = patchouli_tx_fptr_table
};

// patchouli_tx_t patchouli_pen_pw100_high = {
//     .tx_fmin_kHz   = 650,
//     .tx_fmax_kHz   = 780,
//     .tx_fstep_kHz  = 10,
//     .tx_steps      = 14,
//     .tx_ncycle     = 50,
//     .cpu_freq      = 64.0E6f,
//     .tx_fptr_table = patchouli_high_tx_fptr_table
// };


// Optimized NOP macros for precise timing
#define _5NOP()   do{asm volatile ("nop\nnop\nnop\nnop\nnop");} while(0)
#define _10NOP()  do{_5NOP();_5NOP();} while(0)
#define _100NOP() do{_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();}while(0)

// Timing calculation: For 64MHz CPU, each NOP = 1 cycle
// Period = 64MHz / Target_Frequency
// Half-period = Period / 2
// NOPs needed = Half-period - GPIO_operation_overhead
// GPIO overhead ≈ 2-3 cycles for BSRR operations

void _PW100_FSTEP_480(void){
    // 480kHz target frequency
    // Period = 64MHz / 480kHz = 133.33 cycles
    // Half-period = 66.67 cycles
    // HIGH duration = 66 cycles (accounting for GPIO overhead)
    // LOW duration = 67 cycles (accounting for GPIO overhead)
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t i;
    for (i=0; i<ncycle; i++){
        PATCHOULI_TX_HIGH();
        // 66 cycles: 6*10 + 1*5 + 1 = 66
        _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP();
        _5NOP();
        asm volatile ("nop");
        
        PATCHOULI_TX_LOW();
        // 67 cycles: 6*10 + 1*5 + 2 = 67
        _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_490(void){
    // 490kHz target frequency
    // Period = 64MHz / 490kHz = 130.61 cycles
    // Half-period = 65.31 cycles
    // HIGH duration = 65 cycles (accounting for GPIO overhead)
    // LOW duration = 66 cycles (accounting for GPIO overhead)
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t i;
    for (i=0; i<ncycle; i++){
        PATCHOULI_TX_HIGH();
        // 65 cycles: 6*10 + 1*5 = 65
        _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP();
        _5NOP();
        
        PATCHOULI_TX_LOW();
        // 66 cycles: 6*10 + 1*5 + 1 = 66
        _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP();
        _5NOP();
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_500(void){
    // 500kHz target frequency
    // Period = 64MHz / 500kHz = 128 cycles
    // Half-period = 64 cycles
    // HIGH duration = 64 cycles (accounting for GPIO overhead)
    // LOW duration = 64 cycles (accounting for GPIO overhead)
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t i;
    for (i=0; i<ncycle; i++){
        PATCHOULI_TX_HIGH();
        // 64 cycles: 6*10 + 1*5 - 1 = 64
        _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP();
        _5NOP();
        
        PATCHOULI_TX_LOW();
        // 64 cycles: 6*10 + 1*5 - 1 = 64
        _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP(); _10NOP();
        _5NOP();
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_510(void){
    // 510kHz
    // need 125.4 cycles (new) (64MHz / 510kHz)
    // 125 - 13 = 112
    // 112 / 2 = 56
    // 56 - 5.5 = 51
    // 56 + 5.5 = 61.5
    // 51 to 62
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        // 62
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        // 51
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_515(void){
    // 516.2kHz
    // need 124 cycles (new) (64MHz / 516.2kHz)
    // 124 - 13 = 111
    // 111 / 2 = 55.5
    // 55.5 - 5.5 = 50
    // 55.5 + 5.5 = 61
    // 50 to 61
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        // 61
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        // 50
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_520(void){
    // 520kHz
    // need 123 cycles (new) (64MHz / 520kHz)
    // 123 - 13 = 110
    // 110 / 2 = 55
    // 55 - 5.5 = 49.5
    // 55 + 5.5 = 60.5
    // 49.5 to 60.5
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        // 60
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        PATCHOULI_TX_LOW();
        // 50
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_525(void){
    // 525.5kHz
    // need 121 cycles (new) (64MHz / 525.5kHz)
    // 121 - 13 = 108
    // 108 / 2 = 54
    // 54 - 5.5 = 48.5
    // 54 + 5.5 = 59.5
    // 48.5 to 59.5
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        //60
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        PATCHOULI_TX_LOW();
        //49
        _10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_530(void){
    // 530kHz
    // need 120 cycles (new) (64MHz / 530kHz)
    // 120 - 13 = 107
    // 107 / 2 = 53.5
    // 53.5 - 5.5 = 48
    // 53.5 + 5.5 = 59
    // 48 to 59
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        // 59
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        // 48
        _10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_535(void){
    // 535.2kHz
    // need 119 cycles (new) (64MHz / 535.2kHz)
    // 119 - 13 = 106
    // 106 / 2 = 53
    // 53 - 5.5 = 47.5
    // 53 + 5.5 = 58.5
    // 47.5 to 58.5
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        // 58
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        // 48
        _10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_540(void){
    // 540kHz, 266.7 cycles (old)
    // need 118 cycles (new) (64MHz / 540kHz)
    // 118 - 13 = 105
    // 105 / 2 = 52.5
    // 52.5 - 5.5 = 47
    // 52.5 + 5.5 = 58
    // 47 to 58
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
            // 58
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        // 47
        _10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_545(void){
    // 545.3kHz
    // need 117 cycles (new) (64MHz / 545.3kHz)
    // 117 - 13 = 104
    // 104 / 2 = 52
    // 52 - 5.5 = 46.5
    // 52 + 5.5 = 57.5
    // 46.5 to 57.5
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        // 57
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        // 47
        _10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

void _PW100_FSTEP_550(void){
    // 550kHz, 261.8 cycles (old)
    // need 116 cycles (new) (64MHz / 550kHz)
    // 116 - 13 = 103
    // 103 / 2 = 51.5
    // 51.5 - 5.5 = 46
    // 51.5 + 5.5 = 57
    // 46 to 57
    PATCHOULI_TX_PP();
    const int ncycle = patchouli_pen_pw100.tx_ncycle;
    uint16_t  i;
    for (i=0; i<ncycle; i++){ // 3 cycles
        PATCHOULI_TX_HIGH();
        // 57
        _10NOP();_10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
        PATCHOULI_TX_LOW();
        // 46
        _10NOP();_10NOP();_10NOP();_10NOP();
        _5NOP();
        asm volatile ("nop");
        asm volatile ("nop");
    }
    PATCHOULI_TX_TRISTATE();
}

// PW100
// 0   1   2   3   4   5   6   7
// 480 490 500 510 520 530 540 550


// OLD

// void _PW100_FSTEP_480(void){
//     // 480kHz
//     // need 300 cycles
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();_10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_490(void){
//     // 490kHz
//     // need 294 cycles
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();_10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         _5NOP();
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_500(void){
//     // 500kHz
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_510(void){
//     // 510kHz
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();_10NOP();
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_515(void){
//     // 516.2kHz
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_520(void){
//     // 520kHz
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_525(void){
//     // 525.5kHz
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_530(void){
//     // 530kHz
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         _5NOP();
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_535(void){
//     // 535.2kHz
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_540(void){
//     // 540kHz, 266.7 cycles
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_545(void){
//     // 545.3kHz
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();_10NOP();
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }

// void _PW100_FSTEP_550(void){
//     // 550kHz, 261.8 cycles
//     PATCHOULI_TX_PP();
//     const int ncycle = patchouli_pen_pw100.tx_ncycle;
//     uint16_t  i;
//     for (i=0; i<ncycle; i++){ // 3 cycles
//         PATCHOULI_TX_HIGH();
//         _100NOP();
//         _10NOP();_10NOP();_10NOP();
//         PATCHOULI_TX_LOW();
//         _100NOP();
//         _10NOP();
//         _5NOP();
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//         asm volatile ("nop");
//     }
//     PATCHOULI_TX_TRISTATE();
// }