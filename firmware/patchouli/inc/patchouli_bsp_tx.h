#ifndef _PATCHOULI_BSP_TX_H_
#define _PATCHOULI_BSP_TX_H_

#include "stdint.h"

// define void function handle type as patchouli_tx_handler_t
typedef void (*patchouli_void_fptr_t)(void);
// Define a structure
typedef struct {
    uint16_t               tx_fmin_kHz;
    uint16_t               tx_fmax_kHz;
    uint16_t               tx_fstep_kHz;
    uint8_t                tx_steps;
    uint16_t               tx_ncycle;
    float                  cpu_freq;
    patchouli_void_fptr_t* tx_fptr_table;
} patchouli_tx_t;


#endif /* _PATCHOULI_BSP_TX_H_ */