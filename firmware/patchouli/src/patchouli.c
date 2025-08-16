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

#if defined(PATCHOULI_PCB_DISCRETE_SST)
  #include "patchouli_bsp_discrete_sst.h"
#elif defined(PATCHOULI_PCB_GLIDER_ADDON_V1)
  #include "patchouli_bsp_glider_addon_v1.h"
#endif

#include "patchouli.h"
#include "patchouli_math.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
// #include "usbd_cdc_acms.h"

// #define OLD_IMPL

// Moving Average Filters
patchouli_mva_t mva_xpos;	// X Position
patchouli_mva_t mva_ypos;	// Y Position
patchouli_mva_t mva_p;		// Tip Pressure

patchouli_debug_t debug_state = PATCHOULI_DEBUG_XSCAN;
float    pavg;
int      maxpptr = 7;
uint16_t psamples[PATCHOULI_TIM_N_STEP];
bool     pen_present = false;
float    pen_zero_pressure = 0;

patchouli_position_t xpos, ypos;

patchouli_position_t patchouli_get_position(uint8_t fstep, int lastpos, bool x_yn){
    float smem[PATCHOULI_GWIN];
    const int window_size = PATCHOULI_GWIN;
    const int window_half = window_size/2;
    const int n_coils = x_yn ? PATCHOULI_N_X_COIL : PATCHOULI_N_Y_COIL;
    if (lastpos<window_half)           lastpos =         window_half  ;
    if (lastpos>n_coils-window_half-1) lastpos = n_coils-window_half-1;

    for (int i=0; i<window_size; i++){
        patchouli_coil_select(lastpos-window_half+i,x_yn);
        smem[i] = (float)patchouli_simple_take_sample(!x_yn, fstep) - PATCHOULI_DATA_OFFSET;
    }
    patchouli_quad_interp_t interp = {
        .n = PATCHOULI_GWIN,
        .low = 0,
        .high = PATCHOULI_GWIN-1,
        .data = smem,
    };
    float posavg = patchouli_quad_interp(&interp) + lastpos - window_half;
    patchouli_position_t ret = {
        .idx = interp.result_maxidx + lastpos - window_half,
        .val = posavg,
        .x_yn = x_yn,
    };
    return ret;
}

void patchouli_init(){
	  patchouli_coil_select(0,1);
	  patchouli_TIM_init();
	  patchouli_gauss_init_kernel();
	  patchouli_mva_buffer_init(&mva_p,3);
}

// Default Running Mode
void _patchouli_cycle_default(){
    // Scan X-Y Position
    xpos = patchouli_get_position(maxpptr, xpos.idx, true );
    ypos = patchouli_get_position(maxpptr, ypos.idx, false);
    float xavg = xpos.val;
    float yavg = ypos.val;

	// Scan Frequency to find the peak
	uint16_t maxpval = 0;
	for (int fstep=0;fstep<8;fstep++){
		patchouli_coil_select(xpos.idx, 1);
		psamples[fstep] = patchouli_simple_take_sample(false, fstep);
		if(psamples[fstep]>maxpval){
            maxpval = psamples[fstep];
            maxpptr = fstep;
        }
	}
	if(maxpval>PATCHOULI_PDET_THRES){
		patchouli_led_on();
		// Gauss Fit PDET
		float c1[3];
		patchouli_gauss_apply((psamples+maxpptr-2), c1);
		float t = exp(11-(maxpptr-c1[1]/(2*c1[2])));
		if(t<2000) pavg = patchouli_mva_buffer_push(&mva_p, t);

        // Auto-zeroing
        if (!pen_present) {
            pen_present       = true;
            pen_zero_pressure = pavg;
        }
        const float x_src_min = 0.5f;
        const float x_src_max = 10.5f;
        const float y_src_min = 0.5f;
        const float y_src_max = 7.5f;

        const float x_dst_min = 0.0f;
        const float x_dst_max = 32000.0f;
        const float y_dst_min = 20000.0f;
        const float y_dst_max = 0.0f;

        int x = (int)round(patchouli_linear_map(x_src_min, x_src_max, x_dst_min, x_dst_max, xavg));
        int y = (int)round(patchouli_linear_map(y_src_min, y_src_max, y_dst_min, y_dst_max, yavg));

        const float pmax = 28.8f;
        const float pmin = 970.0f;
        const float prange = pmax-pmin;

        int p = (int)(4096*(((pavg-pen_zero_pressure)/prange)*((pavg-pen_zero_pressure)/prange))); // Square the value to get a linear response
        if (p>4095) p = 4095;
        PATCHOULI_UsrLog("$%d, %d, %d, %d, %d;\r\n", (int)(xavg*1000), (int)(yavg*1000),(int)(pavg*1000), p, maxpval);
		patchouli_report_t report = {0};
		report.xpos = x;
		report.ypos = y;
		report.tip  = p*4;
		patchouli_transmit(&report);
	} else {
        pen_present = false;
		patchouli_led_off();
	}
}

void _patchouli_cycle_xscan(){
    // Scan X Position
    for (int xstep=0;xstep<PATCHOULI_N_X_COIL;xstep++){
        patchouli_coil_select(xstep, true);
        patchouli_simple_take_sample(false, 0);
    }
    // Sleep for 1000 cycles
    for (int i=0;i<50000;i++){
        // Do nothing, just wait
        __asm__ volatile ("nop");
    }
}

void _patchouli_cycle_yscan(){
    // Scan Y Position
    for (int ystep=0;ystep<PATCHOULI_N_Y_COIL;ystep++){
        patchouli_coil_select(ystep, false);
        patchouli_simple_take_sample(true, 0);
    }
    // Sleep for 1000 cycles
    for (int i=0;i<50000;i++){
        // Do nothing, just wait
        __asm__ volatile ("nop");
    }
}
void patchouli_set_mode(patchouli_debug_t mode){
    // todo: reset all global variables when mode changes
    debug_state = mode;
}
void patchouli_cycle(){
    switch (debug_state) {
        case PATCHOULI_DEBUG_NONE:
            _patchouli_cycle_default();
            break;
        case PATCHOULI_DEBUG_XSCAN:
            _patchouli_cycle_xscan();
            break;
        case PATCHOULI_DEBUG_YSCAN:
            _patchouli_cycle_yscan();
            break;
        default:
            // Undefined mode
            PATCHOULI_ErrLog("Undefined mode: %d", debug_state);
    }
}

// Debug helper function implementation
void patchouli_debug_output(const char* prefix, const char* format, ...) {
    char debug_buffer[256];
    va_list args;
    
    // Format the message
    va_start(args, format);
    int len = snprintf(debug_buffer, sizeof(debug_buffer), "%s", prefix);
    len += vsnprintf(debug_buffer + len, sizeof(debug_buffer) - len, format, args);
    len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len, "\n");
    va_end(args);
    
    // Send via CDC
    CDC_Transmit_FS((uint8_t*)debug_buffer, len);
}
