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
#include "usbd_cdc_if.h"
// #include "usbd_cdc_acms.h"

// #define OLD_IMPL

// Moving Average Filters
patchouli_mva_t mva_xpos;	// X Position
patchouli_mva_t mva_ypos;	// Y Position
patchouli_mva_t mva_p;		// Tip Pressure

patchouli_debug_t debug_state = PATCHOULI_DEBUG_NONE;
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
    CDC_Transmit_FS((uint8_t*)"Posavg:\n", 8);
    char buf[17];
    sprintf(buf, "%f\n", posavg);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));    
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
    // CDC_Transmit_FS((uint8_t*)"X-Y Position\n", 13);
    float xavg = xpos.val;
    float yavg = ypos.val;
    
    char buf[17]; // Buffer for string conversion

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
    // CDC_Transmit_FS((uint8_t*)"Max Pressure\n", 13);

    // CDC_Transmit_FS((uint8_t*)"Max Pressure Value:\n", 20);
    // sprintf(buf, "%d\n", maxpval);
    // uint8_t res = CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
    // if(res == 0){
    //     CDC_Transmit_FS((uint8_t*)"Successfully transmitted maxpval\n", 33);
    // } else if(res == 1) {
    //     CDC_Transmit_FS((uint8_t*)"Busy\n", 5);
    // } else {
    //     CDC_Transmit_FS((uint8_t*)"Failed\n", 7);
    // }
    // CDC_Transmit_FS((uint8_t*)"\n", 1);
	if(maxpval>PATCHOULI_PDET_THRES){
		patchouli_led_on();
		// Gauss Fit PDET
        CDC_Transmit_FS((uint8_t*)"Gauss Fit PDET\n", 15);
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

        // Print the mapped coordinates
        CDC_Transmit_FS((uint8_t*)"X:", 2);
        sprintf(buf, "%d\n", x);
        CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
        CDC_Transmit_FS((uint8_t*)"Y:", 2);
        sprintf(buf, "%d\n", y);
        CDC_Transmit_FS((uint8_t*)buf, strlen(buf));

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
        // CDC_Transmit_FS((uint8_t*)"Pen not present\n", 17);
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
    CDC_Transmit_FS((uint8_t*)"Cycle\n", 6);
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
        case PATCHOULI_DEBUG_PEN_SCAN:
            patchouli_scan_passive_pen_parameters();
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
    CDC_Transmit_FS(debug_buffer, len);
}

// Passive pen detection and parameter logging helper function
void patchouli_scan_passive_pen_parameters() {
    char buf[32];
    uint16_t max_val = 0;
    uint16_t min_val = 0xFFFF;
    uint32_t total_sum = 0;
    uint16_t sample_count = 0;
    
    CDC_Transmit_FS((uint8_t*)"=== PASSIVE PEN SCAN ===\n", 25);
    
    // Scan through all X coils at different frequencies
    CDC_Transmit_FS((uint8_t*)"Scanning X coils:\n", 18);
    for (int coil = 0; coil < PATCHOULI_N_X_COIL; coil++) {
        patchouli_coil_select(coil, true);
        
        for (int fstep = 0; fstep < 8; fstep++) {
            uint16_t sample = patchouli_simple_take_sample(false, fstep);
            
            // Track min/max/average
            if (sample > max_val) max_val = sample;
            if (sample < min_val) min_val = sample;
            total_sum += sample;
            sample_count++;
            
            // Log individual readings
            sprintf(buf, "X%d-F%d: %d\n", coil, fstep, sample);
            CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
        }
    }
    
    // Scan through all Y coils at different frequencies
    CDC_Transmit_FS((uint8_t*)"Scanning Y coils:\n", 18);
    for (int coil = 0; coil < PATCHOULI_N_Y_COIL; coil++) {
        patchouli_coil_select(coil, false);
        
        for (int fstep = 0; fstep < 8; fstep++) {
            uint16_t sample = patchouli_simple_take_sample(true, fstep);
            
            // Track min/max/average
            if (sample > max_val) max_val = sample;
            if (sample < min_val) min_val = sample;
            total_sum += sample;
            sample_count++;
            
            // Log individual readings
            sprintf(buf, "Y%d-F%d: %d\n", coil, fstep, sample);
            CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
        }
    }
    
    // Calculate and log statistics
    uint16_t avg_val = total_sum / sample_count;
    uint16_t range = max_val - min_val;
    
    CDC_Transmit_FS((uint8_t*)"=== SCAN STATISTICS ===\n", 24);
    sprintf(buf, "Min: %d\n", min_val);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
    sprintf(buf, "Max: %d\n", max_val);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
    sprintf(buf, "Avg: %d\n", avg_val);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
    sprintf(buf, "Range: %d\n", range);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
    sprintf(buf, "Samples: %d\n", sample_count);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
    
    // Suggest threshold based on statistics
    uint16_t suggested_threshold = avg_val + (range / 4);
    sprintf(buf, "Suggested threshold: %d\n", suggested_threshold);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
    
    CDC_Transmit_FS((uint8_t*)"=== SCAN COMPLETE ===\n", 22);
}
