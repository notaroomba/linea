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
#ifndef __PATCHOULI_MATH_H
#define __PATCHOULI_MATH_H
#include "stdint.h"

#define PATCHOULI_GWIN 5


float patchouli_linear_map(float src_min, float src_max, float dst_min, float dst_max, float val);

uint16_t patchouli_median(uint16_t *arr, uint16_t n);
void     patchouli_gauss_init_kernel();
float*   patchouli_gauss_apply(int16_t* arr, float* dst);
float    patchouli_gauss_calc_mean();
float    patchouli_gauss_calc_std();
float    patchouli_gauss_calc_amp();

typedef struct {
    float*   buf;
    int      size;
    int      index;
    float    sum;
    float    mean;
} patchouli_mva_t;

void patchouli_mva_buffer_init(patchouli_mva_t* mva, int size);
float patchouli_mva_buffer_push(patchouli_mva_t* mva, float val);

typedef struct {
    float x[2];
    float P[2][2];
    float Q[2][2];
    float R;
} patchouli_kalman_t;

void patchouli_kalman_init(patchouli_kalman_t* k, float Q, float R);
float patchouli_kalman_update(patchouli_kalman_t* k, float val);

typedef struct {
    float* data;
    float* coefficient;
    float  acc;
    int    len;
} patchouli_fir_t;

typedef enum {
    PATCHOULI_FIR_LPF,
    PATCHOULI_FIR_HPF,
    PATCHOULI_FIR_BPF,
    PATCHOULI_FIR_BSF
} patchouli_fir_type_t;

void patchouli_fir_init(patchouli_fir_t* fir, int len, patchouli_fir_type_t ftype, float order, float bw, float fc);
void patchouli_fir_free(patchouli_fir_t* fir);
float patchouli_fir_update(patchouli_fir_t* fir, float val);

typedef enum {
    PATCHOULI_QUAD_INTERP_LEFT,
    PATCHOULI_QUAD_INTERP_NORMAL,
    PATCHOULI_QUAD_INTERP_RIGHT
} patchouli_quad_interp_boundary_t;

typedef struct {
    uint8_t n;
    float   low;
    float   high;
    float*  data;
    int     result_maxidx;
    float   result_interp;
    float   result_mapped;
    patchouli_quad_interp_boundary_t boundary_status;
} patchouli_quad_interp_t;

float patchouli_quad_interp(patchouli_quad_interp_t* q);

#endif /* __PATCHOULI_MATH_H */
