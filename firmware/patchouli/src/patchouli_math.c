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
// This file implements the following math and signal processing functions 
// for the Patchouli project.
// - median filter
// - weighted average filter
// - Gaussian fitting
// - moving average filter
// - Kalman filter
// - FIR filter & coefficient calculation

#include "patchouli_math.h"
#include "arm_math.h" 
#include "malloc.h"

//----------------------------------------------------------------
// Mapping
//----------------------------------------------------------------
float patchouli_linear_map(float src_min, float src_max, float dst_min, float dst_max, float val){
    return dst_min + (dst_max-dst_min)*(val-src_min)/(src_max-src_min);
}

//----------------------------------------------------------------
// Median 
//----------------------------------------------------------------
uint16_t patchouli_median(uint16_t *arr, uint16_t n) {
  uint16_t temp;
  for (uint16_t i = 0; i < n; i++) {
    for (uint16_t j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }
  return arr[n / 2];
}

//----------------------------------------------------------------
// Weighted Average
//----------------------------------------------------------------
float patchouli_wavg_f32(float* pos, float* weight, int len){
    float sum = 0;
    float wsum = 0;
    for (int i=0; i<len; i++){
        sum  += pos[i]*weight[i];
        wsum += weight[i];
    }
    return sum/wsum;
}

//----------------------------------------------------------------
// Gaussian fitting
//----------------------------------------------------------------
float32_t               patchouli_kernel_f32[3*PATCHOULI_GWIN];
arm_matrix_instance_f32 patchouli_kernel_mat;
float32_t               patchouli_c_f32[3];
arm_matrix_instance_f32 patchouli_c_mat;
float32_t               patchouli_b_f32[PATCHOULI_GWIN];
arm_matrix_instance_f32 patchouli_b_mat;

// Compute the kernel for the Gaussian fit
void patchouli_gauss_init_kernel(){
    float32_t A_f32 [3*PATCHOULI_GWIN];
    float32_t AT_f32[3*PATCHOULI_GWIN];
    float32_t ATMA_f32[3*3];
    float32_t ATMAI_f32[3*3];
    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 AT;
    arm_matrix_instance_f32 ATMA;
    arm_matrix_instance_f32 ATMAI;
    arm_mat_init_f32(&A, PATCHOULI_GWIN,3 ,  A_f32 );
    arm_mat_init_f32(&AT, 3, PATCHOULI_GWIN, AT_f32);
    arm_mat_init_f32(&ATMA, 3, 3, ATMA_f32);
    arm_mat_init_f32(&ATMAI, 3, 3, ATMAI_f32);
    arm_mat_init_f32(&patchouli_kernel_mat, 3, PATCHOULI_GWIN, patchouli_kernel_f32);
    arm_mat_init_f32(&patchouli_b_mat, PATCHOULI_GWIN,1, patchouli_b_f32);
    arm_mat_init_f32(&patchouli_c_mat, 3,1, patchouli_c_f32);
    for (int x=0; x<PATCHOULI_GWIN;x++) {
        A_f32[x*3+0] = 1;
        A_f32[x*3+1] = x+1;
        A_f32[x*3+2] = (x+1)*(x+1);
    }
    arm_mat_trans_f32(&A, &AT);
    arm_mat_mult_f32(&AT, &A, &ATMA);
    arm_mat_inverse_f32(&ATMA, &ATMAI);
    arm_mat_mult_f32(&ATMAI, &AT, &patchouli_kernel_mat);
}
// Apply the Gaussian fit
float* patchouli_gauss_apply(int16_t* arr, float* dst){
    for (int x=0; x<PATCHOULI_GWIN;x++) {
        patchouli_b_f32[x] = (float)arr[x];
    }
    arm_mat_mult_f32(&patchouli_kernel_mat, &patchouli_b_mat, &patchouli_c_mat);
    memcpy(dst, patchouli_c_f32, 3*sizeof(float));
    return dst;
}
float patchouli_gauss_calc_mean(){return -patchouli_c_f32[1]/(2.0f*patchouli_c_f32[2]);}
float patchouli_gauss_calc_std(){return sqrtf(1.0f/(2.0f*patchouli_c_f32[2]));}
float patchouli_gauss_calc_amp(){return patchouli_c_f32[0]-patchouli_c_f32[1]*patchouli_c_f32[1]/(4.0f*patchouli_c_f32[2]);}

//----------------------------------------------------------------
// Moving average
//----------------------------------------------------------------
void patchouli_mva_buffer_init(patchouli_mva_t* mva, int size){
    mva->buf = (float*)malloc(size*sizeof(float));
    mva->size = size;
    mva->index = 0;
    mva->sum = 0;
    mva->mean = 0;
    for (int i=0; i<size; i++) mva->buf[i] = 0;
}

void patchouli_mva_buffer_free(patchouli_mva_t* mva){
    free(mva->buf);
}

float patchouli_mva_buffer_push(patchouli_mva_t* mva, float val){
    mva->sum -= mva->buf[mva->index];
    mva->buf[mva->index] = val;
    mva->sum += val;
    mva->index = (mva->index+1)%mva->size;
    mva->mean = mva->sum/mva->size;
    return mva->mean;
}

//----------------------------------------------------------------
// Kalman filtering
//----------------------------------------------------------------
void patchouli_kalman_init(patchouli_kalman_t* k, float Q, float R){
    k->x[0] = 0;
    k->x[1] = 0;
    k->P[0][0] = 0;
    k->P[0][1] = 0;
    k->P[1][0] = 0;
    k->P[1][1] = 0;
    k->Q[0][0] = Q;
    k->Q[0][1] = 0;
    k->Q[1][0] = 0;
    k->Q[1][1] = Q;
    k->R = R;
}

float patchouli_kalman_update(patchouli_kalman_t* k, float val){
    float y, S, K[2];
    y = val - k->x[0];
    S = k->P[0][0] + k->R;
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;
    k->x[0] += K[0] * y;
    k->x[1] += K[1] * y;
    k->P[0][0] -= K[0] * k->P[0][0];
    k->P[0][1] -= K[0] * k->P[0][1];
    k->P[1][0] -= K[1] * k->P[0][0];
    k->P[1][1] -= K[1] * k->P[0][1];
    k->P[0][0] += k->Q[0][0];
    k->P[0][1] += k->Q[0][1];
    k->P[1][0] += k->Q[1][0];
    k->P[1][1] += k->Q[1][1];
    return k->x[0];
}

//----------------------------------------------------------------
// FIR filter
//----------------------------------------------------------------
void patchouli_fir_init(patchouli_fir_t* fir, int len, patchouli_fir_type_t ftype, float order, float bw, float fc){
    fir->len         = len;
    fir->data        = (float*)malloc(len*sizeof(float));
    fir->coefficient = (float*)malloc(len*sizeof(float));
    switch (ftype){
        case PATCHOULI_FIR_LPF:
            // Calculate the coefficients for a low-pass filter
            for (int i=0; i<len; i++){
                fir->coefficient[i] = 2.0f*bw*sinf(PI*(i-len/2.0f)*order)/(PI*(i-len/2.0f));
            }
            break;
        case PATCHOULI_FIR_HPF:
            // Calculate the coefficients for a high-pass filter
            for (int i=0; i<len; i++){
                if (i==len/2){
                    fir->coefficient[i] = 1.0f - 2.0f*bw;
                } else {
                    fir->coefficient[i] = -2.0f*bw*sinf(PI*(i-len/2.0f)*order)/(PI*(i-len/2.0f));
                }
            }
            break;
        default:
            // Not implemented yet
            break;
    }
}

void patchouli_fir_free(patchouli_fir_t* fir){
    free(fir->data);
    free(fir->coefficient);
}

float patchouli_fir_update(patchouli_fir_t* fir, float val){
    // Push new data
    for (int i=fir->len-1; i>0; i--){
        fir->data[i] = fir->data[i-1];
    }
    fir->data[0] = val;
    // Apply the filter
    fir->acc = 0;
    for (int i=0; i<fir->len; i++){
        fir->acc += fir->data[i]*fir->coefficient[i];
    }
    return fir->acc;
}

//----------------------------------------------------------------
// Quadratic interpolation
//----------------------------------------------------------------
// General purpose quadratic interpolation util function
// Treats boundary conditions
float patchouli_quad_interp(patchouli_quad_interp_t* q){
    int n = q->n;
    q->result_maxidx = 0;
    float maxval = -100.0f;
    for (int i=0; i<n; i++){
        if (q->data[i]>maxval){
            maxval = q->data[i];
            q->result_maxidx = i;
        }
    }
    int xl, xm, xr;
    if (q->result_maxidx==0) {
        q->boundary_status = PATCHOULI_QUAD_INTERP_LEFT;
        xl = 0;
        xm = 0;
        xr = 1;
    } else if (q->result_maxidx==n-1){
        q->boundary_status = PATCHOULI_QUAD_INTERP_RIGHT;
        xl = n-2;
        xm = n-1;
        xr = n-1;
    } else {
        q->boundary_status = PATCHOULI_QUAD_INTERP_NORMAL;
        xl = q->result_maxidx-1;
        xm = q->result_maxidx;
        xr = q->result_maxidx+1;
    }
    float yl = q->data[xl];
    float ym = q->data[xm];
    float yr = q->data[xr];

    // Quadratic interpolation
    q->result_interp = (float)xl+0.5f*(3*yl-4*ym+yr)/(yl-2*ym+yr);
    if (q->boundary_status==PATCHOULI_QUAD_INTERP_LEFT)  q->result_interp -= 0.0f;
    if (q->boundary_status==PATCHOULI_QUAD_INTERP_RIGHT) q->result_interp -= 1.0f;

    // Map the result to the range [low, high]
    q->result_mapped = q->low + (q->high-q->low)*q->result_interp/(n-1);
    return q->result_mapped;
}

