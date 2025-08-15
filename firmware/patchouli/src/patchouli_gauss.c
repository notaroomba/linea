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

#include "patchouli_gauss.h"

void patchouli_gauss_init_kernel(patchouli_gauss_t* k, int window_size){
    k->window_size = window_size;
    float32_t A_f32 [3*window_size];
    float32_t AT_f32[3*window_size];
    float32_t ATMA_f32[3*3];
    float32_t ATMAI_f32[3*3];
    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 AT;
    arm_matrix_instance_f32 ATMA;
    arm_matrix_instance_f32 ATMAI;
    k->kernel_f32 = (float32_t*)malloc(3*window_size*sizeof(float32_t));
    k->c_f32 = (float32_t*)malloc(3*sizeof(float32_t));
    k->b_f32 = (float32_t*)malloc(window_size*sizeof(float32_t));
    arm_mat_init_f32(&A, window_size,3 ,  A_f32 );
    arm_mat_init_f32(&AT, 3, window_size, AT_f32);
    arm_mat_init_f32(&ATMA, 3, 3, ATMA_f32);
    arm_mat_init_f32(&ATMAI, 3, 3, ATMAI_f32);
    arm_mat_init_f32(&k->kernel_mat, 3, window_size, k->kernel_f32);
    arm_mat_init_f32(&k->b_mat, window_size,1, k->b_f32);
    arm_mat_init_f32(&k->c_mat, 3,1, k->c_f32);
    for (int x=0; x<window_size;x++) {
        A_f32[x*3+0] = 1;
        A_f32[x*3+1] = x+1;
        A_f32[x*3+2] = (x+1)*(x+1);
    }
    arm_mat_trans_f32(&A, &AT);
    arm_mat_mult_f32(&AT, &A, &ATMA);
    arm_mat_inverse_f32(&ATMA, &ATMAI);
    arm_mat_mult_f32(&ATMAI, &AT, &k->kernel_mat);
}

float* patchouli_gauss_apply(patchouli_gauss_t* k, int16_t* arr, float* dst) {
    for (int x=0; x<k->window_size;x++) {
        k->b_f32[x] = (float)arr[x];
    }
    arm_mat_mult_f32(&k->kernel_mat, &k->b_mat, &k->c_mat);
    memcpy(dst, k->c_f32, 3*sizeof(float));
    return dst;
}