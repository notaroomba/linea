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

#ifndef _PATCHOULI_GAUSS_H
#define _PATCHOULI_GAUSS_H

#include "arm_math.h"
typedef struct {
    int                     window_size;
    float32_t*              kernel_f32;
    arm_matrix_instance_f32 kernel_mat;
    float32_t*              c_f32;
    arm_matrix_instance_f32 c_mat;
    float32_t*              b_f32;
    arm_matrix_instance_f32 b_mat;
} patchouli_gauss_t;

void   patchouli_gauss_init_kernel(patchouli_gauss_t* k, int window_size);
float* patchouli_gauss_apply(patchouli_gauss_t* k, int16_t* arr, float* dst);



#endif // _PATCHOULI_GAUSS_H