/*
* Project Linea
* Copyright (C) 2024 Anhang Li (thelithcore@gmail.com)
* Adapted for STM32WB55 by Nathan
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

#ifndef __LINEA_MATH_H
#define __LINEA_MATH_H

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

// Mathematical constants
#define LINEA_PI 3.14159265359f
#define LINEA_2PI 6.28318530718f
#define LINEA_PI_2 1.57079632679f
#define LINEA_PI_4 0.78539816339f

// Mathematical functions
float linea_sin(float x);
float linea_cos(float x);
float linea_tan(float x);
float linea_asin(float x);
float linea_acos(float x);
float linea_atan(float x);
float linea_atan2(float y, float x);
float linea_sqrt(float x);
float linea_pow(float x, float y);
float linea_exp(float x);
float linea_log(float x);
float linea_log10(float x);
float linea_abs(float x);
float linea_floor(float x);
float linea_ceil(float x);
float linea_round(float x);
float linea_fmod(float x, float y);

// Statistical functions
float linea_mean(float* data, uint16_t length);
float linea_variance(float* data, uint16_t length);
float linea_std(float* data, uint16_t length);
float linea_median(float* data, uint16_t length);
float linea_min(float* data, uint16_t length);
float linea_max(float* data, uint16_t length);

// Filtering functions
float linea_lowpass_filter(float input, float* state, float alpha);
float linea_highpass_filter(float input, float* state, float alpha);
float linea_moving_average(float input, float* buffer, uint16_t* index, uint16_t length);

// Utility functions
float linea_clamp(float value, float min, float max);
float linea_map(float value, float in_min, float in_max, float out_min, float out_max);
float linea_constrain(float value, float min, float max);
bool linea_is_nan(float value);
bool linea_is_inf(float value);

#endif /* __LINEA_MATH_H */
