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

#include "linea_math.h"
#include <string.h>
#include <stdlib.h>

// Mathematical functions - mostly wrappers around standard math library
float linea_sin(float x) { return sinf(x); }
float linea_cos(float x) { return cosf(x); }
float linea_tan(float x) { return tanf(x); }
float linea_asin(float x) { return asinf(x); }
float linea_acos(float x) { return acosf(x); }
float linea_atan(float x) { return atanf(x); }
float linea_atan2(float y, float x) { return atan2f(y, x); }
float linea_sqrt(float x) { return sqrtf(x); }
float linea_pow(float x, float y) { return powf(x, y); }
float linea_exp(float x) { return expf(x); }
float linea_log(float x) { return logf(x); }
float linea_log10(float x) { return log10f(x); }
float linea_abs(float x) { return fabsf(x); }
float linea_floor(float x) { return floorf(x); }
float linea_ceil(float x) { return ceilf(x); }
float linea_round(float x) { return roundf(x); }
float linea_fmod(float x, float y) { return fmodf(x, y); }

// Statistical functions
float linea_mean(float* data, uint16_t length) {
    if (length == 0) return 0.0f;
    
    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum / length;
}

float linea_variance(float* data, uint16_t length) {
    if (length <= 1) return 0.0f;
    
    float mean_val = linea_mean(data, length);
    float sum_sq = 0.0f;
    
    for (uint16_t i = 0; i < length; i++) {
        float diff = data[i] - mean_val;
        sum_sq += diff * diff;
    }
    
    return sum_sq / (length - 1);
}

float linea_std(float* data, uint16_t length) {
    return sqrtf(linea_variance(data, length));
}

float linea_median(float* data, uint16_t length) {
    if (length == 0) return 0.0f;
    if (length == 1) return data[0];
    
    // Create a copy to sort
    float* sorted = malloc(length * sizeof(float));
    if (!sorted) return 0.0f;
    
    memcpy(sorted, data, length * sizeof(float));
    
    // Simple bubble sort (can be optimized for larger arrays)
    for (uint16_t i = 0; i < length - 1; i++) {
        for (uint16_t j = 0; j < length - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    float result;
    if (length % 2 == 0) {
        result = (sorted[length/2 - 1] + sorted[length/2]) / 2.0f;
    } else {
        result = sorted[length/2];
    }
    
    free(sorted);
    return result;
}

float linea_min(float* data, uint16_t length) {
    if (length == 0) return 0.0f;
    
    float min_val = data[0];
    for (uint16_t i = 1; i < length; i++) {
        if (data[i] < min_val) {
            min_val = data[i];
        }
    }
    return min_val;
}

float linea_max(float* data, uint16_t length) {
    if (length == 0) return 0.0f;
    
    float max_val = data[0];
    for (uint16_t i = 1; i < length; i++) {
        if (data[i] > max_val) {
            max_val = data[i];
        }
    }
    return max_val;
}

// Filtering functions
float linea_lowpass_filter(float input, float* state, float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    *state = alpha * input + (1.0f - alpha) * (*state);
    return *state;
}

float linea_highpass_filter(float input, float* state, float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    float output = input - *state;
    *state = alpha * output + (1.0f - alpha) * (*state);
    return output;
}

float linea_moving_average(float input, float* buffer, uint16_t* index, uint16_t length) {
    if (length == 0) return input;
    
    buffer[*index] = input;
    *index = (*index + 1) % length;
    
    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum += buffer[i];
    }
    
    return sum / length;
}

// Utility functions
float linea_clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float linea_map(float value, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min) return out_min;
    
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float linea_constrain(float value, float min, float max) {
    return linea_clamp(value, min, max);
}

bool linea_is_nan(float value) {
    return value != value; // NaN is the only value that is not equal to itself
}

bool linea_is_inf(float value) {
    return value > 1e38f || value < -1e38f; // Simple infinity check for 32-bit floats
}
