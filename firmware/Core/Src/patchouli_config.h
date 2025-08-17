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

#ifndef __PATCHOULI_CONFIG_H
#define __PATCHOULI_CONFIG_H

// -- Define the type of pen
// #define PATCHOULI_PEN_PW550
#define PATCHOULI_PEN_PW100
// #define PATCHOULI_PEN_LP1100K

// -- Define the type of PCB
// #define PATCHOULI_PCB_DISCRETE_SST
#define PATCHOULI_PCB_GLIDER_ADDON_V1

// -- Define the Log level
// Higher level may lead to performance issues
#define PATCHOULI_DEBUG_LEVEL 0U

// Subtracting base level when sampling
#define PATCHOULI_CDS

// Number of ADC samples (pop-filter)
#define PATCHOULI_ADC_NSAMPLE (3u)

#endif /* __PATCHOULI_CONFIG_H */
