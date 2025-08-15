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
// Database of Passive Pens

#ifndef PATCHOULI_PASSIVE_PEN_H
#define PATCHOULI_PASSIVE_PEN_H

// Automatic Parameters
#ifdef PATCHOULI_PEN_PW550
	#define PATCHOULI_TIM_F_MAX    (600.0E3)
	#define PATCHOULI_TIM_F_MIN    (530.0E3)
	#define PATCHOULI_TIM_F_STEP   (10e3)
	#define PATCHOULI_PDET_THRES   (700u)
#endif
#ifdef PATCHOULI_PEN_PW100
	#define PATCHOULI_TIM_F_MAX    (555.0E3)
	#define PATCHOULI_TIM_F_MIN    (475.0E3)
	#define PATCHOULI_TIM_F_STEP   (10e3)
	#define PATCHOULI_PDET_THRES   (150u)
#endif
#define PATCHOULI_TIM_F_CENTER ((PATCHOULI_TIM_F_MAX + PATCHOULI_TIM_F_MIN) / 2.0f)
#define PATCHOULI_TIM_N_STEP   ((int)((PATCHOULI_TIM_F_MAX-PATCHOULI_TIM_F_MIN)/PATCHOULI_TIM_F_STEP+1))

#endif // PATCHOULI_PASSIVE_PEN_H
