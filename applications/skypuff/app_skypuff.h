/*
	Copyright 2020 Kirill Kostiuchenko	<kisel2626@gmail.com>

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef APP_SKYPUFF_H_
#define APP_SKYPUFF_H_

#include <stdint.h>
#include "datatypes.h"

/* --- This is common file for MCU and Skypuff UI apps --- */

// Do not forget to add next settings version command on skypuff_config struct update
// Skypuff commands with COMM_CUSTOM_APP_DATA
typedef enum
{
	SK_COMM_SETTINGS_V1,		   // Receive settings on get_conf
	SK_COMM_SETTINGS_LORA,		   // Propagate new LoRa settings between remote control units
	SK_COMM_POWER_STATS,		   // Send power stats
	SK_COMM_TEMP_STATS,			   // Send power and temp stats
	SK_COMM_FAULT,				   // Send Fault code to UI
	SK_COMM_PULLING_TOO_HIGH,	   // Messages to UI
	SK_COMM_OUT_OF_LIMITS,		   // Will contain entire error message without zero byte
	SK_COMM_UNWINDED_TO_OPPOSITE,  // Many one byte acknowledgments
	SK_COMM_MSG,				   // Just status message (used for debug sometimes)
	SK_COMM_UNWINDED_FROM_SLOWING, // Information about leaving braking + slowing zone
	SK_COMM_DETECTING_MOTION,	   // Information in Pre Pull that we are wating for movement
	SK_COMM_TOO_SLOW_SPEED_UP,     // Information in constant speed modes, that acceleration is too slow
	SK_COMM_ZERO_IS_SET,           // Information that zero is set
	SK_COMM_FORCE_IS_SET,          // Information that new nominal pull force is set
	SK_COMM_SETTINGS_APPLIED,      // Information that new settings applied
	SK_COMM_GUILLOTINE,            // Cut the rope command
} skypuff_custom_app_data_command;

// Winch FSM
typedef enum
{
	UNINITIALIZED,			   // Released motor until some valid configuration set
	BRAKING,				   // Braking zone near zero. Applies small amount of brake to the motor when external torque applied.
	BRAKING_EXTENSION,		   // Next to braking zone to prevent automatic transition to UNWINDING. Used for passive winches.
	SLOWING,				   // Next to braking zone to decrease speed and make transition to SLOW mode.
	SLOW,					   // Constant slow speed in direction to zero.
	UNWINDING,				   // Low force rope tension during step away. Also required for transition to PULL states.
	REWINDING,				   // Fast rope winding to zero with more force then UNWINDING.
	PRE_PULL,				   // Pull the pilot while stays on the takeoff
	TAKEOFF_PULL,			   // Takeoff pull
	PULL,					   // Nominal pull
	FAST_PULL,				   // Fast pull
	MANUAL_BRAKING,			   // Any position braking mode, caused by operator or communication timeout
	MANUAL_SLOW_SPEED_UP,	   // Speed up until manual constant speed reached with positive current
	MANUAL_SLOW,			   // Constant speed mode with positive current
	MANUAL_SLOW_BACK_SPEED_UP, // Speed up until manual constant speed with negative current
	MANUAL_SLOW_BACK,		   // Constant speed mode with negative current
	MANUAL_DEBUG_SMOOTH,	   // Debug smooth motor movements with 'smooth' terminal commands
	DISCONNECTED,			   // UI only state
} skypuff_state;

/*
	Smooth Motor Control

	This program respects pilot and does not pull or release him too rough.

	Smooth motor code uses config.amps_per_sec as force changing speed and
	smooth_max_step_delay as maximum step delay.

	Always brake instantly if braking zone.

	Functions started with 'smooth_' prefix control the motor via VESC mc_interface
	API. Skypuff FSM control the motor via 'smooth_' API.
*/
typedef enum
{
	MOTOR_RELEASED,
	MOTOR_CURRENT,
	MOTOR_BRAKING,
	MOTOR_SPEED,
} smooth_motor_mode;

// UI side use this too
inline const char *state_str(const skypuff_state s)
{
	switch (s)
	{
	case UNINITIALIZED:
		return "UNINITIALIZED";
	case BRAKING:
		return "BRAKING";
	case BRAKING_EXTENSION:
		return "BRAKING_EXTENSION";
	case MANUAL_BRAKING:
		return "MANUAL_BRAKING";
	case MANUAL_SLOW_SPEED_UP:
		return "MANUAL_SLOW_SPEED_UP";
	case MANUAL_SLOW:
		return "MANUAL_SLOW";
	case MANUAL_SLOW_BACK_SPEED_UP:
		return "MANUAL_SLOW_BACK_SPEED_UP";
	case MANUAL_SLOW_BACK:
		return "MANUAL_SLOW_BACK";
	case UNWINDING:
		return "UNWINDING";
	case REWINDING:
		return "REWINDING";
	case SLOWING:
		return "SLOWING";
	case SLOW:
		return "SLOW";
	case PRE_PULL:
		return "PRE_PULL";
	case TAKEOFF_PULL:
		return "TAKEOFF_PULL";
	case PULL:
		return "PULL";
	case FAST_PULL:
		return "FAST_PULL";
	case MANUAL_DEBUG_SMOOTH:
		return "MANUAL_DEBUG_SMOOTH";
	case DISCONNECTED:
		return "DISCONNECTED";
	default:
		return "UNKNOWN";
	}
}

inline const char *sk_command_str(const skypuff_custom_app_data_command c)
{
	switch (c)
	{
	case SK_COMM_SETTINGS_V1:
		return "SK_COMM_SETTINGS_V1";
	case SK_COMM_SETTINGS_LORA:
		return "SK_COMM_SETTINGS_LORA";
	case SK_COMM_POWER_STATS:
		return "SK_COMM_POWER_STATS";
	case SK_COMM_TEMP_STATS:
		return "SK_COMM_TEMP_STATS";
	case SK_COMM_FAULT:
		return "SK_COMM_FAULT";
	case SK_COMM_PULLING_TOO_HIGH:
		return "SK_COMM_PULLING_TOO_HIGH";
	case SK_COMM_OUT_OF_LIMITS:
		return "SK_COMM_OUT_OF_LIMITS";
	case SK_COMM_UNWINDED_TO_OPPOSITE:
		return "SK_COMM_UNWINDED_TO_OPPOSITE";
	case SK_COMM_UNWINDED_FROM_SLOWING:
		return "SK_COMM_UNWINDED_FROM_SLOWING";
	case SK_COMM_DETECTING_MOTION:
		return "SK_COMM_DETECTING_MOTION";
	case SK_COMM_TOO_SLOW_SPEED_UP:
		return "SK_COMM_TOO_SLOW_SPEED_UP";
	case SK_COMM_ZERO_IS_SET:
		return "SK_COMM_ZERO_IS_SET";
	case SK_COMM_FORCE_IS_SET:
		return "SK_COMM_FORCE_IS_SET";
	case SK_COMM_SETTINGS_APPLIED:
		return "SK_COMM_SETTINGS_APPLIED";
	case SK_COMM_MSG:
		return "SK_COMM_MSG";
	default:
		return "SK_COMM_UNKNOWN";
	}
}

inline static const char *motor_mode_str(smooth_motor_mode m)
{
	switch (m)
	{
	case MOTOR_RELEASED:
		return "MOTOR_RELEASED";
	case MOTOR_BRAKING:
		return "MOTOR_BRAKING";
	case MOTOR_CURRENT:
		return "MOTOR_CURRENT";
	case MOTOR_SPEED:
		return "MOTOR_SPEED";
	default:
		return "MOTOR_UNKNOWN";
	}
}

#endif