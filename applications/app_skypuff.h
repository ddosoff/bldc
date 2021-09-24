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

/* --- This is common file for MCU and UI SkyPUFF apps --- */

// Do not forget to add next settings version command on skypuff_config struct update
// Skypuff commands with COMM_CUSTOM_APP_DATA
typedef enum
{
	SK_COMM_SETTINGS_V1,		  // Receive settings on get_conf
	SK_COMM_SETTINGS_LORA,		  // Propagate new LoRa settings between remote control units
	SK_COMM_ALIVE_POWER_STATS,	  // Get alive timeout from UI and send back power stats
	SK_COMM_ALIVE_TEMP_STATS,	  // Get alive timeout from UI and send back power and temp stats
	SK_COMM_FAULT,				  // Send Fault code to UI
	SK_COMM_STATE,				  // Send/receive new state
	SK_COMM_PULLING_TOO_HIGH,	  // Messages to UI
	SK_COMM_OUT_OF_LIMITS,		  // Will contain entire error message without zero byte
	SK_COMM_UNWINDED_TO_OPPOSITE, // Many one byte acknowledgments
	SK_COMM_MSG,				  // Just status message (used for debug sometimes)
	SK_COMM_UNWINDED_FROM_SLOWING,
	SK_COMM_DETECTING_MOTION,
	SK_COMM_TOO_SLOW_SPEED_UP,
	SK_COMM_ZERO_IS_SET,
	SK_COMM_FORCE_IS_SET,
	SK_COMM_SETTINGS_APPLIED,
	SK_COMM_GUILLOTINE,
} skypuff_custom_app_data_command;

// Winch FSM
typedef enum
{
	UNINITIALIZED,			   // Released motor until some valid configuration set
	BRAKING,				   // Braking zone near take off
	BRAKING_EXTENSION,		   // Braking after braking zone. Set long to disable automatic transition to UNWINDING. Manual possible.
	SLOWING,				   // Next to braking zone to slow down the motor
	SLOW,					   // Constant speed in direction to zero
	UNWINDING,				   // Low force rope tension during step up or unwinder mode
	REWINDING,				   // Fast rope winding to zero for unwinder mode
	PRE_PULL,				   // Pull the pilot while stays on the takeoff
	TAKEOFF_PULL,			   // Takeoff pull
	PULL,					   // Nominal pull
	FAST_PULL,				   // Fast pull
	MANUAL_BRAKING,			   // Any position braking caused by operator or communication timeout
	MANUAL_SLOW_SPEED_UP,	   // Speed up until manual constant speed with positive current
	MANUAL_SLOW,			   // Constant speed mode with positive current
	MANUAL_SLOW_BACK_SPEED_UP, // Speed up until manual constant speed in negative current
	MANUAL_SLOW_BACK,		   // Constant speed mode with negative current
	MANUAL_DEBUG_SMOOTH,       // Debug smooth motor movements with 'smooth' terminal commands
	DISCONNECTED,              // UI only state
} skypuff_state;

// Winch settings
typedef struct
{
	float amps_per_kg;					// Winch drive force coefficient
	int pull_applying_period;			// Milliseconds to apply pull force, amps_per_sec will be calculated from this delay
	int rope_length;					// Winch rope length in tachometer steps (used by interface only)
	int braking_length;					// Tachometer range of braking zone
	int braking_extension_length;		// Increase braking_length for passive winches when car drive 150m from takeoff
	int slowing_length;					// Range after braking zone to slow down motor when unwinding to zero
	float slow_erpm;					// Constant erpm in direction to zero
	int rewinding_trigger_length;		// Switch to fast rewinding state after going back this length
	int unwinding_trigger_length;		// Back to unwinding from rewinding if this range unwinded again
	float pull_current;					// Winch normal pull force, usually pilot weight
	float pre_pull_k;					// pre_pull_k * pull_current = pull current when pilots stays on the ground
	float takeoff_pull_k;				// takeoff_pull_k * pull_current = pull current during takeoff
	float fast_pull_k;					// fast_pull_k * pull_current = pull current to get altitude fast
	int takeoff_trigger_length;			// Minimal PRE_PULL movement for transition to TAKEOFF_PULL
	int pre_pull_timeout;				// Timeout before saving position after PRE_PULL
	int takeoff_period;					// Time of TAKEOFF_PULL and then switch to normal PULL
	float brake_current;				// Braking zone force, could be set high to charge battery driving away
	float slowing_current;				// Set zero to release motor when slowing or positive value to brake
	float manual_brake_current;			// Manual braking force
	float unwinding_current;			// Unwinding force
	float unwinding_strong_current;		// Due to motor cogging we need more powerfull unwinding near zero speed
	float unwinding_strong_erpm;		// Enable strong unwinding if unwinding speed is less then this value, set negative to enable on low speed unwinding
	float rewinding_current;			// Rewinding force
	float slow_max_current;				// Max force for constant slow speed
	float manual_slow_max_current;		// Max force for MANUAL_SLOW and MANUAL_SLOW_BACK
	float manual_slow_speed_up_current; // Speed up current for manual constant speed states
	float manual_slow_erpm;				// Constant speed for manual rotation
	float max_speed_ms;					// Speed scale limit (only affects the interface)

	// Antisex dampering
	float antisex_starting_integral;	// Start antisex algorihtm if acceleration integral is higher then this
	float antisex_reduce_amps;			// Reduce motor amps to this value on deceleration
	int antisex_reduce_steps;			// Maximum number of steps to increase deceleration
	float antisex_reduce_amps_per_step; // Deceleration increase amps per step
	float antisex_unwinding_gain;		// Coefficient to multiply current when unwinding
	float antisex_gain_speed;			// Unwinding speed when antisex_unwinding_gain applied
} skypuff_config;

// Drive settings part of mc_configuration
typedef struct
{
	int motor_poles;
	float wheel_diameter;
	float gear_ratio;

	// vesc_tool is not necessary to change battery limits
	BATTERY_TYPE battery_type;
	int battery_cells;

	/* This mc_configuration values will be updated according battery_type and cells number

    float l_battery_cut_start;
    float l_battery_cut_end;
    float l_max_vin;
    */
} skypuff_drive;

/*
	Smooth Motor Control

	Respect pilot and do not pull or release him too sharply.
	Unwinding current is minimal step of applying pull or brake.

	Use config.amps_per_sec as force changing speed and
	smooth_max_step_delay as maximum step delay.

	In case of different current and target modes, 
	switch instantly if force is less then unwinding 
	and smoothly decrease/increase until unwinding if above.

	Always brake instantly if braking zone.
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
	case SK_COMM_ALIVE_POWER_STATS:
		return "SK_COMM_ALIVE_POWER_STATS";
	case SK_COMM_ALIVE_TEMP_STATS:
		return "SK_COMM_ALIVE_TEMP_STATS";
	case SK_COMM_FAULT:
		return "SK_COMM_FAULT";
	case SK_COMM_STATE:
		return "SK_COMM_STATE";
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
