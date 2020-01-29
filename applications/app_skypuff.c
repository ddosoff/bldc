/*
	Copyright 2019 Kirill Kostiuchenko	<kisel2626@gmail.com>

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

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "comm_can.h"
#include "commands.h"
#include "encoder.h"
#include "hw.h"
#include "mc_interface.h"
#include "terminal.h"
#include "timeout.h"
#include "utils.h"
#include "packet.h"
#include "buffer.h"

#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Uncomment to enable Smooth Motor Control debugging terminal commands
//#define DEBUG_SMOOTH_MOTOR
#include "app_skypuff.h"

/*
	This application turns VESC into paragliding winch controller.
	Some progress here: https://www.youtube.com/watch?v=KoNegc4SzxY

	To play with model winch, send terminal commands 'example_conf' 
	and 'alive 3000000' to set long communication timeout.

	Skypuff can't use VESC timeout because of smooth pull release mechanism.

	Terminal output format is unified to simplify UI side parsing.

	Each print consist of current state and comma delimeted messages.

	State: type message, type message, ...

	Types:
	  'pos': current position
	  'pull': pulling or braking force
	  'speed': current speed
	  '--': warning text until end of line 

	Examples:
		UNITIALIZED: -- CONFIGURATION IS OUT OF LIMITS -- Braking current 0.0A is out of limits (0.5, 20.0)
		BRAKING: pos 0.00m (0 steps), speed: -0.0ms (-0 ERPM), braking 1.0Kg (7.0A)

	On double '--' UI will show message dialog with first payload as title and second as text.
*/

const int short_print_delay = 500; // 0.5s, measured in control loop counts
const int long_print_delay = 3000;
const int temps_print_delay = 15000;
const int smooth_max_step_delay = 50;

const char *limits_wrn = "-- CONFIGURATION IS OUT OF LIMITS --";

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void terminal_set_zero(int argc, const char **argv);
static void terminal_print_conf(int argc, const char **argv);
static void terminal_get_conf(int argc, const char **argv); // Send serialized conf with COMM_CUSTOM_APP_DATA
static void terminal_set_example_conf(int argc, const char **argv);
static void terminal_alive(int argc, const char **argv);
static void terminal_set_state(int argc, const char **argv);
static void terminal_set_pull_force(int argc, const char **argv);
#ifdef DEBUG_SMOOTH_MOTOR
static void terminal_smooth(int argc, const char **argv);
#endif

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static const volatile mc_configuration *mc_conf;
static int timeout_reset_interval;		 // System app timeout divided by 2
static int loop_step;					 // Main control loop counter
static int prev_abs_tac;				 // Detect movements
static float prev_erpm;					 // Detect change in direction of rotation
static int prev_print;					 // Loop counter value of the last state print
static int prev_temps_print;			 // Loop counter value of the last temps print
static int prev_printed_tac;			 // Do not print the same position
static float prev_printed_fets_temp;	 // Do not print close temps
static float prev_printed_motor_temp;	// Do not print close temps
static float prev_printed_bat_temp;		 // Do not print close temps
static float prev_printed_bat_v;		 // Do not print small changes
static float prev_printed_wh_out;		 // Do not print small changes
static float prev_printed_wh_in;		 // Do not print small changes
static mc_fault_code prev_printed_fault; // Do not print the same fault many times
static float v_in_filtered;				 // Average for v_in
static float erpm_filtered;				 // For speed up states
static int alive_until;					 // In good communication we trust until (i < alive_until)
static int state_start_time;			 // Count the duration of state
static float terminal_pull_kg;			 // Pulling force to set
static volatile int alive_inc;			 // Communication timeout increment from terminal thread

static volatile skypuff_state state; // Readable from commands threads too
static skypuff_config config;
static skypuff_config set_config; // Updates from terminal thread
static skypuff_drive set_drive;   // Update mc_configuration additional drive settings

// Terminal thread commands
typedef enum
{
	DO_NOTHING,
	SET_ZERO,
	SET_MANUAL_BRAKING,
	SET_BRAKING_EXTENSION,
	SET_UNWINDING,
	SET_MANUAL_SLOW,
	SET_MANUAL_SLOW_BACK,
	SET_PULL_FORCE,
	SET_PRE_PULL,
	SET_TAKEOFF_PULL,
	SET_PULL,
	SET_FAST_PULL,
	PRINT_CONF,
	SET_CONF,
	SEND_CONF,
#ifdef DEBUG_SMOOTH_MOTOR
	SET_SMOOTH,
#endif
} skypuff_terminal_command;

static volatile skypuff_terminal_command terminal_command;

// This struct is not for serialization
typedef struct
{
	smooth_motor_mode mode;
	union {
		float current;
		float erpm;
	} param;
} smooth_motor_state;

static smooth_motor_state current_motor_state;
static smooth_motor_state target_motor_state;
#ifdef DEBUG_SMOOTH_MOTOR
static smooth_motor_state terminal_motor_state;
#endif
static int next_smooth_motor_adjustment; // Loop count for next motor adjustment
static int prev_smooth_motor_adjustment; // Loop count of previous motor adjustment
static float amps_per_sec;				 // Speed to change force during smooth motor adjustments, calculated

/*
	Skypuff relies on system drive settings to calculate rope meters
	This struct is necessary for limits only,
	actual values stored in mc_configuration struct.
*/
static const skypuff_drive min_drive_limits = {
	.motor_poles = 2,
	.wheel_diameter = 0.01, // Meters
	.gear_ratio = 0.05,		// Motor turns per 1 spool turn
};

static const skypuff_drive max_drive_limits = {
	.motor_poles = 60,
	.wheel_diameter = 2,
	.gear_ratio = 20,
};

// Do we actually need meters and kilograms here?
// Check them on UI side
static const skypuff_config min_config = {
	.amps_per_kg = 0.5,
	.pull_applying_period = 1, // 0.001 secs
	.rope_length = 5,
	.braking_length = 5,
	.braking_extension_length = 3, // To create trigger length after unwinding
	.slowing_length = 3,
	.slow_erpm = 100,
	.rewinding_trigger_length = 10,
	.unwinding_trigger_length = 3,
	.pull_current = 1,
	.pre_pull_k = 0.1,
	.takeoff_pull_k = 0.3,
	.fast_pull_k = 1.05,
	.takeoff_trigger_length = 3,
	.pre_pull_timeout = 100, // 0.1 secs
	.takeoff_period = 100,
	.brake_current = 1,
	.slowing_current = 0,
	.manual_brake_current = 1,
	.unwinding_current = 1,
	.rewinding_current = 1,
	.slow_max_current = 1,
	.manual_slow_max_current = 1,
	.manual_slow_speed_up_current = 1,
	.manual_slow_erpm = 100,
};

static const skypuff_config max_config = {
	.amps_per_kg = 30,
	.pull_applying_period = 10000, // 10 secs
	.rope_length = 5000 * 120,	 // 120 - maximum motor poles * 3
	.braking_length = 100 * 120,
	.braking_extension_length = 5000 * 120,
	.slowing_length = 100 * 120,
	.slow_erpm = 30000,
	.rewinding_trigger_length = 50 * 120,
	.unwinding_trigger_length = 10 * 120,
	.pull_current = 600, // Believe in gliders
	.pre_pull_k = 0.5,
	.takeoff_pull_k = 0.8,
	.fast_pull_k = 1.5,
	.takeoff_trigger_length = 5000 * 120,
	.pre_pull_timeout = 5000,   // 5 secs
	.takeoff_period = 60000,	// 1 min
	.brake_current = 500,		// Charge battery mode possible
	.slowing_current = 30,		// Do not brake hardly on high unwinding speeds
	.manual_brake_current = 30, // Do not kill pilot in passive winch mode
	.unwinding_current = 50,
	.rewinding_current = 100,
	.slow_max_current = 50,
	.manual_slow_max_current = 50,
	.manual_slow_speed_up_current = 50,
	.manual_slow_erpm = 40000,
};

// Convert units functions
inline static float meters_per_rev(void)
{
	return mc_conf->si_wheel_diameter / mc_conf->si_gear_ratio * M_PI;
}

inline static float steps_per_rev(void)
{
	return mc_conf->si_motor_poles * 3;
}

inline static int meters_to_tac_steps(float meters)
{
	return round(meters / meters_per_rev() * steps_per_rev());
}

inline static float tac_steps_to_meters(int steps)
{
	return (float)steps / steps_per_rev() * meters_per_rev();
}

inline static float ms_to_erpm(float ms)
{
	float rps = ms / meters_per_rev();
	float rpm = rps * 60;

	return rpm * (mc_conf->si_motor_poles / 2);
}

inline static float erpm_to_ms(float erpm)
{
	float erps = erpm / 60;
	float rps = erps / (mc_conf->si_motor_poles / 2);

	return rps * meters_per_rev();
}

// Smooth motor functions
#ifdef DEBUG_SMOOTH_MOTOR
inline static void snprintf_motor_state(const smooth_motor_state *s, char *buf, int buf_len)
{
	switch (s->mode)
	{
	case MOTOR_CURRENT:
	case MOTOR_BRAKING:
		// Print precisely to simplify debuggging
		snprintf(buf, buf_len, "%s (%.3fkg / %.2fA)", motor_mode_str(s->mode),
				 (double)(s->param.current / config.amps_per_kg), (double)s->param.current);
		break;
	case MOTOR_SPEED:
		snprintf(buf, buf_len, "%s (%.1fms / %.0f ERPM)", motor_mode_str(s->mode),
				 (double)erpm_to_ms(s->param.erpm), (double)s->param.erpm);
		break;
	default:
		snprintf(buf, buf_len, "%s", motor_mode_str(s->mode));
		break;
	}
}
#endif

inline static void smooth_motor_instant_brake(void)
{
#ifdef DEBUG_SMOOTH_MOTOR
	const int buf_len = 100;
	char buf[buf_len];
	snprintf_motor_state(&target_motor_state, buf, buf_len);
	commands_printf("%s: loop %d, -- set brake instantly %s", state_str(state), loop_step, buf);
#endif
	current_motor_state = target_motor_state;
	mc_interface_set_brake_current(target_motor_state.param.current);

	next_smooth_motor_adjustment = INT_MAX;
	prev_smooth_motor_adjustment = loop_step;
	return;
}

inline static void smooth_motor_instant_current(void)
{
#ifdef DEBUG_SMOOTH_MOTOR
	const int buf_len = 100;
	char buf[buf_len];
	snprintf_motor_state(&target_motor_state, buf, buf_len);
	commands_printf("%s: loop %d, -- set current instantly %s", state_str(state), loop_step, buf);
#endif

	current_motor_state = target_motor_state;

	mc_interface_set_current(target_motor_state.param.current);

	next_smooth_motor_adjustment = INT_MAX;
	prev_smooth_motor_adjustment = loop_step;
	return;
}

inline static int smooth_motor_prev_adjustment_delay(void)
{
	int prev_adjustment_delay = loop_step - prev_smooth_motor_adjustment;

	// Truncate to smooth_max_step_delay if higher
	if (prev_adjustment_delay > smooth_max_step_delay)
		prev_adjustment_delay = smooth_max_step_delay;

	return prev_adjustment_delay;
}

inline static void smooth_calculate_new_speed(void)
{
	amps_per_sec = config.pull_current / ((float)config.pull_applying_period / (float)1000.0);
}

// Do not complicate this calculation with jumps over unwinding boundaries
inline static int smooth_calc_next_delay(const float c1, const float c2)
{
	double secs_delay = fabs(c2 - c1) / (double)amps_per_sec;
	int millis_delay = (int)(secs_delay * (double)1000.0);

#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, smooth_calc_next_delay(%.5fA, %.5fA) secs_delay: %.5fs, millis: %d",
					state_str(state), loop_step, (double)c1, (double)c2, (double)secs_delay, millis_delay);
#endif

	// Cut maximum delay with smooth_max_step_delay
	millis_delay = smooth_max_step_delay < millis_delay ? smooth_max_step_delay : millis_delay;

	return millis_delay;
}

inline static bool smooth_is_between_unwinding(const float c)
{
	return c > -config.unwinding_current && c < config.unwinding_current;
}

inline static float smooth_signed_unwinding_by(float c)
{
	return c >= 0 ? config.unwinding_current : -config.unwinding_current;
}

// Returns new current and next adjustment delay
inline static float smooth_calc_step(const float c1,
									 const float c2,
									 const int prev_adjustment_delay,
									 int *next_delay)
{
	// Calculate current step respecting force changing speed and delay from previous force adjustment
	float step = amps_per_sec * (float)prev_adjustment_delay / (float)1000.0;

	float c;

	// Target is below last current?
	if (c2 < c1)
	{
		// Step down to target
		c = c1 - step;
		if (c <= c2)
			c = c2; // Do not jump over
	}
	else
	{
		// Step up to target
		c = c1 + step;
		if (c >= c2)
			c = c2; // Do not jump over
	}

	// Calculated current is less then unwinding (both signs)?
	if (smooth_is_between_unwinding(c))
	{
		// Target is weak?
		if (smooth_is_between_unwinding(c2))
			return c2; // Set target instanly
		else
			// Start adjustments from unwinding with correct sign
			c = smooth_signed_unwinding_by(c2);
	}

	*next_delay = smooth_calc_next_delay(c, c2);

	// Prevent extra adjustments
	if (*next_delay <= 1)
		*next_delay = 0; // Set target mode instantly

	return c;
}

// Process all posible motor mode transitions
inline static void smooth_motor_adjustment(const int cur_tac, const int abs_tac)
{
	int prev_adjustment_delay = smooth_motor_prev_adjustment_delay();

#ifdef DEBUG_SMOOTH_MOTOR
	const int buf_len = 100;
	char current_buf[buf_len], target_buf[buf_len];
	snprintf_motor_state(&current_motor_state, current_buf, buf_len);
	snprintf_motor_state(&target_motor_state, target_buf, buf_len);
	commands_printf("%s: loop %d, -- smooth_motor_adjustment(cur_tac %d), prev_delay %d, %s -> %s",
					state_str(state), loop_step, cur_tac, prev_adjustment_delay,
					current_buf, target_buf);
#endif

	float step_current;
	float signed_unwinding_current;
	int next_delay = 0; // Set target instantly by default

	switch (target_motor_state.mode)
	{
	case MOTOR_BRAKING:
		// Braking and slowing zone always instant
		if (abs_tac <= config.braking_length + config.slowing_length)
		{
#ifdef DEBUG_SMOOTH_MOTOR
			commands_printf("%s: loop %d, -- braking/slowing zone detected", state_str(state), loop_step);
#endif
			smooth_motor_instant_brake();
			return;
		}

		switch (current_motor_state.mode)
		{
		case MOTOR_BRAKING:
			step_current = smooth_calc_step(current_motor_state.param.current,
											target_motor_state.param.current,
											prev_adjustment_delay,
											&next_delay);

			// Finished?
			if (!next_delay)
			{
				smooth_motor_instant_brake();
				return;
			}

			mc_interface_set_brake_current(step_current);
			current_motor_state.param.current = step_current;

			prev_smooth_motor_adjustment = loop_step;
			next_smooth_motor_adjustment = loop_step + next_delay;
			break;
		case MOTOR_CURRENT:
			// Smoothly decrease current until unwinding current and then start braking
			signed_unwinding_current = smooth_signed_unwinding_by(current_motor_state.param.current);

			step_current = smooth_calc_step(current_motor_state.param.current,
											signed_unwinding_current,
											prev_adjustment_delay,
											&next_delay);

			// signed_unwinding_current could be reached on this adjustment?
			if (!next_delay)
			{
				current_motor_state.mode = MOTOR_BRAKING;
				current_motor_state.param.current = 0;

				smooth_motor_adjustment(cur_tac, abs_tac);
				return;
			}

			mc_interface_set_current(step_current);
			current_motor_state.param.current = step_current;

			prev_smooth_motor_adjustment = loop_step;
			next_smooth_motor_adjustment = loop_step + next_delay;

			break;
		default:
			// Process MOTOR_RELEASED and MOTOR_SPEED here
			// Just smoothly increase braking
			current_motor_state.mode = MOTOR_BRAKING;
			current_motor_state.param.current = 0;

			smooth_motor_adjustment(cur_tac, abs_tac);
			break;
		}

		break;
	case MOTOR_CURRENT: // Target state
		switch (current_motor_state.mode)
		{
		case MOTOR_CURRENT:
			step_current = smooth_calc_step(current_motor_state.param.current,
											target_motor_state.param.current,
											prev_adjustment_delay,
											&next_delay);

			// Finished?
			if (!next_delay)
			{
				smooth_motor_instant_current();
				return;
			}

			mc_interface_set_current(step_current);
			current_motor_state.param.current = step_current;

			prev_smooth_motor_adjustment = loop_step;
			next_smooth_motor_adjustment = loop_step + next_delay;
			break;
		default:
			// Process MOTOR_BRAKING, MOTOR_RELEASED and MOTOR_SPEED here
			// Just smoothly increase current
			current_motor_state.mode = MOTOR_CURRENT;
			current_motor_state.param.current = 0;

			smooth_motor_adjustment(cur_tac, abs_tac);
			break;
		}
		break;

	default:
		commands_printf("%s: -- Wrong smooth motor adjustment target mode: '%s'",
						state_str(state), motor_mode_str(target_motor_state.mode));
		next_smooth_motor_adjustment = INT_MAX;
		return;
	}
}

// Always instantly
inline static void smooth_motor_release(void)
{
#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, -- set instant MOTOR_RELEASED, smooth_motor_release()", state_str(state), loop_step);
#endif

	target_motor_state.mode = MOTOR_RELEASED;
	target_motor_state.param.current = 0;

	mc_interface_release_motor();

	current_motor_state = target_motor_state;
	next_smooth_motor_adjustment = INT_MAX;
	prev_smooth_motor_adjustment = loop_step;
}

// Always instantly
inline static void smooth_motor_speed(const float erpm)
{
#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, -- set instant MOTOR_SPEED smooth_motor_speed(%.0f ERPM)",
					state_str(state), loop_step, (double)erpm);
#endif
	target_motor_state.mode = MOTOR_SPEED;
	target_motor_state.param.erpm = erpm;

	mc_interface_set_pid_speed(erpm);

	current_motor_state = target_motor_state;
	next_smooth_motor_adjustment = INT_MAX;
	prev_smooth_motor_adjustment = loop_step;
}

inline static void smooth_motor_brake(const int cur_tac, const int abs_tac, const float current)
{
#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, -- smooth_motor_brake(cur_tac %d, %.2fA)",
					state_str(state), loop_step, cur_tac, (double)current);
#endif

	target_motor_state.mode = MOTOR_BRAKING;
	target_motor_state.param.current = current;

	smooth_motor_adjustment(cur_tac, abs_tac);
}

inline static void smooth_motor_current(const int cur_tac, const int abs_tac, const float current)
{
#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, -- smooth_motor_current(%.2fA)",
					state_str(state), loop_step, (double)current);
#endif

	target_motor_state.mode = MOTOR_CURRENT;
	target_motor_state.param.current = current;

	smooth_motor_adjustment(cur_tac, abs_tac);
}

// Helper functions to check limits
inline static bool is_int_out_of_limits(const char *name, const char *units,
										const int val, const int min, const int max)
{
	if (val >= min && val <= max)
		return false;

	commands_printf("%s: %s %s: %d %s is out of limits [%d, %d]",
					state_str(state), limits_wrn, name, val, units, min, max);
	return true;
}

inline static bool is_float_out_of_limits(const char *name, const char *units,
										  const float val, const float min, const float max)
{
	if (val >= min && val <= max)
		return false;

	commands_printf("%s: %s %s: %.5f %s is out of limits [%.5f, %.5f]",
					state_str(state), limits_wrn, name, (double)val, units, (double)min, (double)max);
	return true;
}

inline static bool is_pull_out_of_limits(const char *name,
										 const float amps, const float min, const float max)
{
	if (amps >= min && amps <= max)
		return false;

	commands_printf("%s: %s %s: %.1fA (%.2fKg) is out of limits [%.1fA (%.2fKg), %.1fA (%.2fKg)]",
					state_str(state), limits_wrn, name,
					(double)amps, (double)(amps / config.amps_per_kg),
					(double)min, (double)(min / config.amps_per_kg),
					(double)max, (double)(max / config.amps_per_kg));
	return true;
}

inline static bool is_distance_out_of_limits(const char *name,
											 const int steps, const int min, const int max)
{
	if (steps >= min && steps <= max)
		return false;

	commands_printf("%s: %s %s: %d steps (%.2f meters) is out of limits [%d (%.2fm), %d (%.2fm)]",
					state_str(state), limits_wrn, name,
					steps, (double)tac_steps_to_meters(steps),
					min, (double)tac_steps_to_meters(min),
					max, (double)tac_steps_to_meters(max));
	return true;
}

inline static bool is_speed_out_of_limits(const char *name,
										  const float erpm, const float min, const float max)
{
	if (erpm >= min && erpm <= max)
		return false;

	commands_printf("%s: %s %s: %.1f ERPM (%.1f m/s) is out of limits [%.1f (%.1f m/s), %.1f (%.1f m/s)]",
					state_str(state), limits_wrn, name,
					(double)erpm, (double)erpm_to_ms(erpm),
					(double)min, (double)erpm_to_ms(min),
					(double)max, (double)erpm_to_ms(max));
	return true;
}

static bool is_drive_config_out_of_limits(const skypuff_drive *drv)
{
	if (drv->motor_poles % 2)
	{
		commands_printf("%s: %s motor_poles: %dp must be odd",
						state_str(state), limits_wrn, drv->motor_poles);
		return true;
	}

	return is_int_out_of_limits("motor_poles", "p", drv->motor_poles,
								min_drive_limits.motor_poles, max_drive_limits.motor_poles) ||
		   is_float_out_of_limits("wheel_diameter", "mm", drv->wheel_diameter * (float)1000,
								  min_drive_limits.wheel_diameter * (float)1000.0,
								  max_drive_limits.wheel_diameter * (float)1000.0) ||
		   is_float_out_of_limits("gear_ratio", "turn(s)", drv->gear_ratio,
								  min_drive_limits.gear_ratio, max_drive_limits.gear_ratio);
}

static bool is_config_out_of_limits(const skypuff_config *conf)
{
	return is_float_out_of_limits("amps_per_kg", "KgA", conf->amps_per_kg,
								  min_config.amps_per_kg, max_config.amps_per_kg) ||
		   is_int_out_of_limits("pull_applying_period", "milliseconds", conf->pull_applying_period,
								min_config.pull_applying_period, max_config.pull_applying_period) ||
		   is_distance_out_of_limits("rope_length", conf->rope_length,
									 min_config.rope_length, max_config.rope_length) ||
		   is_distance_out_of_limits("braking_length", conf->braking_length,
									 min_config.braking_length, max_config.braking_length) ||
		   is_distance_out_of_limits("braking_extension_length", conf->braking_extension_length,
									 min_config.braking_extension_length, max_config.braking_extension_length) ||
		   is_distance_out_of_limits("slowing_length", conf->slowing_length,
									 min_config.slowing_length, max_config.slowing_length) ||
		   is_distance_out_of_limits("rewinding_trigger_length", conf->rewinding_trigger_length,
									 min_config.rewinding_trigger_length, max_config.rewinding_trigger_length) ||
		   is_distance_out_of_limits("unwinding_trigger_length", conf->unwinding_trigger_length,
									 min_config.unwinding_trigger_length, max_config.unwinding_trigger_length) ||
		   is_distance_out_of_limits("takeoff_trigger_length", conf->takeoff_trigger_length,
									 min_config.takeoff_trigger_length, max_config.takeoff_trigger_length) ||
		   is_speed_out_of_limits("slow_erpm", conf->slow_erpm,
								  min_config.slow_erpm, max_config.slow_erpm) ||
		   is_speed_out_of_limits("manual_slow_erpm", conf->manual_slow_erpm,
								  min_config.manual_slow_erpm, max_config.manual_slow_erpm) ||
		   is_pull_out_of_limits("pull_current", conf->pull_current,
								 min_config.pull_current, max_config.pull_current) ||
		   is_pull_out_of_limits("brake_current", conf->brake_current,
								 min_config.brake_current, max_config.brake_current) ||
		   is_pull_out_of_limits("slowing_current", conf->slowing_current,
								 min_config.slowing_current, max_config.slowing_current) ||
		   is_pull_out_of_limits("manual_brake_current", conf->manual_brake_current,
								 min_config.manual_brake_current, max_config.manual_brake_current) ||
		   is_pull_out_of_limits("unwinding_current", conf->unwinding_current,
								 min_config.unwinding_current, max_config.unwinding_current) ||
		   is_pull_out_of_limits("rewinding_current", conf->rewinding_current,
								 conf->unwinding_current, max_config.rewinding_current) ||
		   is_pull_out_of_limits("slow_max_current", conf->slow_max_current,
								 min_config.slow_max_current, max_config.slow_max_current) ||
		   is_pull_out_of_limits("manual_slow_max_current", conf->manual_slow_max_current,
								 min_config.manual_slow_max_current, max_config.manual_slow_max_current) ||
		   is_pull_out_of_limits("manual_slow_speed_up_current", conf->manual_slow_speed_up_current,
								 min_config.manual_slow_speed_up_current, max_config.manual_slow_speed_up_current) ||
		   is_float_out_of_limits("pre_pull_k", "%", conf->pre_pull_k * (float)100,
								  min_config.pre_pull_k * (float)100, max_config.pre_pull_k * (float)100) ||
		   is_float_out_of_limits("takeoff_pull_k", "%", conf->takeoff_pull_k * (float)100,
								  min_config.takeoff_pull_k * (float)100, max_config.takeoff_pull_k * (float)100) ||
		   is_float_out_of_limits("fast_pull_k", "%", conf->fast_pull_k * (float)100,
								  min_config.fast_pull_k * (float)100, max_config.fast_pull_k * (float)100) ||
		   is_int_out_of_limits("pre_pull_timeout", "milliseconds", conf->pre_pull_timeout,
								min_config.pre_pull_timeout, max_config.pre_pull_timeout) ||
		   is_int_out_of_limits("takeoff_period", "milliseconds", conf->takeoff_period,
								min_config.takeoff_period, max_config.takeoff_period);
}

// EEPROM functions
static void store_config_to_eeprom(const skypuff_config *c)
{
	mc_interface_release_motor();

	eeprom_var *e = (eeprom_var *)c;

	// Implement this if you need
	if (sizeof(skypuff_config) % sizeof(eeprom_var))
	{
		commands_printf("%s: -- store_config_to_eeprom(): config size %u must be dividable by %u",
						state_str(state), sizeof(skypuff_config), sizeof(eeprom_var));
		return;
	}

	for (unsigned int a = 0; a < sizeof(skypuff_config) / sizeof(eeprom_var); a++)
		conf_general_store_eeprom_var_custom(e + a, a);
}

static void read_config_from_eeprom(skypuff_config *c)
{
	eeprom_var *e = (eeprom_var *)c;

	for (unsigned int a = 0; a < sizeof(skypuff_config) / sizeof(eeprom_var); a++)
		conf_general_read_eeprom_var_custom(e + a, a);
}

// State setters functions
inline static void brake_state(const int cur_tac, const skypuff_state new_state, const float current,
							   const char *additional_msg)
{
	// Braking could be applied in the braking states to renew control timeout
	if (state != new_state)
	{
		float erpm = mc_interface_get_rpm();
		prev_print = loop_step;
		prev_printed_tac = cur_tac;
		commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), braking %.1fKg (%.1fA)%s",
						state_str(new_state),
						(double)tac_steps_to_meters(cur_tac), cur_tac,
						(double)erpm_to_ms(erpm), (double)erpm,
						(double)(current / config.amps_per_kg), (double)current,
						additional_msg);

		state = new_state;
	}

	prev_abs_tac = abs(cur_tac);
	smooth_motor_brake(cur_tac, prev_abs_tac, current);
	timeout_reset();
}

inline static void braking(const int cur_tac)
{
	brake_state(cur_tac, BRAKING, config.brake_current, "");
}

// This state mostly to inform UI that it's time to enable Unwinding and Pull buttons
inline static void braking_extension(const int cur_tac)
{
	brake_state(cur_tac, BRAKING_EXTENSION, config.brake_current, "");
}

inline static void manual_brake(const int cur_tac)
{
	brake_state(cur_tac, MANUAL_BRAKING, config.manual_brake_current,
				loop_step >= alive_until ? ", -- Communication timeout, send 'alive <period>'" : "");
}

inline static void pull_state(const int cur_tac, const float pull_current, const skypuff_state new_state,
							  const char *additional_msg)
{
	// Detect direction to zero depending on tachometer value
	float current = cur_tac < 0 ? pull_current : -pull_current;

	// Updates of pulling force sets this state again, do not change state_start_time
	if (state != new_state)
	{
		state_start_time = loop_step;
		state = new_state;
	}

	prev_print = loop_step;
	prev_printed_tac = cur_tac;
	float erpm = mc_interface_get_rpm();
	commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)%s",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)erpm_to_ms(erpm), (double)erpm,
					(double)(current / config.amps_per_kg), (double)current,
					additional_msg);

	prev_abs_tac = abs(cur_tac);
	prev_erpm = mc_interface_get_rpm();

	smooth_motor_current(cur_tac, prev_abs_tac, current);
	timeout_reset();
}

inline static void unwinding(const int cur_tac)
{
	pull_state(cur_tac, config.unwinding_current, UNWINDING, "");
}

inline static void rewinding(const int cur_tac)
{
	pull_state(cur_tac, config.rewinding_current, REWINDING, "");
}

inline static void pre_pull(const int cur_tac)
{
	pull_state(cur_tac, config.pre_pull_k * config.pull_current, PRE_PULL, "");
}

inline static void takeoff_pull(const int cur_tac)
{
	pull_state(cur_tac, config.takeoff_pull_k * config.pull_current, TAKEOFF_PULL, "");
}

inline static void pull(const int cur_tac)
{
	pull_state(cur_tac, config.pull_current, PULL, "");
}

inline static void fast_pull(const int cur_tac)
{
	pull_state(cur_tac, config.fast_pull_k * config.pull_current, FAST_PULL, "");
}

inline static void manual_slow_speed_up(const int cur_tac)
{
	char msg[64];
	snprintf(msg, 64, ", until: %.1fms (%.0f ERPM)",
			 (double)erpm_to_ms(config.manual_slow_erpm), (double)config.manual_slow_erpm);
	// Set high filtered value on entering speed up mode
	erpm_filtered = cur_tac < 0 ? config.manual_slow_erpm : -config.manual_slow_erpm;
	pull_state(cur_tac, config.manual_slow_speed_up_current, MANUAL_SLOW_SPEED_UP, msg);
}

inline static void manual_slow_back_speed_up(const int cur_tac)
{
	char msg[64];
	snprintf(msg, 64, ", until: %.1fms (%.0f ERPM)",
			 (double)erpm_to_ms(config.manual_slow_erpm), (double)config.manual_slow_erpm);
	// Set high filtered value on entering speed up mode
	erpm_filtered = cur_tac < 0 ? -config.manual_slow_erpm : config.manual_slow_erpm;
	pull_state(cur_tac, -config.manual_slow_speed_up_current, MANUAL_SLOW_BACK_SPEED_UP, msg);
}

inline static void slowing(const int cur_tac, const float erpm)
{
	state = SLOWING;

	prev_print = loop_step;
	prev_printed_tac = cur_tac;
	commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), until %.1fms (%.0f ERPM)",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)erpm_to_ms(erpm), (double)erpm,
					(double)erpm_to_ms(config.slow_erpm), (double)config.slow_erpm);

	// Brake or release
	if (config.slowing_current > 0.1)
		smooth_motor_brake(cur_tac, fabs(cur_tac), config.slowing_current);
	else
		smooth_motor_release();
}

inline static void slow_state(const int cur_tac, const float cur_erpm,
							  const float constant_erpm, const skypuff_state new_state)
{
	// Detect zero direction depending on tachometer value
	float to_zero_constant_erpm = cur_tac < 0 ? constant_erpm : -constant_erpm;

	state = new_state;

	prev_print = loop_step;
	prev_printed_tac = cur_tac;
	commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), constant speed %.1fms (%.0f ERPM)",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
					(double)erpm_to_ms(to_zero_constant_erpm), (double)to_zero_constant_erpm);

	prev_erpm = cur_erpm;
	smooth_motor_speed(to_zero_constant_erpm);
	timeout_reset();
}

inline static void slow(const int cur_tac, const float cur_erpm)
{
	slow_state(cur_tac, cur_erpm, config.slow_erpm, SLOW);
}

inline static void manual_slow(const int cur_tac, const float cur_erpm)
{
	slow_state(cur_tac, cur_erpm, config.manual_slow_erpm, MANUAL_SLOW);
}

inline static void manual_slow_back(const int cur_tac, const float cur_erpm)
{
	slow_state(cur_tac, cur_erpm, -config.manual_slow_erpm, MANUAL_SLOW_BACK);
}

// Example conf for my winch model: https://youtu.be/KoNegc4SzxY?t=6
static void set_example_conf(skypuff_config *cfg)
{
	// Some example ranges
	cfg->rope_length = meters_to_tac_steps(50);
	cfg->braking_length = meters_to_tac_steps(1);
	cfg->braking_extension_length = meters_to_tac_steps(0.5);
	cfg->rewinding_trigger_length = meters_to_tac_steps(0.2);
	cfg->unwinding_trigger_length = meters_to_tac_steps(0.05);
	cfg->slowing_length = meters_to_tac_steps(3);

	// Slow speeds
	cfg->slow_erpm = ms_to_erpm(1);
	cfg->manual_slow_erpm = ms_to_erpm(2);

	// Forces
	cfg->amps_per_kg = 7;			  // 7 Amps for 1Kg force
	cfg->pull_applying_period = 1500; // 1.5 secs
	cfg->brake_current = 0.3 * cfg->amps_per_kg;
	cfg->slowing_current = 0.5 * cfg->amps_per_kg;
	cfg->manual_brake_current = 1 * cfg->amps_per_kg;
	cfg->unwinding_current = 0.4 * cfg->amps_per_kg;
	cfg->rewinding_current = 0.6 * cfg->amps_per_kg;
	cfg->slow_max_current = 1 * cfg->amps_per_kg;
	cfg->manual_slow_max_current = 1 * cfg->amps_per_kg;
	cfg->manual_slow_speed_up_current = 0.4 * cfg->amps_per_kg;

	// Pull settings
	cfg->pull_current = 0.9 * cfg->amps_per_kg;
	cfg->pre_pull_k = 30 / 100.0;
	cfg->takeoff_pull_k = 60 / 100.0;
	cfg->fast_pull_k = 120 / 100.0;
	cfg->takeoff_trigger_length = meters_to_tac_steps(0.1);
	cfg->pre_pull_timeout = 2 * 1000;
	cfg->takeoff_period = 5 * 1000;
}

// Serialization/deserialization functions
inline static void serialize_drive(uint8_t *buffer, int32_t *ind)
{
	buffer[(*ind)++] = (uint8_t)mc_conf->si_motor_poles;
	buffer_append_float32_auto(buffer, mc_conf->si_gear_ratio, ind);
	buffer_append_float32_auto(buffer, mc_conf->si_wheel_diameter, ind);
}

inline static bool deserialize_drive(unsigned char *data, unsigned int len, skypuff_drive *to, int32_t *ind)
{
	const int32_t serialized_drive_v1_length = 1 + 4 * 2;

	int available_bytes = len - *ind;
	if (available_bytes < serialized_drive_v1_length)
	{
		commands_printf("%s: -- Can't deserialize drive settings -- Received %d bytes, expecting %d.",
						state_str(state), available_bytes, serialized_drive_v1_length);
		return false;
	}

	to->motor_poles = data[(*ind)++];
	to->gear_ratio = buffer_get_float32_auto(data, ind);
	to->wheel_diameter = buffer_get_float32_auto(data, ind);
	return true;
}

inline static void serialize_config(uint8_t *buffer, int32_t *ind)
{
	buffer_append_float32_auto(buffer, config.amps_per_kg, ind);
	buffer_append_int16(buffer, config.pull_applying_period, ind);
	buffer_append_int32(buffer, config.rope_length, ind);
	buffer_append_int32(buffer, config.braking_length, ind);
	buffer_append_int32(buffer, config.braking_extension_length, ind);
	buffer_append_int32(buffer, config.slowing_length, ind);
	buffer_append_float32_auto(buffer, config.slow_erpm, ind);
	buffer_append_int32(buffer, config.rewinding_trigger_length, ind);
	buffer_append_int32(buffer, config.unwinding_trigger_length, ind);
	buffer_append_float32_auto(buffer, config.pull_current, ind);
	buffer_append_float32_auto(buffer, config.pre_pull_k, ind);
	buffer_append_float32_auto(buffer, config.takeoff_pull_k, ind);
	buffer_append_float32_auto(buffer, config.fast_pull_k, ind);
	buffer_append_int32(buffer, config.takeoff_trigger_length, ind);
	buffer_append_int32(buffer, config.pre_pull_timeout, ind);
	buffer_append_int32(buffer, config.takeoff_period, ind);
	buffer_append_float32_auto(buffer, config.brake_current, ind);
	buffer_append_float32_auto(buffer, config.slowing_current, ind);
	buffer_append_float32_auto(buffer, config.manual_brake_current, ind);
	buffer_append_float32_auto(buffer, config.unwinding_current, ind);
	buffer_append_float32_auto(buffer, config.rewinding_current, ind);
	buffer_append_float32_auto(buffer, config.slow_max_current, ind);
	buffer_append_float32_auto(buffer, config.manual_slow_max_current, ind);
	buffer_append_float32_auto(buffer, config.manual_slow_speed_up_current, ind);
	buffer_append_float32_auto(buffer, config.manual_slow_erpm, ind);
}

inline static bool deserialize_config(unsigned char *data, unsigned int len, skypuff_config *to, int32_t *ind)
{
	const int32_t serialized_settings_v1_length = 4 * 25 - 2;

	int available_bytes = len - *ind;
	if (available_bytes < serialized_settings_v1_length)
	{
		commands_printf("%s: -- Can't deserialize config settings -- Received %d bytes, expecting %d.",
						state_str(state), available_bytes, serialized_settings_v1_length);
		return false;
	}

	to->amps_per_kg = buffer_get_float32_auto(data, ind);
	to->pull_applying_period = buffer_get_int16(data, ind);
	to->rope_length = buffer_get_int32(data, ind);
	to->braking_length = buffer_get_int32(data, ind);
	to->braking_extension_length = buffer_get_int32(data, ind);

	to->slowing_length = buffer_get_int32(data, ind);
	to->slow_erpm = buffer_get_float32_auto(data, ind);
	to->rewinding_trigger_length = buffer_get_int32(data, ind);
	to->unwinding_trigger_length = buffer_get_int32(data, ind);
	to->pull_current = buffer_get_float32_auto(data, ind);

	to->pre_pull_k = buffer_get_float32_auto(data, ind);
	to->takeoff_pull_k = buffer_get_float32_auto(data, ind);
	to->fast_pull_k = buffer_get_float32_auto(data, ind);
	to->takeoff_trigger_length = buffer_get_int32(data, ind);
	to->pre_pull_timeout = buffer_get_int32(data, ind);

	to->takeoff_period = buffer_get_int32(data, ind);
	to->brake_current = buffer_get_float32_auto(data, ind);
	to->slowing_current = buffer_get_float32_auto(data, ind);
	to->manual_brake_current = buffer_get_float32_auto(data, ind);
	to->unwinding_current = buffer_get_float32_auto(data, ind);

	to->rewinding_current = buffer_get_float32_auto(data, ind);
	to->slow_max_current = buffer_get_float32_auto(data, ind);
	to->manual_slow_max_current = buffer_get_float32_auto(data, ind);
	to->manual_slow_speed_up_current = buffer_get_float32_auto(data, ind);
	to->manual_slow_erpm = buffer_get_float32_auto(data, ind);

	return true;
}

inline static void get_stats(float *erpm, float *motor_amps, float *power)
{
	*erpm = mc_interface_get_rpm();
	*motor_amps = mc_interface_read_reset_avg_motor_current();
	*power = v_in_filtered * mc_interface_read_reset_avg_input_current();
}

inline float get_battery_temp(void)
{
	return 0; // Not implemented yet
}

inline static void serialize_alive(uint8_t *buffer, int32_t *ind, const int cur_tac)
{
	float erpm;
	float motor_amps;
	float power;
	get_stats(&erpm, &motor_amps, &power);

	buffer[(*ind)++] = current_motor_state.mode;
	buffer_append_int32(buffer, cur_tac, ind);
	buffer_append_float32(buffer, erpm, 1e1, ind);
	buffer_append_float32(buffer, motor_amps, 1e1, ind);
	buffer_append_float32(buffer, power, 1e1, ind);
}

inline static bool deserialize_alive(unsigned char *data, unsigned int len, int32_t *ind)
{
	const int32_t serialized_alive_length = 4;

	int available_bytes = len - *ind;
	if (available_bytes < serialized_alive_length)
	{
		commands_printf("%s: -- Can't deserialize alive command -- Received %d bytes, expecting %d.",
						state_str(state), available_bytes, serialized_alive_length);
		return false;
	}

	alive_inc = buffer_get_int32(data, ind);
	return true;
}

// No sense to split initial state into many commands
inline static void send_conf(const int cur_tac)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = SK_COMM_SETTINGS_V1;
	buffer[ind++] = state;
	buffer[ind++] = mc_interface_get_fault();

	// Let UI to set limits of the scales
	buffer_append_float16(buffer, mc_conf->lo_current_max, 1e1, &ind);
	buffer_append_float16(buffer, mc_conf->l_temp_fet_start, 1e1, &ind);
	buffer_append_float16(buffer, mc_conf->l_temp_motor_start, 1e1, &ind);
	buffer_append_float32(buffer, fmax(mc_conf->l_min_vin, mc_conf->l_battery_cut_start), 1e2, &ind);
	buffer_append_float32(buffer, mc_conf->l_max_vin, 1e2, &ind);

	buffer[ind++] = mc_conf->si_battery_cells;
	buffer[ind++] = mc_conf->si_battery_type;

	// And stats
	buffer_append_float32(buffer, v_in_filtered, 1e2, &ind);
	buffer_append_float16(buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
	buffer_append_float16(buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
	buffer_append_float16(buffer, get_battery_temp(), 1e1, &ind);
	buffer_append_float32(buffer, mc_interface_get_watt_hours(false), 1e4, &ind);
	buffer_append_float32(buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind);
	serialize_drive(buffer, &ind);
	serialize_config(buffer, &ind);
	serialize_alive(buffer, &ind, cur_tac);

	if (ind > max_buf_size)
	{
		commands_printf("%s: -- ALARMA!!! -- send_conf() max buffer size %d, serialized bufer %d bytes. Memory corrupted!",
						state_str(state), max_buf_size, ind);
	}

	commands_send_app_data(buffer, ind);
}

// Send motor mode, speed and current amps as reply to alive command
inline static void send_alive(const int cur_tac)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	// Serialization magic: version, state, position, speed, drive settings, skypuff settings
	buffer[ind++] = SK_COMM_ALIVE;
	serialize_alive(buffer, &ind, cur_tac);

	if (ind > max_buf_size)
	{
		commands_printf("%s: -- ALARMA!!! -- send_alive() max buffer size %d, serialized bufer %d bytes. Memory corrupted!",
						state_str(state), max_buf_size, ind);
	}

	commands_send_app_data(buffer, ind);
}

void custom_app_data_handler(unsigned char *data, unsigned int len)
{
	if (len < 1)
	{
		commands_printf("%s: -- Can't deserialize command -- No data.",
						state_str(state));
		return;
	}

	skypuff_custom_app_data_command command;

	int32_t ind = 0;
	command = data[ind++];

	switch (command)
	{
	case SK_COMM_ALIVE:
		// Set alive_inc right there
		if (!deserialize_alive(data, len, &ind))
			return;

		break;
	case SK_COMM_SETTINGS_V1:
		if (!deserialize_drive(data, len, &set_drive, &ind))
			return;
		if (!deserialize_config(data, len, &set_config, &ind))
			return;

		terminal_command = SET_CONF;

		break;
	default:
		commands_printf("%s: -- Can't deserialize command -- Unknown command '%d'.",
						state_str(state), (int)command);
		return;
	}

	// Extra bytes?
	if (ind != (int32_t)len)
	{
		commands_printf("%s: -- Extra bytes received -- Extra %d bytes on %s command deserialization.",
						state_str(state), (len - ind), sk_command_str(command));
		return;
	}
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
	commands_printf("app_skypuff started");

	// Reset tachometer on app start to prevent instant unwinding to zero
	mc_interface_get_tachometer_value(true);

	mc_conf = mc_interface_get_configuration();

	// Static variables initial state
	timeout_reset_interval = app_get_configuration()->timeout_msec / 2;
	prev_print = INT_MIN / 2;
	prev_printed_tac = INT_MIN / 2;
	alive_until = 0;
	prev_abs_tac = 0;
	prev_erpm = 0;
	prev_print = INT_MIN / 2;
	prev_temps_print = INT_MIN / 2;
	prev_printed_fets_temp = -1000;
	prev_printed_motor_temp = -1000;
	prev_printed_bat_temp = -1000;
	prev_printed_wh_out = 0;
	prev_printed_wh_in = 0;
	prev_printed_bat_v = 0;
	prev_printed_tac = INT_MIN / 2;
	prev_printed_fault = FAULT_CODE_NONE;
	v_in_filtered = GET_INPUT_VOLTAGE();
	terminal_command = DO_NOTHING;
	stop_now = false;

	smooth_motor_release();

	read_config_from_eeprom(&config);

	// Update smooth speed for current pull
	smooth_calculate_new_speed();

	// Check system drive settings and our config for limits
	set_drive.motor_poles = mc_conf->si_motor_poles;
	set_drive.gear_ratio = mc_conf->si_gear_ratio;
	set_drive.wheel_diameter = mc_conf->si_wheel_diameter;

	state = is_drive_config_out_of_limits(&set_drive) || is_config_out_of_limits(&config) ? UNINITIALIZED : BRAKING;

	commands_set_app_data_handler(custom_app_data_handler);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
		"set_zero",
		"Move SkyPUFF zero point to this position",
		"", terminal_set_zero);
	terminal_register_command_callback(
		"skypuff",
		"Print configuration",
		"", terminal_print_conf);
	terminal_register_command_callback(
		"get_conf",
		"Send SkuPUFF serialized state and settings in the COMM_CUSTOM_APP_DATA payload",
		"", terminal_get_conf);
	terminal_register_command_callback(
		"example_conf",
		"Set SkyPUFF model winch configuration",
		"", terminal_set_example_conf);
	terminal_register_command_callback(
		"alive",
		"Prolong SkyPUFF communication alive period",
		"[milliseconds]", terminal_alive);
	terminal_register_command_callback(
		"set",
		"Set new SkyPUFF state: MANUAL_BRAKING, UNWINDING, MANUAL_SLOW, MANUAL_SLOW_BACK, PRE_PULL, TAKEOFF_PULL, PULL, FAST_PULL",
		"state", terminal_set_state);
	terminal_register_command_callback(
		"force",
		"Set SkyPUFF pull force",
		"[kg]", terminal_set_pull_force);
#ifdef DEBUG_SMOOTH_MOTOR
	terminal_register_command_callback(
		"smooth",
		"Debug smooth motor control.",
		"<release/brake/current/speed> [current/erpm]", terminal_smooth);
#endif

	// Run control loop thread
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa), NORMALPRIO, my_thread, NULL);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
	commands_set_app_data_handler(NULL);
	terminal_unregister_callback(terminal_set_zero);
	terminal_unregister_callback(terminal_print_conf);
	terminal_unregister_callback(terminal_get_conf);
	terminal_unregister_callback(terminal_set_example_conf);
	terminal_unregister_callback(terminal_alive);
	terminal_unregister_callback(terminal_set_state);
	terminal_unregister_callback(terminal_set_pull_force);
#ifdef DEBUG_SMOOTH_MOTOR
	terminal_unregister_callback(terminal_smooth);
#endif

	stop_now = true;
	while (is_running)
	{
		chThdSleepMilliseconds(1);
	}
	commands_printf("app_skypuff stopped");
}

void app_custom_configure(app_configuration *conf)
{
	(void)conf;
}

// The same code for unwinding, rewinding and pulling states
// Returns true on transition
static bool brake_or_slowing(const int cur_tac, const int abs_tac)
{
	// We are in the braking range?
	if (abs_tac <= config.braking_length)
	{
		braking(cur_tac);
		return true;
	}

	// Slowing range and direction to zero?
	if (abs_tac <= config.braking_length + config.slowing_length)
	{
		float erpm = mc_interface_get_rpm();

		// Check direction and erpm to decide about slowing
		if (cur_tac < 0 && erpm > config.slow_erpm)
		{
			slowing(cur_tac, erpm);
			return true;
		}
		else if (cur_tac >= 0 && erpm < -config.slow_erpm)
		{
			slowing(cur_tac, erpm);
			return true;
		}
	}

	return false;
}

inline static void print_stats_periodically(void)
{
	UTILS_LP_FAST(v_in_filtered, GET_INPUT_VOLTAGE(), 0.1);
	mc_fault_code f = mc_interface_get_fault();

	// Will print new fault immediately
	if (f != prev_printed_fault || loop_step - prev_temps_print > temps_print_delay)
	{
		float fets_temp = mc_interface_temp_fet_filtered();
		float motor_temp = mc_interface_temp_motor_filtered();
		float bat_temp = get_battery_temp();
		float wh_in = mc_interface_get_watt_hours(false);
		float wh_out = mc_interface_get_watt_hours_charged(false);

		// Small changes?
		if (f == prev_printed_fault &&
			fabs(fets_temp - prev_printed_fets_temp) < (double)1 &&
			(motor_temp < -40 || fabs(motor_temp - prev_printed_motor_temp) < (double)1) &&
			fabs(bat_temp - prev_printed_bat_temp) < (double)1 &&
			fabs(wh_in - prev_printed_wh_in) < (double)0.001 &&
			fabs(wh_out - prev_printed_wh_out) < (double)0.001 &&
			fabs(v_in_filtered - prev_printed_bat_v) < (double)0.1)
		{
			prev_temps_print = loop_step;
			return;
		}

		prev_printed_fets_temp = fets_temp;
		prev_printed_motor_temp = motor_temp;
		prev_printed_bat_temp = bat_temp;
		prev_printed_wh_in = wh_in;
		prev_printed_wh_out = wh_out;
		prev_printed_bat_v = v_in_filtered;
		prev_temps_print = loop_step;
		prev_printed_fault = f;

		commands_printf("%s: fault %s, t_fets %.1fC, t_motor %.1fC, t_bat %.1fC, wh_in %.3fWh, wh_out %.3fWh, v_bat %.2fV",
						state_str(state),
						mc_interface_fault_to_string(prev_printed_fault),
						(double)prev_printed_fets_temp,
						(double)prev_printed_motor_temp,
						(double)prev_printed_bat_temp,
						(double)prev_printed_wh_in,
						(double)prev_printed_wh_out,
						(double)prev_printed_bat_v);
	}
}

inline static void print_position_periodically(const int cur_tac, const int delay, const char *additional_msg)
{
	// prolong delay if not moving
	if (cur_tac == prev_printed_tac)
	{
		prev_print = loop_step;
		return;
	}

	if (loop_step - prev_print > delay)
	{
		prev_print = loop_step;
		prev_printed_tac = cur_tac;
		float erpm = mc_interface_get_rpm();
		commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%0.f ERPM)%s",
						state_str(state),
						(double)tac_steps_to_meters(cur_tac), cur_tac,
						(double)erpm_to_ms(erpm), (double)erpm,
						additional_msg);
	}
}

inline static bool is_filtered_speed_too_low(const float cur_erpm)
{
	UTILS_LP_FAST(erpm_filtered, cur_erpm, 0.01);

	// Takes about 500 iterations if speed is about zero to reach filtered 0.1
	if (fabs(erpm_filtered) < (double)0.1)
	{
		commands_printf("%s: -- Too slow speed up", state_str(state));
		return true;
	}

	return false;
}

inline static void brake_or_unwinding(const int cur_tac, const int abs_tac)
{
	if (abs_tac <= config.braking_length)
		braking(cur_tac);
	else
		unwinding(cur_tac);
}

inline static void brake_or_manual_brake(const int cur_tac, const int abs_tac)
{
	if (abs_tac <= config.braking_length)
		braking(cur_tac);
	else
		manual_brake(cur_tac);
}

inline static bool unwinded_to_opposite_braking_zone(const int cur_tac, const float cur_erpm)
{
	if ((cur_erpm > 0 && cur_tac >= config.braking_length) ||
		(cur_erpm < 0 && cur_tac <= -config.braking_length))
	{
		commands_printf("%s: -- Unwinded to opposite braking zone", state_str(state));
		return true;
	}

	return false;
}

// More prints to tweak slowing zone
inline static void slowing_or_speed_up_print(const int cur_tac, const float cur_erpm, const float target_erpm)
{
	if (loop_step - prev_print > short_print_delay)
	{
		int distance_left = abs(cur_tac) - config.braking_length;
		prev_print = loop_step;
		commands_printf(
			"%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), until %.1fms (%.0f ERPM), -- %.2fm to braking zone",
			state_str(state),
			(double)tac_steps_to_meters(cur_tac), cur_tac,
			(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
			(double)erpm_to_ms(target_erpm), (double)target_erpm,
			(double)tac_steps_to_meters(distance_left));
	}
}

// Control loop state machine
inline static void process_states(const int cur_tac, const int abs_tac)
{
	float cur_erpm, abs_erpm;
	float cur_current, abs_current;

	switch (state)
	{
	case UNINITIALIZED:
		// Only SET_CONF will take us from here
		{
			const char *msg;

			if (abs_tac > config.braking_length)
				msg = ", -- Position is out from safe braking zone!";
			else
				msg = ", -- Waiting for 'example_conf' or COMM_CUSTOM_APP_DATA";

			print_position_periodically(cur_tac, long_print_delay, msg);
		}
		break;
	case BRAKING:
		// Moved to unwinding zone?
		if (abs_tac > config.braking_length)
		{
			braking_extension(cur_tac);
			break;
		}

		// Timeout thread will remove braking every second by default
		// Apply brake again if position changed
		if (timeout_has_timeout() && abs_tac != prev_abs_tac)
			braking(cur_tac);
		else
			prev_abs_tac = abs_tac;

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case BRAKING_EXTENSION:
		// Moved to unwinding zone?
		if (abs_tac > config.braking_length + config.braking_extension_length)
		{
			unwinding(cur_tac);
			break;
		}

		// Moved to braking zone?
		if (abs_tac <= config.braking_length)
		{
			braking(cur_tac);
		}

		// Timeout thread will remove braking every second by default
		// Apply brake again if position changed
		if (timeout_has_timeout() && abs_tac != prev_abs_tac)
			braking_extension(cur_tac);
		else
			prev_abs_tac = abs_tac;

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case MANUAL_BRAKING:
		// Apply brake again on timeout and position change
		if (timeout_has_timeout() && abs_tac != prev_abs_tac)
			manual_brake(cur_tac);
		else
			prev_abs_tac = abs_tac;

		print_position_periodically(cur_tac, long_print_delay,
									loop_step >= alive_until ? ", -- Communication timeout" : "");

		break;
	case UNWINDING:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		// Use prev_abs_tac as max tachometer
		if (abs_tac > prev_abs_tac)
		{
			// Print debug message if we are going out from slowing zone
			int eof_slowing = config.braking_length + config.slowing_length;
			if (prev_abs_tac < eof_slowing && abs_tac >= eof_slowing)
			{
				commands_printf("%s: -- Unwinded from slowing zone %.2fm (%d steps)",
								state_str(state),
								(double)tac_steps_to_meters(cur_tac), cur_tac);
			}

			// Update maximum value of tachometer
			prev_abs_tac = abs_tac;
		}

		// Going back more then config.rewinding_trigger_length?
		if (abs_tac < prev_abs_tac - config.rewinding_trigger_length)
		{
			rewinding(cur_tac);
			break;
		}

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case REWINDING:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		// Now use prev_abs_tac as min value
		if (abs_tac < prev_abs_tac)
			prev_abs_tac = abs_tac;

		// Unwinding again?
		if (abs_tac > prev_abs_tac + config.unwinding_trigger_length)
			unwinding(cur_tac);

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case SLOWING:
		// We are in the braking range?
		if (abs_tac <= config.braking_length)
		{
			braking(cur_tac);
			break;
		}

		cur_erpm = mc_interface_get_rpm();
		abs_erpm = fabs(cur_erpm);

		// Slow enough for PID speed?
		if (abs_erpm < config.slow_erpm)
		{
			slow(cur_tac, cur_erpm);
			break;
		}

		slowing_or_speed_up_print(cur_tac, cur_erpm, config.slow_erpm);
		break;
	case SLOW:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		cur_current = mc_interface_get_tot_current_directional_filtered();
		abs_current = fabs(cur_current);
		cur_erpm = mc_interface_get_rpm();

		// If current above the limits - brake or unwinding
		if (abs_current > config.slow_max_current)
		{
			commands_printf(
				"SLOW: speed %.1fms (%.0f ERPM), -- Pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.amps_per_kg), (double)cur_current,
				(double)(config.slow_max_current / config.amps_per_kg), (double)config.slow_max_current);

			brake_or_unwinding(cur_tac, abs_tac);
			break;
		}

		// Slowly rewinded more then opposite side of braking  zone?
		if (unwinded_to_opposite_braking_zone(cur_tac, cur_erpm))
		{
			braking(cur_tac);
			break;
		}

		if (loop_step - prev_print > long_print_delay)
		{
			prev_print = loop_step;
			commands_printf("SLOW: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.amps_per_kg), (double)cur_current);
		}
		break;
	case MANUAL_SLOW_SPEED_UP:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		cur_erpm = mc_interface_get_rpm();

		// Rotating direction changed or stopped?
		if (is_filtered_speed_too_low(cur_erpm))
		{
			manual_brake(cur_tac);
			break;
		}

		// Fast enough for PID speed in zero direction?
		if ((cur_tac < 0 && cur_erpm >= config.manual_slow_erpm) ||
			(cur_tac >= 0 && cur_erpm <= -config.manual_slow_erpm))
		{
			manual_slow(cur_tac, cur_erpm);
			break;
		}

		slowing_or_speed_up_print(cur_tac, cur_erpm, config.manual_slow_erpm);

		break;
	case MANUAL_SLOW_BACK_SPEED_UP:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		cur_erpm = mc_interface_get_rpm();

		// Rotating direction changed or stopped?
		if (is_filtered_speed_too_low(cur_erpm))
		{
			manual_brake(cur_tac);
			break;
		}

		// Fast enough for PID speed in opposite from zero direction?
		if ((cur_tac >= 0 && cur_erpm >= config.manual_slow_erpm) ||
			(cur_tac < 0 && cur_erpm <= -config.manual_slow_erpm))
		{
			manual_slow_back(cur_tac, cur_erpm);
			break;
		}

		slowing_or_speed_up_print(cur_tac, cur_erpm, config.manual_slow_erpm);
		break;
	case MANUAL_SLOW:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		cur_current = mc_interface_get_tot_current_directional_filtered();
		abs_current = fabs(cur_current);
		cur_erpm = mc_interface_get_rpm();

		// If slowing zone and speed is more then SLOW, go SLOWING
		if (abs_tac < config.braking_length + config.slowing_length &&
			config.slow_erpm < config.manual_slow_erpm)
		{
			commands_printf("%s: -- Slowing zone, manual slow speed is too high, go SLOWING", state_str(state));
			slowing(cur_tac, cur_erpm);
			break;
		}

		// If current is above the limits
		if (abs_current > config.manual_slow_max_current)
		{
			commands_printf(
				"MANUAL_SLOW: speed %.1fms (%.0f ERPM), -- Pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.amps_per_kg), (double)cur_current,
				(double)(config.manual_slow_max_current / config.amps_per_kg), (double)config.manual_slow_max_current);

			brake_or_manual_brake(cur_tac, abs_tac);
			break;
		}

		// Slowly rewinded more then opposite side of braking zone?
		if (unwinded_to_opposite_braking_zone(cur_tac, cur_erpm))
		{
			braking(cur_tac);
			break;
		}

		if (loop_step - prev_print > long_print_delay)
		{
			prev_print = loop_step;
			commands_printf("MANUAL_SLOW: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.amps_per_kg), (double)cur_current);
		}

		break;
	case MANUAL_SLOW_BACK:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		cur_current = mc_interface_get_tot_current_directional_filtered();
		abs_current = fabs(cur_current);
		cur_erpm = mc_interface_get_rpm();

		// If current is above the limits
		if (abs_current > config.manual_slow_max_current)
		{
			commands_printf(
				"MANUAL_SLOW_BACK: speed %.1fms (%.0f ERPM), -- Pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.amps_per_kg), (double)cur_current,
				(double)(config.manual_slow_max_current / config.amps_per_kg),
				(double)config.manual_slow_max_current);

			brake_or_manual_brake(cur_tac, abs_tac);
			break;
		}

		if (loop_step - prev_print > long_print_delay)
		{
			prev_print = loop_step;
			commands_printf("MANUAL_SLOW_BACK: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.amps_per_kg), (double)cur_current);
		}

		break;
	case PRE_PULL:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		// Enough time to tight the rope?
		int timeout_step = state_start_time + config.pre_pull_timeout;
		if (loop_step == timeout_step)
		{
			float erpm = mc_interface_get_rpm();
			commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Pre pull %.1fs timeout passed, saving position",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(erpm), (double)erpm,
							(double)config.pre_pull_timeout / (double)1000.0);
			prev_abs_tac = abs_tac;
		}

		// Timeout passed and moved enough to takeoff?
		if (loop_step > timeout_step && abs(prev_abs_tac - abs_tac) >= config.takeoff_trigger_length)
		{
			float erpm = mc_interface_get_rpm();
			commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Motion %.2fm (%d steps) detected",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(erpm), (double)erpm,
							(double)tac_steps_to_meters(config.takeoff_trigger_length), config.takeoff_trigger_length);
			takeoff_pull(cur_tac);
			break;
		}

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case TAKEOFF_PULL:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		// Enough time of weak takeoff pull?
		if (loop_step >= state_start_time + config.takeoff_period)
		{
			float erpm = mc_interface_get_rpm();
			commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Takeoff %.1fs timeout passed",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(erpm), (double)erpm,
							(double)config.takeoff_period / (double)1000.0);
			pull(cur_tac);
			break;
		}

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case PULL:
	case FAST_PULL:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
#ifdef DEBUG_SMOOTH_MOTOR
	case MANUAL_DEBUG_SMOOTH:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
#endif
	default:
		commands_printf("SkyPUFF: unknown control loop state, exiting!");
		stop_now = true;
	}
}

inline static void print_conf(const int cur_tac)
{
	float erpm;
	float motor_amps;
	float power;
	get_stats(&erpm, &motor_amps, &power);

	commands_printf("VESC additional info:");
	commands_printf("  wheel diameter: %.2fmm", (double)(mc_conf->si_wheel_diameter * 1000));
	commands_printf("  motor poles: %dp", mc_conf->si_motor_poles);
	commands_printf("  gear ratio: %.5f", (double)mc_conf->si_gear_ratio);

	commands_printf("SkyPUFF configuration version %s:", sk_command_str(SK_COMM_SETTINGS_V1));
	commands_printf("  amperes per 1kg force: %.1fAKg", (double)config.amps_per_kg);
	commands_printf("  pull applying period: %.1fs (%d loops)", (double)config.pull_applying_period / (double)1000.0, config.pull_applying_period);
	commands_printf("  rope length: %.2fm (%d steps)", (double)tac_steps_to_meters(config.rope_length), config.rope_length);
	commands_printf("  braking range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.braking_length), config.braking_length);
	commands_printf("  braking extension range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.braking_extension_length), config.braking_extension_length);
	commands_printf("  slowing range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.slowing_length), config.slowing_length);
	commands_printf("  rewinding trigger range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.rewinding_trigger_length), config.rewinding_trigger_length);
	commands_printf("  unwinding trigger range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.unwinding_trigger_length), config.unwinding_trigger_length);
	commands_printf("  brake force: %.2fkg (%.1fA)", (double)(config.brake_current / config.amps_per_kg), (double)config.brake_current);
	commands_printf("  manual brake force: %.2fkg (%.1fA)", (double)(config.manual_brake_current / config.amps_per_kg), (double)config.manual_brake_current);
	commands_printf("  unwinding force: %.2fkg (%.1fA)", (double)(config.unwinding_current / config.amps_per_kg), (double)config.unwinding_current);
	commands_printf("  rewinding force: %.2fkg (%.1fA)", (double)(config.rewinding_current / config.amps_per_kg), (double)config.rewinding_current);
	commands_printf("  slowing brake force: %.2fkg (%.1fA)", (double)(config.slowing_current / config.amps_per_kg), (double)config.slowing_current);
	commands_printf("  slow speed: %.1fms (%.0f ERPM)", (double)erpm_to_ms(config.slow_erpm), (double)config.slow_erpm);
	commands_printf("  maximum slow force: %.2fkg (%.1fA)", (double)(config.slow_max_current / config.amps_per_kg), (double)config.slow_max_current);
	commands_printf("  manual slow max force: %.2fkg (%.1fA)", (double)(config.manual_slow_max_current / config.amps_per_kg), (double)config.manual_slow_max_current);
	commands_printf("  manual slow speed up force: %.2fkg (%.1fA)", (double)(config.manual_slow_speed_up_current / config.amps_per_kg), (double)config.manual_slow_speed_up_current);
	commands_printf("  manual slow speed: %.1fms (%.0f ERPM)", (double)erpm_to_ms(config.manual_slow_erpm), (double)config.manual_slow_erpm);

	commands_printf("  pull force: %.2fkg (%.1fA)", (double)(config.pull_current / config.amps_per_kg), (double)config.pull_current);
	commands_printf("  takeoff trigger range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.takeoff_trigger_length), config.takeoff_trigger_length);
	commands_printf("  pre pull timeout: %.1fs (%d loops)", (double)config.pre_pull_timeout / (double)1000.0, config.pre_pull_timeout);
	commands_printf("  takeoff period: %.1fs (%d loops)", (double)config.takeoff_period / (double)1000.0, config.takeoff_period);
	commands_printf("  pre pull coefficient: %.0f%% (%.2fkg, %.5f)", (double)config.pre_pull_k * (double)100.0, (double)(config.pull_current / config.amps_per_kg * config.pre_pull_k), (double)config.pre_pull_k);
	commands_printf("  takeoff pull coefficient: %.0f%% (%.2fkg, %.5f)", (double)config.takeoff_pull_k * (double)100.0, (double)(config.pull_current / config.amps_per_kg * config.takeoff_pull_k), (double)config.takeoff_pull_k);
	commands_printf("  fast pull coefficient: %.0f%% (%.2fkg, %.5f)", (double)config.fast_pull_k * (double)100.0, (double)(config.pull_current / config.amps_per_kg * config.fast_pull_k), (double)config.fast_pull_k);

	commands_printf("SkyPUFF state:");
	commands_printf("  %s: pos %.2fm (%d steps), speed %.1fm/s (%.1f ERPM)", state_str(state), (double)tac_steps_to_meters(cur_tac), cur_tac, (double)erpm_to_ms(erpm), (double)erpm);
	commands_printf("  motor state %s: %.2fkg (%.1fA), power: %.1fW", motor_mode_str(current_motor_state.mode), (double)(motor_amps / config.amps_per_kg), (double)motor_amps, (double)power);
	commands_printf("  timeout reset interval: %dms", timeout_reset_interval);
	commands_printf("  calculated force changing speed: %.2fKg/sec (%.1fA/sec)", (double)(amps_per_sec / config.amps_per_kg), (double)amps_per_sec);
	commands_printf("  loop counter: %d, alive until: %d, %s", loop_step, alive_until, loop_step >= alive_until ? "communication timeout" : "no timeout");
}

inline static void process_terminal_commands(int *cur_tac, int *abs_tac)
{
	// In case of new command during next switch
	skypuff_terminal_command prev_command = terminal_command;

	switch (terminal_command)
	{
	case DO_NOTHING:
		return;

	case SET_ZERO:
		switch (state)
		{
		case UNINITIALIZED:
		case BRAKING:
		case BRAKING_EXTENSION:
		case MANUAL_BRAKING:
		{
			float erpm = mc_interface_get_rpm();
			mc_interface_get_tachometer_value(true);
			prev_abs_tac = 0;
			prev_printed_tac = 0;
			*cur_tac = 0;
			*abs_tac = 0;
			commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%0.f ERPM), -- Zero is set",
							state_str(state),
							(double)tac_steps_to_meters(*cur_tac), *cur_tac,
							(double)erpm_to_ms(erpm), (double)erpm);
		}
		break;

		default:
			commands_printf("%s: -- Can't set zero -- Only possible from UNITIALIZED or BRAKING states",
							state_str(state));
			break;
		}
		break;
	case SET_MANUAL_BRAKING:
		switch (state)
		{
		case UNINITIALIZED:
			commands_printf("%s: -- Can't switch to MANUAL_BRAKING -- Not possible from UNITIALIZED", state_str(state));
			break;
		default:
			manual_brake(*cur_tac);
			break;
		}

		break;
	case SET_MANUAL_SLOW:
		switch (state)
		{
		case MANUAL_BRAKING:
			if (*abs_tac <= config.braking_length)
			{
				commands_printf("%s: -- Can't switch to MANUAL_SLOW -- Please unwind from braking zone %.1fm (%d steps)",
								state_str(state), (double)tac_steps_to_meters(config.braking_length),
								config.braking_length);
				break;
			}
			manual_slow_speed_up(*cur_tac);
			break;
		default:
			commands_printf("%s: -- Can't switch to MANUAL_SLOW -- Only possible from MANUAL_BRAKING", state_str(state));
			break;
		}

		break;
	case SET_MANUAL_SLOW_BACK:
		switch (state)
		{
		case MANUAL_BRAKING:
			if (*abs_tac <= config.braking_length)
			{
				commands_printf("%s: -- Can't switch to MANUAL_SLOW_BACK -- Please unwind from braking zone %.1fm (%d steps)",
								state_str(state), (double)tac_steps_to_meters(config.braking_length),
								config.braking_length);
				break;
			}
			manual_slow_back_speed_up(*cur_tac);
			break;
		default:
			commands_printf("%s: -- Can't switch to MANUAL_SLOW_BACK -- Only possible from MANUAL_BRAKING", state_str(state));
			break;
		}

		break;
	case SET_UNWINDING:
		// Just warning for terminal mode
		if (loop_step >= alive_until)
		{
			commands_printf("%s: -- Can't switch to UNWINDING -- Update timeout with 'alive <period>' before",
							state_str(state));
			break;
		}

		// Possible from manual braking or pulling states
		switch (state)
		{
		case BRAKING_EXTENSION:
		case MANUAL_BRAKING:
		case PRE_PULL:
		case TAKEOFF_PULL:
		case PULL:
		case FAST_PULL:
			unwinding(*cur_tac);
			break;

		default:
			commands_printf("%s: -- Can't switch to UNWINDING -- Only possible from BRAKING_EXTENSION, MANUAL_BRAKING, PRE_PULL, TAKEOFF_PULL, PULL, FAST_PULL", state_str(state));
			break;
		}

		break;
	case SET_BRAKING_EXTENSION:
		switch (state)
		{
		case MANUAL_BRAKING:
		case UNWINDING:
		case REWINDING:
			braking_extension(*cur_tac);
			break;

		default:
			commands_printf("%s: -- Can't switch to BRAKING_EXTENSION -- Only possible from UNWINDING, REWINDING or MANUAL_BRAKING", state_str(state));
			break;
		}

		break;
	case SET_PULL_FORCE:
		// We need correct config.amps_per_kg and drive settings
		if (state == UNINITIALIZED)
		{
			commands_printf("%s: -- Can't update pull force from UNITIALIZED", state_str(state));
			break;
		}

		// Calculate amperes from Kg
		float pull_current = config.amps_per_kg * terminal_pull_kg;

		if (is_pull_out_of_limits("pull_current",
								  pull_current,
								  min_config.pull_current,
								  max_config.pull_current))
			break;

		config.pull_current = pull_current;

		// Update smooth speed
		smooth_calculate_new_speed();

		commands_printf("%s: -- %.2fKg (%.1fA, %.1fA/sec) is set",
						state_str(state), (double)terminal_pull_kg, (double)config.pull_current,
						(double)amps_per_sec);

		// Update pull force now?
		switch (state)
		{
		case PRE_PULL:
			pre_pull(*cur_tac);
			break;
		case TAKEOFF_PULL:
			takeoff_pull(*cur_tac);
			break;
		case PULL:
			pull(*cur_tac);
			break;
		case FAST_PULL:
			fast_pull(*cur_tac);
			break;

		default:
			break;
		}

		break;
	case SET_PRE_PULL:
		switch (state)
		{
		case BRAKING_EXTENSION:
		case UNWINDING:
		case REWINDING:
			pre_pull(*cur_tac);
			break;
		default:
			commands_printf("%s: -- Can't switch to PRE_PULL -- Only possible from BRAKING_EXTENSION, UNWINDING or REWINDING", state_str(state));
			break;
		}

		break;
	case SET_TAKEOFF_PULL:
		switch (state)
		{
		case PRE_PULL:
			takeoff_pull(*cur_tac);
			break;

		default:
			commands_printf("%s: -- Can't switch to TAKEOFF_PULL -- Only possible from PRE_PULL", state_str(state));
			break;
		}

		break;
	case SET_PULL:
		switch (state)
		{
		case BRAKING_EXTENSION:
		case TAKEOFF_PULL:
		case UNWINDING:
		case REWINDING:
			pull(*cur_tac);
			break;

		default:
			commands_printf("%s: -- Can't switch to PULL -- Only possible from BRAKING_EXTENSION, TAKEOFF_PULL, UNWINDING or REWINDING", state_str(state));
		}

		break;
	case SET_FAST_PULL:
		switch (state)
		{
		case PULL:
			fast_pull(*cur_tac);
			break;

		default:
			commands_printf("%s: -- Can't switch to FAST_PULL -- Only possible from PULL", state_str(state));
			break;
		}

		break;
	case PRINT_CONF:
		print_conf(*cur_tac);

		break;
	case SEND_CONF:
		send_conf(*cur_tac);

		break;
	case SET_CONF:
		switch (state)
		{
		case UNINITIALIZED:
		case BRAKING:
		case BRAKING_EXTENSION:
		case MANUAL_BRAKING:
			if (is_drive_config_out_of_limits(&set_drive) || is_config_out_of_limits(&set_config))
				break;

			// Use braking_length from received config
			if (*abs_tac > set_config.braking_length + set_config.braking_extension_length)
			{
				commands_printf("%s: -- Can't set configuration -- Position is out of safe braking zone", state_str(state));
				break;
			}

			// mc_configuration changed?
			if (set_drive.motor_poles != mc_conf->si_motor_poles ||
				set_drive.gear_ratio != mc_conf->si_gear_ratio ||
				set_drive.wheel_diameter != mc_conf->si_wheel_diameter)
			{
				mc_configuration new_mc_conf = *mc_conf;

				new_mc_conf.si_motor_poles = set_drive.motor_poles;
				new_mc_conf.si_gear_ratio = set_drive.gear_ratio;
				new_mc_conf.si_wheel_diameter = set_drive.wheel_diameter;

				// TODO: move this code into separate function and use from commands.c
				conf_general_store_mc_configuration(&new_mc_conf);
				mc_interface_set_configuration(&new_mc_conf);
				mc_conf = mc_interface_get_configuration();
			}

			config = set_config;

			smooth_calculate_new_speed();

			store_config_to_eeprom(&config);

			commands_printf("%s: -- Settings are set -- Have a nice puffs!", state_str(state));

			// Announce new settings
			send_conf(*cur_tac);

			// Forget about UNITIALIZED :)
			if (state == UNINITIALIZED)
				braking(*cur_tac);

			break;
		default:
			commands_printf("%s: -- Can't set configuration -- Only possible from UNITIALIZED or any BRAKING states",
							state_str(state));
		}

		break;
#ifdef DEBUG_SMOOTH_MOTOR
	case SET_SMOOTH:
		state = MANUAL_DEBUG_SMOOTH;

		switch (terminal_motor_state.mode)
		{
		case MOTOR_RELEASED:
			smooth_motor_release();
			break;
		case MOTOR_BRAKING:
			smooth_motor_brake(*cur_tac, *abs_tac, terminal_motor_state.param.current);
			break;
		case MOTOR_CURRENT:
			smooth_motor_current(*cur_tac, *abs_tac, terminal_motor_state.param.current);
			break;
		case MOTOR_SPEED:
			smooth_motor_speed(terminal_motor_state.param.erpm);
			break;

		default:
			break;
		}

		break;
#endif
	default:
		commands_printf("SkyPUFF: unknown terminal command, exiting!");
		stop_now = true;
	}

	// Nothing changed during switch?
	if (prev_command == terminal_command)
		terminal_command = DO_NOTHING;
}

static THD_FUNCTION(my_thread, arg)
{
	(void)arg;

	chRegSetThreadName("App SkyPUFF");

	is_running = true;

	// Main control loop
	for (loop_step = 0;; loop_step++)
	{
		// Check if it is time to stop app
		if (stop_now)
		{
			is_running = false;
			return;
		}

		int cur_tac = mc_interface_get_tachometer_value(false);
		int abs_tac = abs(cur_tac);

		// terminal command 'alive'?
		if (alive_inc)
		{
			alive_until = loop_step + alive_inc;
			alive_inc = 0;
			send_alive(cur_tac);
		}

		// Communication timeout?
		// BRAKING or MANUAL_BRAKING possible on timeout
		if (loop_step >= alive_until && abs_tac > config.braking_length &&
			state != UNINITIALIZED && state != MANUAL_BRAKING)
			manual_brake(cur_tac);

		process_terminal_commands(&cur_tac, &abs_tac);
		process_states(cur_tac, abs_tac);

		// Time to adjust motor?
		if (loop_step >= next_smooth_motor_adjustment)
			smooth_motor_adjustment(cur_tac, abs_tac);

		print_stats_periodically();

		chThdSleepMilliseconds(1);
	}
}

// Terminal command to change tachometer value
// NOT SAFE
/*
static void terminal_move_tac(int argc, const char **argv)
{
	if (argc == 2)
	{
		float d = 0;
		if (sscanf(argv[1], "%f", &d) == EOF)
		{
			commands_printf("move_tac: can't parse meters: '%s' value.", argv[1]);
			return;
		};

		int steps = meters_to_tac_steps(d);

		int cur_tac = mc_interface_get_tachometer_value(false);

		int new_tac = cur_tac + steps;

		commands_printf("move_tac: moving zero %.2fm (%d steps) %s, cur pos: %.2fm (%d steps), new pos: %.2fm (%d steps)",
						(double)d, steps, d < 0 ? "backward" : "forward",
						(double)tac_steps_to_meters(cur_tac), cur_tac,
						(double)tac_steps_to_meters(new_tac), new_tac);

		mc_interface_set_tachometer_value(new_tac);
	}
	else
	{
		commands_printf("This command requires one argument: 'move_tac -5.2' will move zero 5.2 meters backward");
	}
}
*/

static void terminal_set_zero(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	terminal_command = SET_ZERO;
}

static void terminal_print_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	terminal_command = PRINT_CONF;
}

static void terminal_get_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	terminal_command = SEND_CONF;
}

static void terminal_set_example_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	set_example_conf(&set_config);
	terminal_command = SET_CONF;
}

static void terminal_alive(int argc, const char **argv)
{
	if (argc < 2)
	{
		commands_printf("%s: -- Command requires one argument -- 'alive 300' will let skypuff work 300ms before timeout and MANUAL_BRAKING");
		return;
	}

	int t = 0;
	if (sscanf(argv[1], "%d", &t) == EOF)
	{
		commands_printf("%s: -- Can't parse '%s' as milliseconds value.",
						state_str(state), argv[1]);
		return;
	};

	alive_inc = t;
}

static void terminal_set_pull_force(int argc, const char **argv)
{
	if (argc < 2)
	{
		commands_printf("%s: -- Command requires one argument -- 'pull_force 100' will set pull force 100Kg");
		return;
	}

	float kg;
	if (sscanf(argv[1], "%f", &kg) == EOF)
	{
		commands_printf("%s: -- Can't parse '%s' as kg value.",
						state_str(state), argv[1]);
		return;
	};

	// Limits will be checked in process_terminal_commands()
	terminal_pull_kg = kg;
	terminal_command = SET_PULL_FORCE;
}

// Helper function to uppercase terminal commands
inline static void uppercase(char *out, const char *in, const int out_len)
{
	int in_len = strlen(in);

	// do not crash MCU occasionally
	if (out_len <= 0 || out_len > 1024)
		return;

	// Get minumum
	int valid_len = out_len - 1 < in_len ? out_len - 1 : in_len;

	// Iterate over the source string (i.e. s) and cast the case changing.
	for (int a = 0; a < valid_len; a++)
		out[a] = toupper(in[a]);

	// The last zero
	out[valid_len] = 0;
}

static void terminal_set_state(int argc, const char **argv)
{
	if (argc < 2)
	{
		commands_printf("%s: -- Command requires at least one argument -- For example: 'set_state UNWINDING'",
						state_str(state));
		return;
	}

	const int up_len = 64;
	char up[up_len];
	uppercase(up, argv[1], up_len);

	if (!strcmp(up, "UNWINDING"))
	{
		terminal_command = SET_UNWINDING;
		return;
	}
	else if (!strcmp(up, "MANUAL_BRAKING"))
	{
		terminal_command = SET_MANUAL_BRAKING;
		return;
	}
	else if (!strcmp(up, "BRAKING_EXTENSION"))
	{
		terminal_command = SET_BRAKING_EXTENSION;
		return;
	}
	else if (!strcmp(up, "MANUAL_SLOW"))
	{
		terminal_command = SET_MANUAL_SLOW;
		return;
	}
	else if (!strcmp(up, "MANUAL_SLOW_BACK"))
	{
		terminal_command = SET_MANUAL_SLOW_BACK;
		return;
	}
	else if (!strcmp(up, "PRE_PULL"))
	{
		terminal_command = SET_PRE_PULL;
		return;
	}
	else if (!strcmp(up, "TAKEOFF_PULL"))
	{
		terminal_command = SET_TAKEOFF_PULL;
		return;
	}
	else if (!strcmp(up, "PULL"))
	{
		terminal_command = SET_PULL;
		return;
	}
	else if (!strcmp(up, "FAST_PULL"))
	{
		terminal_command = SET_FAST_PULL;
		return;
	}
	else
	{
		commands_printf("%s: -- 'set %s' not implemented",
						state_str(state), argv[1]);
	}
}

#ifdef DEBUG_SMOOTH_MOTOR
static void terminal_smooth(int argc, const char **argv)
{
	if (argc < 2)
	{
		commands_printf("%s: -- Command requires at least one argument -- For example: 'smooth RELEASE'",
						state_str(state));
		return;
	}

	const int up_len = 64;
	char up[up_len];
	uppercase(up, argv[1], up_len);

	if (!strcmp(up, "RELEASE"))
	{
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_RELEASED;
		terminal_motor_state.param.current = 0;
		return;
	}
	else if (!strcmp(up, "BRAKE"))
	{
		if (argc < 3)
		{
			commands_printf("%s: -- smooth BRAKE requires current argument -- For example: 'smooth BRAKE 2'",
							state_str(state));
			return;
		}
		float current;
		if (sscanf(argv[2], "%f", &current) == EOF)
		{
			commands_printf("%s: -- Can't parse '%s' as current value.",
							state_str(state), argv[2]);
			return;
		};
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_BRAKING;
		terminal_motor_state.param.current = current;
		return;
	}
	else if (!strcmp(up, "CURRENT"))
	{
		if (argc < 3)
		{
			commands_printf("%s: -- smooth CURRENT requires current argument -- For example: 'smooth CURRENT 2'",
							state_str(state));
			return;
		}
		float current;
		if (sscanf(argv[2], "%f", &current) == EOF)
		{
			commands_printf("%s: -- Can't parse '%s' as current value.",
							state_str(state), argv[2]);
			return;
		};
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_CURRENT;
		terminal_motor_state.param.current = current;
		return;
	}
	else if (!strcmp(up, "SPEED"))
	{
		if (argc < 3)
		{
			commands_printf("%s: -- smooth SPEED requires erpm argument -- For example: 'smooth SPEED 1000'",
							state_str(state));
			return;
		}
		float erpm;
		if (sscanf(argv[2], "%f", &erpm) == EOF)
		{
			commands_printf("%s: -- Can't parse '%s' as current value.",
							state_str(state), argv[2]);
			return;
		};
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_SPEED;
		terminal_motor_state.param.erpm = erpm;
		return;
	}
	else
	{
		commands_printf("%s: -- 'smooth %s' not implemented",
						state_str(state), argv[1]);
	}
}
#endif