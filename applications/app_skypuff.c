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
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

//#define DEBUG_SMOOTH_MOTOR
//#define DEBUG_ANTISEX
//#define VERBOSE_TERMINAL

#include "app_skypuff.h"

/*
	This application turns VESC into paragliding winch controller.
	Some progress here: https://www.youtube.com/watch?v=KoNegc4SzxY

	To play with model winch, send terminal commands 'example_conf' 
	and 'alive 3000000' to set long communication timeout.

	Skypuff can't use VESC timeout because of smooth pull release mechanism.

	From 1 March 2020 Skypuff UI use binary protocol only. I tried to minimize packet sizes due to slow radio links.

	From 6 November 2020 protocol will be changed to request -> reply scheme
	due to half-duplex nature of the radio links.

	Asynchronous events will be queued to the buffer and sent with reply only.

	All terminal output will be moved under VERBOSE_TERMINAL define. 
	Only critical logic/hardware errors will be printed to terminal imediatelly.

	From 9 October 2021 'pull in' direction should have positive current, 'pull out' negative.
*/

const int short_print_delay = 500; // 0.5s, measured in control loop counts
const int long_print_delay = 3000;
const int smooth_max_step_delay = 100;
const int strong_unwinding_period = 2000; // 2 secs of strong unwinding current after entering unwinding

const char *limits_wrn = "-- CONFIGURATION IS OUT OF LIMITS --";

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void terminal_set_zero(int argc, const char **argv);
static void terminal_move_zero(int argc, const char **argv);
static void terminal_print_conf(int argc, const char **argv);
static void terminal_get_conf(int argc, const char **argv); // Send serialized conf with COMM_CUSTOM_APP_DATA
static void terminal_set_example_conf(int argc, const char **argv);
static void terminal_alive(int argc, const char **argv);
static void terminal_set_state(int argc, const char **argv);
static void terminal_set_pull_force(int argc, const char **argv);
static void terminal_adc2_tick(int argc, const char **argv);
static void terminal_measure_spool(int argc, const char **argv);
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
static int prev_printed_tac;			 // Do not print the same position
static mc_fault_code prev_printed_fault; // Do not print (send) the same fault many times
static float v_in_filtered;				 // Average for v_in
static float erpm_filtered;				 // For speed up states
static int alive_until;					 // In good communication we trust until (i < alive_until)
static int state_start_time;			 // Count the duration of state
static float terminal_pull_kg;			 // Pulling force to set
static volatile int alive_inc;			 // Communication timeout increment from terminal thread
static int unwinding_start_step;         // loop_step when entering UNWINDING

// Prevent long time oscilations
// Long rope in the air works like a big spring
// and cause long time oscilations together with paraglider
static float antisex_amps;	          // Force additive value to prevent oscilations
static float antisex_amps_gain;       // Force gain
static systime_t antisex_start_time;  // Point in time when pull reduce activated

// Speed and acceleration maesurements based on tachometer value is more accurate then mc_interface_get_rpm()
static systime_t measurement_delay = MS2ST(5); // Measure speed based on tachometer value
static float measurement_speed_filter_period_secs = (float)50.0 / (float)1000.0; // Period of tachometer speed measurements to overage

// Store tachometer based speed measurements here
struct {
	systime_t time;

	int tac;
	float speed_tac_ms;
	float acceleration_tac_mss;

	float filtered_speed_tac_ms;
	float filtered_acceleration_tac_mss;

	bool updated;
} measurement;

// Missing ChibiOS time types
typedef systime_t sysinterval_t;

static volatile skypuff_state state; // Readable from commands threads too
static skypuff_config config;
static skypuff_config set_config; // Updates from terminal thread
static skypuff_drive set_drive;	  // Update mc_configuration additional drive settings

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
	SEND_POWER_STATS,
	SEND_TEMP_STATS,
	SET_SMOOTH,
} skypuff_terminal_command;

static volatile skypuff_terminal_command terminal_command;

// This struct is not for serialization
typedef struct
{
	smooth_motor_mode mode;
	union
	{
		float current;
		float erpm;
	} param;
} smooth_motor_state;

static smooth_motor_state current_motor_state;
static smooth_motor_state target_motor_state;
static smooth_motor_state terminal_motor_state;

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
	.braking_applying_period = 1, // 0.001 secs
	.rope_length = 5,
	.braking_length = 5,
	.braking_extension_length = 3, // To create trigger length after unwinding
	.slowing_length = 3,
	.slow_erpm = 100,
	.rewinding_trigger_length = 10,
	.unwinding_trigger_length = 3,
	.pull_current = 0.5,
	.pre_pull_k = 0.1,
	.takeoff_pull_k = 0.3,
	.fast_pull_k = 1.05,
	.takeoff_trigger_length = 3,
	.pre_pull_timeout = 100, // 0.1 secs
	.takeoff_period = 100,
	.brake_current = 0.5,
	.slowing_current = 0,
	.manual_brake_current = 0.5,
	.unwinding_current = 0.5,
	.unwinding_strong_current = 0.6,
	.unwinding_strong_erpm = -50000,
	.rewinding_current = 0.5,
	.slow_max_current = 0.5,
	.manual_slow_max_current = 0.5,
	.manual_slow_speed_up_current = 0.5,
	.manual_slow_erpm = 100,
	.antisex_min_pull_amps = 0.2,
	.antisex_reduce_amps = 0.2,
	.antisex_acceleration_on_mss = -50,
	.antisex_acceleration_off_mss = -50,
	.antisex_max_period_ms = 100,
	.max_speed_ms = 5,
};

static const skypuff_config max_config = {
	.amps_per_kg = 30,
	.pull_applying_period = 10000,   // 10 secs
	.braking_applying_period = 2000, // 2 secs
	.rope_length = 5000 * 120,	     // 120 - maximum motor poles * 3
	.braking_length = 100 * 120,
	.braking_extension_length = 5000 * 120,
	.slowing_length = 100 * 120,
	.slow_erpm = 30000,
	.rewinding_trigger_length = 5000 * 120, // Could be disabled with large trigger
	.unwinding_trigger_length = 10 * 120,
	.pull_current = 600, // Believe in gliders
	.pre_pull_k = 0.5,
	.takeoff_pull_k = 0.8,
	.fast_pull_k = 1.5,
	.takeoff_trigger_length = 5000 * 120,
	.pre_pull_timeout = 5000,	// 5 secs
	.takeoff_period = 60000,	// 1 min
	.brake_current = 500,		// Charge battery mode possible
	.slowing_current = 30,		// Do not brake hardly on high unwinding speeds
	.manual_brake_current = 30, // Do not kill pilot in passive winch mode
	.unwinding_current = 50,
	.unwinding_strong_current = 50,
	.unwinding_strong_erpm = 50000,
	.rewinding_current = 100,
	.slow_max_current = 50,
	.manual_slow_max_current = 50,
	.manual_slow_speed_up_current = 50,
	.manual_slow_erpm = 40000,
	.antisex_min_pull_amps = 1000,
	.antisex_reduce_amps = 100,
	.antisex_acceleration_on_mss = 50,
	.antisex_acceleration_off_mss = 50,
	.antisex_max_period_ms = 60000,
	.max_speed_ms = 300,
};

// Convert units
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

	return rpm * ((float)mc_conf->si_motor_poles / 2);
}

inline static float erpm_to_ms(float erpm)
{
	float erps = erpm / 60;
	float rps = erps / ((float)mc_conf->si_motor_poles / 2);

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

	mc_interface_set_current(target_motor_state.param.current * antisex_amps_gain + antisex_amps);

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

inline static void smooth_calculate_new_speed(float applying_period)
{
	amps_per_sec = config.pull_current / (applying_period / (float)1000.0);
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

			mc_interface_set_current(step_current * antisex_amps_gain + antisex_amps);
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

			mc_interface_set_current(step_current * antisex_amps_gain + antisex_amps);
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

	smooth_calculate_new_speed(config.braking_applying_period);
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

	smooth_calculate_new_speed(config.pull_applying_period);
	target_motor_state.mode = MOTOR_CURRENT;
	target_motor_state.param.current = current;

	smooth_motor_adjustment(cur_tac, abs_tac);
}

// Helper functions to check limits
inline static void send_out_of_limits(const char *format, ...)
{
	static const int out_of_limits_max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	static uint8_t out_of_limits_buf[PACKET_MAX_PL_LEN - 1];
	int32_t ind = 0;
	out_of_limits_buf[ind++] = SK_COMM_OUT_OF_LIMITS;

	va_list args;
	va_start(args, format);
	int32_t printed = vsnprintf((char *)(out_of_limits_buf + ind), out_of_limits_max_buf_size - ind, format, args);
	va_end(args);

	commands_send_app_data(out_of_limits_buf, ind + printed); // Not necessary to send last NULL

#ifdef VERBOSE_TERMINAL
	commands_printf("%s: %s %s", state_str(state), limits_wrn, (char *)(out_of_limits_buf + ind));
#endif
}

inline static bool is_int_out_of_limits(const char *name, const char *units,
										const int val, const int min, const int max)
{
	if (val >= min && val <= max)
		return false;

	send_out_of_limits("%s %d %s is out of limits [%d, %d]",
					   name, val, units, min, max);

	return true;
}

inline static bool is_float_out_of_limits(const char *name, const char *units,
										  const float val, const float min, const float max)
{
	if (val >= min && val <= max)
		return false;

	send_out_of_limits("%s %.5f %s is out of limits [%.5f, %.5f]",
					   name, (double)val, units, (double)min, (double)max);
	return true;
}

inline static bool is_pull_out_of_limits(const char *name,
										 const float amps, const float min, const float max)
{
	if (amps >= min && amps <= max)
		return false;

	send_out_of_limits("%s %.1fA (%.2fKg) is out of limits [%.1fA (%.2fKg), %.1fA (%.2fKg)]",
					   name,
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

	send_out_of_limits("%s %d steps (%.2f meters) is out of limits [%d (%.2fm), %d (%.2fm)]",
					   name,
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

	send_out_of_limits("%s %.1f ERPM (%.1f m/s) is out of limits [%.1f (%.1f m/s), %.1f (%.1f m/s)]",
					   name,
					   (double)erpm, (double)erpm_to_ms(erpm),
					   (double)min, (double)erpm_to_ms(min),
					   (double)max, (double)erpm_to_ms(max));
	return true;
}

static bool is_drive_config_out_of_limits(const skypuff_drive *drv)
{
	if (drv->motor_poles % 2)
	{
		send_out_of_limits("motor_poles: %dp must be odd", drv->motor_poles);
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
		   is_int_out_of_limits("braking_applying_period", "milliseconds", conf->braking_applying_period,
								min_config.braking_applying_period, max_config.braking_applying_period) ||
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
		   is_speed_out_of_limits("unwinding_strong_erpm", conf->unwinding_strong_erpm,
								  min_config.unwinding_strong_erpm, max_config.unwinding_strong_erpm) ||
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
		   is_pull_out_of_limits("unwinding_strong_current", conf->unwinding_strong_current,
								 min_config.unwinding_strong_current, max_config.unwinding_strong_current) ||
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
								min_config.takeoff_period, max_config.takeoff_period) ||
		   is_pull_out_of_limits("antisex_min_pull_amps", conf->antisex_min_pull_amps,
								 min_config.antisex_min_pull_amps, max_config.antisex_min_pull_amps) ||
		   is_pull_out_of_limits("antisex_reduce_amps", conf->antisex_reduce_amps,
								 min_config.antisex_reduce_amps, max_config.antisex_reduce_amps) ||
		   is_float_out_of_limits("antisex_acceleration_on_mss", "m/sec^2", conf->antisex_acceleration_on_mss,
								  min_config.antisex_acceleration_on_mss, max_config.antisex_acceleration_on_mss) ||
		   is_float_out_of_limits("antisex_acceleration_off_mss", "m/sec^2", conf->antisex_acceleration_off_mss,
								  min_config.antisex_acceleration_off_mss, max_config.antisex_acceleration_off_mss) ||
		   is_int_out_of_limits("antisex_max_period_ms", "milliseconds", conf->antisex_max_period_ms,
								min_config.antisex_max_period_ms, max_config.antisex_max_period_ms) ||
		   is_float_out_of_limits("max_speed_ms", "ms", conf->max_speed_ms,
								  min_config.max_speed_ms, max_config.max_speed_ms);
}

// EEPROM
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

// Send new state to UI
inline static void send_state(const skypuff_state s)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = SK_COMM_STATE;
	buffer[ind++] = s;

	commands_send_app_data(buffer, ind);
}

inline static void send_pulling_too_high(const float current)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = SK_COMM_PULLING_TOO_HIGH;
	buffer_append_float16(buffer, current, 1e1, &ind);

	commands_send_app_data(buffer, ind);
}

inline static void send_force_is_set(void)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = SK_COMM_FORCE_IS_SET;
	buffer_append_float16(buffer, config.pull_current, 1e1, &ind);
	buffer_append_float16(buffer, amps_per_sec, 1e1, &ind);

	commands_send_app_data(buffer, ind);
}

// Used to debug antisex first time
inline static void send_custom_msg(const char *msg)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = SK_COMM_MSG;
	int msg_len = strlen(msg);
	if (max_buf_size - 1 >= msg_len)
	{
		memcpy(buffer + ind, msg, msg_len);
		ind += msg_len;
	}

	commands_send_app_data(buffer, ind);
}

inline static void send_command_only(skypuff_custom_app_data_command c)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = c;

	commands_send_app_data(buffer, ind);
}

// State setters
inline static void brake_state(const int cur_tac, const skypuff_state new_state, const float current,
							   const char *additional_msg)
{
	// Braking could be applied in the braking states to renew control timeout
	if (state != new_state)
	{
#ifdef VERBOSE_TERMINAL
		float erpm = mc_interface_get_rpm();
		prev_print = loop_step;
		prev_printed_tac = cur_tac;
		commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), braking %.1fKg (%.1fA)%s",
						state_str(new_state),
						(double)tac_steps_to_meters(cur_tac), cur_tac,
						(double)erpm_to_ms(erpm), (double)erpm,
						(double)(current / config.amps_per_kg), (double)current,
						additional_msg);
#else
		(void)additional_msg;
#endif
		state = new_state;
		send_state(state);
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

inline static void pull_state(const int cur_tac, const float current, const skypuff_state new_state,
							  const char *additional_msg)
{
	// Updates of pulling force sets this state again, do not change state_start_time
	if (state != new_state)
	{
		state_start_time = loop_step;
		state = new_state;
		send_state(state);
	}

	prev_abs_tac = abs(cur_tac);
	prev_erpm = mc_interface_get_rpm();
#ifdef VERBOSE_TERMINAL
	prev_print = loop_step;
	prev_printed_tac = cur_tac;
	commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)%s",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)erpm_to_ms(prev_erpm), (double)prev_erpm,
					(double)(current / config.amps_per_kg), (double)current,
					additional_msg);
#else
	(void)additional_msg;
#endif

	smooth_motor_current(cur_tac, prev_abs_tac, current);
	timeout_reset();
}

inline static float unwinding_current(void)
{
	if(measurement.filtered_speed_tac_ms > erpm_to_ms(config.unwinding_strong_erpm) ||
	  loop_step - unwinding_start_step < strong_unwinding_period)
		return config.unwinding_strong_current;
	else
		return config.unwinding_current;
}

inline static void unwinding(const int cur_tac)
{
	if(state != UNWINDING)
		unwinding_start_step = loop_step;

	pull_state(cur_tac, unwinding_current(), UNWINDING, "");
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
	// Positive current - positive ERPM, tachometer value goes up
	erpm_filtered = config.manual_slow_erpm;
	
	pull_state(cur_tac, config.manual_slow_speed_up_current, MANUAL_SLOW_SPEED_UP, msg);
}

inline static void manual_slow_back_speed_up(const int cur_tac)
{
	char msg[64];
	snprintf(msg, 64, ", until: %.1fms (%.0f ERPM)",
			 (double)erpm_to_ms(config.manual_slow_erpm), (double)config.manual_slow_erpm);

	// Set high filtered value on entering speed up mode
	// Negative current - negative ERPM, tachometer value goes down
	erpm_filtered = -config.manual_slow_erpm;
	
	pull_state(cur_tac, -config.manual_slow_speed_up_current, MANUAL_SLOW_BACK_SPEED_UP, msg);
}

inline static void slowing(const int cur_tac, const float erpm)
{
	state = SLOWING;
	send_state(state);

	prev_print = loop_step;
	prev_printed_tac = cur_tac;
#ifdef VERBOSE_TERMINAL
	commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), until %.1fms (%.0f ERPM)",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)erpm_to_ms(erpm), (double)erpm,
					(double)erpm_to_ms(config.slow_erpm), (double)config.slow_erpm);
#else
	(void)erpm;
#endif
	// Brake or release
	if (config.slowing_current > 0.1)
		smooth_motor_brake(cur_tac, fabs(cur_tac), config.slowing_current);
	else
		smooth_motor_release();
}

inline static void speed_state(const int cur_tac, const float cur_erpm,
							  const float to_zero_constant_erpm, const skypuff_state new_state)
{
	state = new_state;
	send_state(state);

	prev_print = loop_step;
	prev_printed_tac = cur_tac;
#ifdef VERBOSE_TERMINAL
	commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), constant speed %.1fms (%.0f ERPM)",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
					(double)erpm_to_ms(to_zero_constant_erpm), (double)to_zero_constant_erpm);
#endif
	prev_erpm = cur_erpm;
	smooth_motor_speed(to_zero_constant_erpm);
	timeout_reset();
}

inline static void slow(const int cur_tac, const float cur_erpm)
{
	speed_state(cur_tac, cur_erpm, config.slow_erpm, SLOW);
}

inline static void manual_slow(const int cur_tac, const float cur_erpm)
{
	speed_state(cur_tac, cur_erpm, config.manual_slow_erpm, MANUAL_SLOW);
}

inline static void manual_slow_back(const int cur_tac, const float cur_erpm)
{
	speed_state(cur_tac, cur_erpm, -config.manual_slow_erpm, MANUAL_SLOW_BACK);
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
	cfg->unwinding_strong_erpm = ms_to_erpm(-1);

	// Forces
	cfg->amps_per_kg = 3;			  // 7 Amps for 1Kg force
	cfg->pull_applying_period = 1500; // 1.5 secs
	cfg->braking_applying_period = 500; // 0.5 secs
	cfg->brake_current = 0.3 * cfg->amps_per_kg;
	cfg->slowing_current = 0.5 * cfg->amps_per_kg;
	cfg->manual_brake_current = 1 * cfg->amps_per_kg;
	cfg->unwinding_current = 0.4 * cfg->amps_per_kg;
	cfg->unwinding_strong_current = 0.6 * cfg->amps_per_kg;
	cfg->rewinding_current = 0.6 * cfg->amps_per_kg;
	cfg->slow_max_current = 1 * cfg->amps_per_kg;
	cfg->manual_slow_max_current = 1 * cfg->amps_per_kg;
	cfg->manual_slow_speed_up_current = 0.4 * cfg->amps_per_kg;

	// Pull settings
	cfg->pull_current = 3 * cfg->amps_per_kg;
	cfg->pre_pull_k = 30 / 100.0;
	cfg->takeoff_pull_k = 60 / 100.0;
	cfg->fast_pull_k = 120 / 100.0;
	cfg->takeoff_trigger_length = meters_to_tac_steps(0.1);
	cfg->pre_pull_timeout = 2 * 1000;
	cfg->takeoff_period = 5 * 1000;

	// Antisex
	cfg->antisex_min_pull_amps = 0.7 * cfg->amps_per_kg;
	cfg->antisex_reduce_amps = 0.5 * cfg->amps_per_kg;
	cfg->antisex_acceleration_on_mss = 7;
	cfg->antisex_acceleration_off_mss = -3;
	cfg->antisex_max_period_ms = 2000;

	// Speed scale limit
	cfg->max_speed_ms = 20;
}

static void set_example_drive(skypuff_drive *drv)
{
	drv->motor_poles = 14;
	drv->wheel_diameter = 0.05;
	drv->gear_ratio = 1;

	drv->battery_type = 255; // Do not change voltage limits
	drv->battery_cells = mc_conf->si_battery_cells;
}

// Serialization/deserialization functions
inline static void serialize_drive(uint8_t *buffer, int32_t *ind)
{
	buffer[(*ind)++] = (uint8_t)mc_conf->si_motor_poles;
	buffer_append_float32_auto(buffer, mc_conf->si_gear_ratio, ind);
	buffer_append_float32_auto(buffer, mc_conf->si_wheel_diameter, ind);
	buffer[(*ind)++] = (uint8_t)mc_conf->si_battery_type;
	buffer[(*ind)++] = (uint8_t)mc_conf->si_battery_cells;
}

inline static bool deserialize_drive(unsigned char *data, unsigned int len, skypuff_drive *to, int32_t *ind)
{
	const int32_t serialized_drive_v1_length = 1 + 4 * 2 + 2;

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
	to->battery_type = data[(*ind)++];
	to->battery_cells = data[(*ind)++];
	return true;
}

inline static void serialize_config(uint8_t *buffer, int32_t *ind)
{
	buffer_append_float16(buffer, config.amps_per_kg, 1e2, ind);
	buffer_append_uint16(buffer, config.pull_applying_period, ind);
	buffer_append_int32(buffer, config.rope_length, ind);
	buffer_append_int32(buffer, config.braking_length, ind);
	buffer_append_int32(buffer, config.braking_extension_length, ind);

	buffer_append_int32(buffer, config.slowing_length, ind);
	buffer_append_uint16(buffer, config.slow_erpm, ind);
	buffer_append_int32(buffer, config.rewinding_trigger_length, ind);
	buffer_append_int32(buffer, config.unwinding_trigger_length, ind);
	buffer_append_float16(buffer, config.pull_current, 10, ind);

	buffer_append_float16(buffer, config.pre_pull_k, 1e4, ind);
	buffer_append_float16(buffer, config.takeoff_pull_k, 1e4, ind);
	buffer_append_float16(buffer, config.fast_pull_k, 1e4, ind);
	buffer_append_uint16(buffer, config.takeoff_trigger_length, ind);
	buffer_append_uint16(buffer, config.pre_pull_timeout, ind);

	buffer_append_uint16(buffer, config.takeoff_period, ind);
	buffer_append_float16(buffer, config.brake_current, 10, ind);
	buffer_append_float16(buffer, config.slowing_current, 10, ind);
	buffer_append_float16(buffer, config.manual_brake_current, 10, ind);
	buffer_append_float16(buffer, config.unwinding_current, 10, ind);

	buffer_append_float16(buffer, config.rewinding_current, 10, ind);
	buffer_append_float16(buffer, config.slow_max_current, 10, ind);
	buffer_append_float16(buffer, config.manual_slow_max_current, 10, ind);
	buffer_append_float16(buffer, config.manual_slow_speed_up_current, 10, ind);
	buffer_append_uint16(buffer, config.manual_slow_erpm, ind);

	buffer_append_float16(buffer, config.antisex_min_pull_amps, 10, ind);
	buffer_append_float16(buffer, config.antisex_reduce_amps, 10, ind);
	buffer_append_float16(buffer, config.antisex_acceleration_on_mss, 1e2, ind);
	buffer_append_float16(buffer, config.antisex_acceleration_off_mss, 1e2, ind);
	buffer_append_uint16(buffer, config.antisex_max_period_ms, ind);

	buffer_append_float16(buffer, config.max_speed_ms, 1e2, ind);
	buffer_append_uint16(buffer, config.braking_applying_period, ind);
	buffer_append_float16(buffer, config.unwinding_strong_current, 10, ind);
	buffer_append_int16(buffer, config.unwinding_strong_erpm, ind);

}

inline static bool deserialize_config(unsigned char *data, unsigned int len, skypuff_config *to, int32_t *ind)
{
	const int32_t serialized_settings_v1_length = 16 + 16 + 10 + 10 + 10 + 10 + 8;

	int available_bytes = len - *ind;
	if (available_bytes < serialized_settings_v1_length)
	{
		commands_printf("%s: -- Can't deserialize config settings -- Received %d bytes, expecting %d.",
						state_str(state), available_bytes, serialized_settings_v1_length);
		return false;
	}

	// 2 * 2 + 4 * 3 = 16 bytes
	to->amps_per_kg = buffer_get_float16(data, 1e2, ind);
	to->pull_applying_period = buffer_get_uint16(data, ind);
	to->rope_length = buffer_get_int32(data, ind);
	to->braking_length = buffer_get_int32(data, ind);
	to->braking_extension_length = buffer_get_int32(data, ind);

	// 4 * 3 + 2 * 2 = 16 bytes
	to->slowing_length = buffer_get_int32(data, ind);
	to->slow_erpm = buffer_get_uint16(data, ind);
	to->rewinding_trigger_length = buffer_get_int32(data, ind);
	to->unwinding_trigger_length = buffer_get_int32(data, ind);
	to->pull_current = buffer_get_float16(data, 10, ind);

	// 2 * 5 = 10 bytes
	to->pre_pull_k = buffer_get_float16(data, 1e4, ind);
	to->takeoff_pull_k = buffer_get_float16(data, 1e4, ind);
	to->fast_pull_k = buffer_get_float16(data, 1e4, ind);
	to->takeoff_trigger_length = buffer_get_uint16(data, ind); // short distance
	to->pre_pull_timeout = buffer_get_uint16(data, ind);

	// 2 * 5 = 10 bytes
	to->takeoff_period = buffer_get_uint16(data, ind);
	to->brake_current = buffer_get_float16(data, 10, ind);
	to->slowing_current = buffer_get_float16(data, 10, ind);
	to->manual_brake_current = buffer_get_float16(data, 10, ind);
	to->unwinding_current = buffer_get_float16(data, 10, ind);

	// 2 * 5 = 10 bytes
	to->rewinding_current = buffer_get_float16(data, 10, ind);
	to->slow_max_current = buffer_get_float16(data, 10, ind);
	to->manual_slow_max_current = buffer_get_float16(data, 10, ind);
	to->manual_slow_speed_up_current = buffer_get_float16(data, 10, ind);
	to->manual_slow_erpm = buffer_get_uint16(data, ind);

	// 2 * 5 = 10 bytes
	to->antisex_min_pull_amps = buffer_get_float16(data, 10, ind);
	to->antisex_reduce_amps = buffer_get_float16(data, 10, ind);
	to->antisex_acceleration_on_mss = buffer_get_float16(data, 1e2, ind);
	to->antisex_acceleration_off_mss = buffer_get_float16(data, 1e2, ind);
	to->antisex_max_period_ms = buffer_get_uint16(data, ind);

	// 2 * 4 = 8 bytes
	to->max_speed_ms = buffer_get_float16(data, 1e2, ind);
	to->braking_applying_period = buffer_get_uint16(data, ind);
	to->unwinding_strong_current = buffer_get_float16(data, 10, ind);
	to->unwinding_strong_erpm = buffer_get_int16(data, ind);

	return true;
}

inline static void get_stats(float *erpm, float *motor_amps, float *battery_amps)
{
	*erpm = mc_interface_get_rpm();
	*motor_amps = mc_interface_read_reset_avg_motor_current();
	*battery_amps = mc_interface_read_reset_avg_input_current();
}

inline static void serialize_power_stats(uint8_t *buffer, int32_t *ind, const int cur_tac)
{
	float erpm;
	float motor_amps;
	float battery_amps;
	get_stats(&erpm, &motor_amps, &battery_amps);

	// Try to minimize packet size for slow channels
	//buffer[(*ind)++] = current_motor_state.mode;
	buffer_append_int32(buffer, cur_tac, ind);
	buffer_append_int16(buffer, erpm / 4, ind); // About +-120k ERPM max
	buffer_append_float16(buffer, motor_amps, 1e1, ind);
	buffer_append_float32(buffer, battery_amps, 1e3, ind);
}

inline static void serialize_temp_stats(uint8_t *buffer, int32_t *ind)
{
	float fets_temp = mc_interface_temp_fet_filtered();
	float motor_temp = mc_interface_temp_motor_filtered();

	buffer_append_float16(buffer, v_in_filtered, 1e1, ind);
	buffer_append_float16(buffer, fets_temp, 1e1, ind);
	buffer_append_float16(buffer, motor_temp, 1e1, ind);
}

inline static void serialize_scales(uint8_t *buffer, int32_t *ind)
{
	float max_force = fabs(mc_conf->lo_current_max) > fabs(mc_conf->lo_current_min) ? fabs(mc_conf->lo_current_max) : fabs(mc_conf->lo_current_min);
	float max_discharge = mc_conf->lo_current_max < mc_conf->lo_in_current_max ? mc_conf->lo_current_max : mc_conf->lo_in_current_max;
	// Min (discharge) current is negative
	float max_charge = mc_conf->lo_current_min > mc_conf->lo_in_current_min ? mc_conf->lo_current_min : mc_conf->lo_in_current_min;

	// currents
	buffer_append_float16(buffer, max_force, 1e1, ind);
	buffer_append_float16(buffer, max_discharge, 1e1, ind);
	buffer_append_float16(buffer, max_charge, 1e1, ind);
	// temps
	buffer_append_float16(buffer, mc_conf->l_temp_fet_start, 1e1, ind);
	buffer_append_float16(buffer, mc_conf->l_temp_motor_start, 1e1, ind);
	// battery voltage
	buffer_append_float32(buffer, fmax(mc_conf->l_min_vin, mc_conf->l_battery_cut_start), 1e2, ind);
	buffer_append_float32(buffer, mc_conf->l_max_vin, 1e2, ind);
}

inline static bool deserialize_alive(unsigned char *data, unsigned int len, int32_t *ind)
{
	const int32_t serialized_alive_length = 2;

	int available_bytes = len - *ind;
	if (available_bytes < serialized_alive_length)
	{
		commands_printf("%s: -- Can't deserialize alive command -- Received %d bytes, expecting %d.",
						state_str(state), available_bytes, serialized_alive_length);
		return false;
	}

	alive_inc = buffer_get_uint16(data, ind);
	return true;
}

inline static void send_conf(void)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = SK_COMM_SETTINGS_V1;
	buffer[ind++] = state;
	buffer[ind++] = mc_interface_get_fault();

	serialize_scales(buffer, &ind);
	serialize_drive(buffer, &ind);
	serialize_config(buffer, &ind);

	if (ind > max_buf_size)
	{
		commands_printf("%s: -- ALARMA!!! -- send_conf() max buffer size %d, serialized bufer %d bytes. Memory corrupted!",
						state_str(state), max_buf_size, ind);
	}

	commands_send_app_data(buffer, ind);
}

// Send fault to UI
inline static void send_fault(const mc_fault_code f)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = SK_COMM_FAULT;
	buffer[ind++] = f;

	commands_send_app_data(buffer, ind);
}

// Send motor mode, speed and current amps as reply to alive command
inline static void send_stats(const int cur_tac, bool add_temps)
{
	const int max_buf_size = PACKET_MAX_PL_LEN - 1; // 1 byte for COMM_CUSTOM_APP_DATA
	uint8_t buffer[max_buf_size];
	int32_t ind = 0;

	buffer[ind++] = add_temps ? SK_COMM_ALIVE_TEMP_STATS : SK_COMM_ALIVE_POWER_STATS;

	serialize_power_stats(buffer, &ind, cur_tac);
	if (add_temps)
		serialize_temp_stats(buffer, &ind);

	if (ind > max_buf_size)
	{
		commands_printf("%s: -- ALARMA!!! -- send_stats() max buffer size %d, serialized bufer %d bytes. Memory corrupted!",
						state_str(state), max_buf_size, ind);
	}

	commands_send_app_data(buffer, ind);
}

void set_adc2_pushpull(void)
{
	palClearPad(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN);
	palSetPadMode(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN,
				  PAL_MODE_OUTPUT_PUSHPULL |
				  PAL_STM32_OSPEED_HIGHEST);

#ifdef VERBOSE_TERMINAL
	commands_printf("Skypuff - ADC_EXT2 GPIO set to PUSHPULL mode for guillotine switch");
#endif
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
	case SK_COMM_ALIVE_POWER_STATS:
		// Set alive_inc right there
		if (!deserialize_alive(data, len, &ind))
			return;

		terminal_command = SEND_POWER_STATS;
		break;
	case SK_COMM_ALIVE_TEMP_STATS:
		// Set alive_inc right there
		if (!deserialize_alive(data, len, &ind))
			return;

		terminal_command = SEND_TEMP_STATS;
		break;
	case SK_COMM_SETTINGS_V1:
		if (!deserialize_drive(data, len, &set_drive, &ind))
			return;
		if (!deserialize_config(data, len, &set_config, &ind))
			return;

		terminal_command = SET_CONF;
		break;
	case SK_COMM_GUILLOTINE:
		// Cut the rope!
		terminal_adc2_tick(0, 0);
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

// Speed measurements based on tachometer value
//
// Some ChibiOS missing time functions
static inline sysinterval_t chTimeDiffX(systime_t start, systime_t end) {
   return (sysinterval_t)((systime_t)(end - start));
}

static inline systime_t chTimeAddX(systime_t systime, sysinterval_t interval) {
     return systime + (systime_t)interval;
}

static void measurement_init(void)
{
	measurement.time = chVTGetSystemTime();
	measurement.tac = mc_interface_get_tachometer_value(false);

	measurement.speed_tac_ms = 0;
	measurement.acceleration_tac_mss = 0;

	measurement.filtered_speed_tac_ms = 0;
	measurement.filtered_acceleration_tac_mss = 0;

	measurement.updated = false;
}

// Returns current tachometer value each time
static inline bool measurement_tick(int cur_tac)
{
	// Get new values
	systime_t cur_time = chVTGetSystemTime();
	sysinterval_t delayI = chTimeDiffX(measurement.time, cur_time);

	if(delayI < measurement_delay) {
		measurement.updated = false;
		return false;
	}

	measurement.updated = true;

	float delayS = (float)delayI / (float)CH_CFG_ST_FREQUENCY;

	// Tachometer speed
	int tac_diff = cur_tac - measurement.tac;

	float cur_speed_tac_ms = tac_steps_to_meters(tac_diff) / delayS;

	float avgK = delayS / measurement_speed_filter_period_secs;
	float prev_filtered_speed_tac_ms = measurement.filtered_speed_tac_ms;
	UTILS_LP_FAST(measurement.filtered_speed_tac_ms, (cur_speed_tac_ms + measurement.speed_tac_ms) / 2, avgK);
	
	float cur_acceleration_tac_mss = (measurement.filtered_speed_tac_ms - prev_filtered_speed_tac_ms) / delayS;
	UTILS_LP_FAST(measurement.filtered_acceleration_tac_mss, (cur_acceleration_tac_mss + measurement.acceleration_tac_mss) / 2, avgK);

	measurement.time = cur_time;
	measurement.tac = cur_tac;
	measurement.speed_tac_ms = cur_speed_tac_ms;
	measurement.acceleration_tac_mss = cur_acceleration_tac_mss;

	return true;
}


static void antisex_init(void)
{
	antisex_amps = 0;
	antisex_amps_gain = 1;
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
	// Set guillotine switch to TTL output mode
	set_adc2_pushpull();

	// Enable WiFi / Radio controll
	app_uartcomm_start(UART_PORT_COMM_HEADER);

	// Reset tachometer on app start to prevent instant unwinding to zero
	mc_interface_get_tachometer_value(true);

	mc_conf = mc_interface_get_configuration();

	// Static variables initial state
	timeout_reset_interval = app_get_configuration()->timeout_msec / 2;
	prev_print = INT_MIN / 2;
	prev_printed_tac = INT_MIN / 2;
	prev_printed_fault = FAULT_CODE_NONE;
	alive_until = 0;
	prev_abs_tac = 0;
	prev_erpm = 0;
	v_in_filtered = GET_INPUT_VOLTAGE();
	terminal_command = DO_NOTHING;
	stop_now = false;

	smooth_motor_release();

	read_config_from_eeprom(&config);

	measurement_init();
	antisex_init();

	// Check system drive settings and our config for limits
	set_drive.motor_poles = mc_conf->si_motor_poles;
	set_drive.gear_ratio = mc_conf->si_gear_ratio;
	set_drive.wheel_diameter = mc_conf->si_wheel_diameter;
	set_drive.battery_type = mc_conf->si_battery_type;
	set_drive.battery_cells = mc_conf->si_battery_cells;

	state = is_drive_config_out_of_limits(&set_drive) || is_config_out_of_limits(&config) ? UNINITIALIZED : BRAKING;

	commands_set_app_data_handler(custom_app_data_handler);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
		"set_zero",
		"Move SkyPUFF zero point to this position",
		"", terminal_set_zero);
	terminal_register_command_callback(
		"move_zero",
		"Dangerous! Move SkyPUFF zero position to specified meters delta",
		"[meters]", terminal_move_zero);
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
	terminal_register_command_callback(
		"adc2_tick",
		"Cut the rope with guillotine",
		"", terminal_adc2_tick);
	terminal_register_command_callback(
		"measure_spool",
		"Measure spool virtual mass spinning motor backward and forward with specified force during specified time and measurments with specified interval",
		"<force (kg)> <apply time (seconds)> [0 / 1 - use smooth motor and antisex?]", terminal_measure_spool);

#ifdef DEBUG_SMOOTH_MOTOR
	terminal_register_command_callback(
		"smooth",
		"Debug smooth motor control.",
		"<release/brake/current/speed> [current/erpm]", terminal_smooth);
#endif

	// Run control loop thread
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa), NORMALPRIO, my_thread, NULL);

	commands_printf("app_skypuff started");
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
	commands_set_app_data_handler(NULL);
	terminal_unregister_callback(terminal_set_zero);
	terminal_unregister_callback(terminal_move_zero);
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

// Swithes to BRAKING / SLOWING if needed. Returns true if swithed
static bool brake_or_slowing(const int cur_tac)
{
	// We are in the braking range?
	if (cur_tac >= -config.braking_length)
	{
		braking(cur_tac);
		return true;
	}

	// We are in the slowing range?
	if (cur_tac >= -(config.braking_length + config.slowing_length))
	{
		float erpm = mc_interface_get_rpm();

		// Check direction and erpm to decide about slowing
		if (cur_tac < 0 && erpm > config.slow_erpm)
		{
			slowing(cur_tac, erpm);
			return true;
		}
	}

	return false;
}

inline static void update_stats_check_faults(void)
{
	UTILS_LP_FAST(v_in_filtered, GET_INPUT_VOLTAGE(), 0.1);
	mc_fault_code f = mc_interface_get_fault();

	// Will print new fault immediately
	if (f != prev_printed_fault)
	{
		prev_printed_fault = f;
		send_fault(f);
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
		send_command_only(SK_COMM_TOO_SLOW_SPEED_UP);
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

inline static bool is_unwinded_to_opposite_braking_zone(const int cur_tac, const float cur_erpm)
{
	if ((cur_erpm > 0 && cur_tac >= config.braking_length) ||
		(cur_erpm < 0 && cur_tac <= -config.braking_length))
	{
		send_command_only(SK_COMM_UNWINDED_TO_OPPOSITE);
#ifdef VERBOSE_TERMINAL
		commands_printf("%s: -- Unwinded to opposite braking zone", state_str(state));
#endif
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

			// Always warn if UNINITIALIZED
			print_position_periodically(cur_tac, long_print_delay, msg);
		}
		break;
	case BRAKING:
		// Moved to unwinding zone?
		if (cur_tac < -config.braking_length)
		{
			braking_extension(cur_tac);
			break;
		}
		// Moved to manual braking zone?
		if (cur_tac > config.braking_length)
		{
			manual_brake(cur_tac);
			break;
		}

		// Timeout thread will remove braking every second by default
		// Apply brake again if position changed
		if (timeout_has_timeout() && abs_tac != prev_abs_tac)
			braking(cur_tac);
		else
			prev_abs_tac = abs_tac;

#ifdef VERBOSE_TERMINAL
		print_position_periodically(cur_tac, long_print_delay, "");
#endif
		break;
	case BRAKING_EXTENSION:
		// Moved to unwinding zone?
		if (cur_tac < -(config.braking_length + config.braking_extension_length))
		{
			unwinding(cur_tac);
			break;
		}

		// Moved to braking zone?
		if (abs_tac <= config.braking_length) //todo is it legit to switch to braking_extension with positive tachometer?
		{
			braking(cur_tac);
		}

		// Timeout thread will remove braking every second by default
		// Apply brake again if position changed
		if (timeout_has_timeout() && abs_tac != prev_abs_tac)
			braking_extension(cur_tac);
		else
			prev_abs_tac = abs_tac;

#ifdef VERBOSE_TERMINAL
		print_position_periodically(cur_tac, long_print_delay, "");
#endif
		break;
	case MANUAL_BRAKING:
		// Apply brake again on timeout and position change
		if (timeout_has_timeout() && abs_tac != prev_abs_tac)
			manual_brake(cur_tac);
		else
			prev_abs_tac = abs_tac;

#ifdef VERBOSE_TERMINAL
		print_position_periodically(cur_tac, long_print_delay,
									loop_step >= alive_until ? ", -- Communication timeout" : "");
#endif

		break;
	case UNWINDING:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac))
			break;

		// Strong unwinding or normal?
		if(measurement.updated) {
			float amps = unwinding_current();
			if(target_motor_state.param.current != amps)
				pull_state(cur_tac, amps, UNWINDING, "");
		}

		// Use prev_abs_tac as max tachometer
		if (abs_tac > prev_abs_tac)
		{
			// Print debug message if we are going out from slowing zone
			int eof_slowing = config.braking_length + config.slowing_length;
			if (prev_abs_tac < eof_slowing && abs_tac >= eof_slowing)
			{
				send_command_only(SK_COMM_UNWINDED_FROM_SLOWING);
#ifdef VERBOSE_TERMINAL
				commands_printf("%s: -- Unwinded from slowing zone %.2fm (%d steps)",
								state_str(state),
								(double)tac_steps_to_meters(cur_tac), cur_tac);
#endif
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
#ifdef VERBOSE_TERMINAL
		print_position_periodically(cur_tac, long_print_delay, "");
#endif
		break;
	case REWINDING:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac))
			break;

		// Now use prev_abs_tac as min value
		if (abs_tac < prev_abs_tac)
			prev_abs_tac = abs_tac;

		// Unwinding again?
		if (abs_tac > prev_abs_tac + config.unwinding_trigger_length)
			unwinding(cur_tac);

#ifdef VERBOSE_TERMINAL
		print_position_periodically(cur_tac, long_print_delay, "");
#endif
		break;
	case SLOWING:
		// We are in the braking range?
		if (cur_tac >= -config.braking_length)
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

#ifdef VERBOSE_TERMINAL
		slowing_or_speed_up_print(cur_tac, cur_erpm, config.slow_erpm);
#endif
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
			send_pulling_too_high(abs_current);
#ifdef VERBOSE_TERMINAL
			commands_printf(
				"SLOW: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)tac_steps_to_meters(cur_tac), cur_tac,
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.amps_per_kg), (double)cur_current,
				(double)(config.slow_max_current / config.amps_per_kg), (double)config.slow_max_current);
#endif
			brake_or_unwinding(cur_tac, abs_tac);
			break;
		}

		// Slowly rewinded more then opposite side of braking  zone?
		if (is_unwinded_to_opposite_braking_zone(cur_tac, cur_erpm))
		{
			braking(cur_tac);
			break;
		}

#ifdef VERBOSE_TERMINAL
		if (loop_step - prev_print > long_print_delay)
		{
			prev_print = loop_step;
			commands_printf("SLOW: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.amps_per_kg), (double)cur_current);
		}
#endif
		break;
	case MANUAL_SLOW_SPEED_UP:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		cur_erpm = mc_interface_get_rpm();

		// Stopped long enough?
		if (is_filtered_speed_too_low(cur_erpm))
		{
			manual_brake(cur_tac);
			break;
		}

		// Fast enough for PID speed in zero direction?
		if (cur_erpm >= config.manual_slow_erpm)
		{
			manual_slow(cur_tac, cur_erpm);
			break;
		}

#ifdef VERBOSE_TERMINAL
		slowing_or_speed_up_print(cur_tac, cur_erpm, config.manual_slow_erpm);
#endif
		break;
	case MANUAL_SLOW_BACK_SPEED_UP:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		cur_erpm = mc_interface_get_rpm();

		// Stopped long enough?
		if (is_filtered_speed_too_low(cur_erpm))
		{
			manual_brake(cur_tac);
			break;
		}

		// Fast enough for PID speed in opposite from zero direction?
		if (cur_erpm <= -config.manual_slow_erpm)
		{
			manual_slow_back(cur_tac, cur_erpm);
			break;
		}

#ifdef VERBOSE_TERMINAL
		slowing_or_speed_up_print(cur_tac, cur_erpm, config.manual_slow_erpm);
#endif
		break;
	case MANUAL_SLOW:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		cur_current = mc_interface_get_tot_current_directional_filtered();
		abs_current = fabs(cur_current);
		cur_erpm = mc_interface_get_rpm();

		// If current is above the limits
		if (abs_current > config.manual_slow_max_current)
		{
			send_pulling_too_high(abs_current);
#ifdef VERBOSE_TERMINAL
			commands_printf(
				"MANUAL_SLOW: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)tac_steps_to_meters(cur_tac), cur_tac,
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.amps_per_kg), (double)cur_current,
				(double)(config.manual_slow_max_current / config.amps_per_kg), (double)config.manual_slow_max_current);
#endif
			brake_or_manual_brake(cur_tac, abs_tac);
			break;
		}

#ifdef VERBOSE_TERMINAL
		if (loop_step - prev_print > long_print_delay)
		{
			prev_print = loop_step;
			commands_printf("MANUAL_SLOW: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.amps_per_kg), (double)cur_current);
		}
#endif
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
			send_pulling_too_high(abs_current);
#ifdef VERBOSE_TERMINAL
			commands_printf(
				"MANUAL_SLOW_BACK: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)tac_steps_to_meters(cur_tac), cur_tac,
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.amps_per_kg), (double)cur_current,
				(double)(config.manual_slow_max_current / config.amps_per_kg), (double)config.manual_slow_max_current);
#endif
			brake_or_manual_brake(cur_tac, abs_tac);
			break;
		}

#ifdef VERBOSE_TERMINAL
		if (loop_step - prev_print > long_print_delay)
		{
			prev_print = loop_step;
			commands_printf("MANUAL_SLOW_BACK: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.amps_per_kg), (double)cur_current);
		}
#endif
		break;
	case PRE_PULL:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac))
			break;

		// Enough time to tight the rope?
		int timeout_step = state_start_time + config.pre_pull_timeout;
		if (loop_step == timeout_step)
		{
			send_command_only(SK_COMM_DETECTING_MOTION);
#ifdef VERBOSE_TERMINAL
			float erpm = mc_interface_get_rpm();
			commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Pre pull %.1fs timeout passed, saving position",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(erpm), (double)erpm,
							(double)config.pre_pull_timeout / (double)1000.0);
#endif
			prev_abs_tac = abs_tac;
		}

		// Timeout passed and moved enough to takeoff?
		if (loop_step > timeout_step && abs(prev_abs_tac - abs_tac) >= config.takeoff_trigger_length)
		{
#ifdef VERBOSE_TERMINAL
			float erpm = mc_interface_get_rpm();
			commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Motion %.2fm (%d steps) detected",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(erpm), (double)erpm,
							(double)tac_steps_to_meters(config.takeoff_trigger_length), config.takeoff_trigger_length);
#endif
			takeoff_pull(cur_tac);
			break;
		}

#ifdef VERBOSE_TERMINAL
		print_position_periodically(cur_tac, long_print_delay, "");
#endif
		break;
	case TAKEOFF_PULL:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac))
			break;

		// Enough time of weak takeoff pull?
		if (loop_step >= state_start_time + config.takeoff_period)
		{
#ifdef VERBOSE_TERMINAL
			float erpm = mc_interface_get_rpm();
			commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), -- Takeoff %.1fs timeout passed",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(erpm), (double)erpm,
							(double)config.takeoff_period / (double)1000.0);
#endif
			pull(cur_tac);
			break;
		}

#ifdef VERBOSE_TERMINAL
		print_position_periodically(cur_tac, long_print_delay, "");
#endif
		break;
	case PULL:
	case FAST_PULL:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		// Go braking or slowing?
		if (brake_or_slowing(cur_tac))
			break;

#ifdef VERBOSE_TERMINAL
		print_position_periodically(cur_tac, long_print_delay, "");
#endif
		break;
	case MANUAL_DEBUG_SMOOTH:
		// No system timeouts on this state
		if (!(loop_step % timeout_reset_interval))
			timeout_reset();

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	default:
		commands_printf("SkyPUFF: unknown control loop state, exiting!");
		stop_now = true;
	}
}

inline static void print_conf(const int cur_tac)
{
	float erpm;
	float motor_amps;
	float battery_amps;
	float power;
	get_stats(&erpm, &motor_amps, &battery_amps);
	power = v_in_filtered * battery_amps;

	commands_printf("VESC additional info:");
	commands_printf("  wheel diameter: %.2fmm", (double)(mc_conf->si_wheel_diameter * 1000));
	commands_printf("  motor poles: %dp", mc_conf->si_motor_poles);
	commands_printf("  gear ratio: %.5f", (double)mc_conf->si_gear_ratio);

	commands_printf("SkyPUFF configuration version %s:", sk_command_str(SK_COMM_SETTINGS_V1));
	commands_printf("  amperes per 1kg force: %.1fAKg", (double)config.amps_per_kg);
	commands_printf("  rope gauge max value: %.1fms", (double)config.max_speed_ms);
	commands_printf("  pull applying period: %.1fs (%d loops)", (double)config.pull_applying_period / (double)1000.0, config.pull_applying_period);
	commands_printf("  braking applying period: %.1fs (%d loops)", (double)config.braking_applying_period / (double)1000.0, config.braking_applying_period);
	commands_printf("  rope length: %.2fm (%d steps)", (double)tac_steps_to_meters(config.rope_length), config.rope_length);
	commands_printf("  braking range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.braking_length), config.braking_length);
	commands_printf("  braking extension range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.braking_extension_length), config.braking_extension_length);
	commands_printf("  slowing range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.slowing_length), config.slowing_length);
	commands_printf("  rewinding trigger range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.rewinding_trigger_length), config.rewinding_trigger_length);
	commands_printf("  unwinding trigger range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.unwinding_trigger_length), config.unwinding_trigger_length);
	commands_printf("  brake force: %.2fkg (%.1fA)", (double)(config.brake_current / config.amps_per_kg), (double)config.brake_current);
	commands_printf("  manual brake force: %.2fkg (%.1fA)", (double)(config.manual_brake_current / config.amps_per_kg), (double)config.manual_brake_current);
	commands_printf("  unwinding force: %.2fkg (%.1fA)", (double)(config.unwinding_current / config.amps_per_kg), (double)config.unwinding_current);
	commands_printf("  unwinding strong force: %.2fkg (%.1fA)", (double)(config.unwinding_strong_current / config.amps_per_kg), (double)config.unwinding_strong_current);
	commands_printf("  unwinding strong speed: %.1fms (%.0f ERPM)", (double)erpm_to_ms(config.unwinding_strong_erpm), (double)config.unwinding_strong_erpm);
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

	commands_printf("  antisex activation min pull: %.2fkg (%.1fA)", (double)(config.antisex_min_pull_amps / config.amps_per_kg), (double)config.antisex_min_pull_amps);
	commands_printf("  antisex reducing force: %.2fkg (%.1fA)", (double)(config.antisex_reduce_amps / config.amps_per_kg), (double)config.antisex_reduce_amps);
	commands_printf("  antisex acceleration on: %.2fms/s^2", (double)config.antisex_acceleration_on_mss);
	commands_printf("  antisex acceleration off: %.2fms/s^2", (double)config.antisex_acceleration_off_mss);
	commands_printf("  antisex max applying period: %.1fs", (double)config.antisex_max_period_ms / (double)1000.0);

	commands_printf("SkyPUFF state:");
	commands_printf("  %s: pos %.2fm (%d steps), speed %.1fm/s (%.1f ERPM)", state_str(state), (double)tac_steps_to_meters(cur_tac), cur_tac, (double)erpm_to_ms(erpm), (double)erpm);
	commands_printf("  motor state %s: %.2fkg (%.1fA), battery: %.1fA %.1fV, power: %.1fW", motor_mode_str(current_motor_state.mode), (double)(motor_amps / config.amps_per_kg), (double)motor_amps, (double)battery_amps, (double)v_in_filtered, (double)power);
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
			mc_interface_get_tachometer_value(true);
			prev_abs_tac = 0;
			prev_printed_tac = 0;
			*cur_tac = 0;
			*abs_tac = 0;
			send_command_only(SK_COMM_ZERO_IS_SET);
#ifdef VERBOSE_TERMINAL
			float erpm = mc_interface_get_rpm();
			commands_printf("%s: pos %.2fm (%d steps), speed %.1fms (%0.f ERPM), -- Zero is set",
							state_str(state),
							(double)tac_steps_to_meters(*cur_tac), *cur_tac,
							(double)erpm_to_ms(erpm), (double)erpm);
#endif
		}
		break;

		default:
			commands_printf("%s: -- Can't set zero -- Only possible from UNINITIALIZED or BRAKING states",
							state_str(state));
			break;
		}
		break;
	case SET_MANUAL_BRAKING:
		switch (state)
		{
		case UNINITIALIZED:
			commands_printf("%s: -- Can't switch to MANUAL_BRAKING -- Not possible from UNINIITIALIZED", state_str(state));
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
			if (*cur_tac > config.braking_length) {
				commands_printf("%s: -- Can't switch to UNWINDING -- Positive tachometer value",
								state_str(state));
				break;
			}
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
			commands_printf("%s: -- Can't update pull force from UNINITIALIZED", state_str(state));
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

		send_force_is_set();
#ifdef VERBOSE_TERMINAL
		commands_printf("%s: -- %.2fKg (%.1fA, %.1fA/sec) is set",
						state_str(state), (double)terminal_pull_kg, (double)config.pull_current,
						(double)amps_per_sec);
#endif
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
			if (*cur_tac > config.braking_length) {
				commands_printf("%s: -- Can't switch to PRE_PULL -- Positive tachometer value",
								state_str(state));
				break;
			}
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
			if (*cur_tac > config.braking_length) {
				commands_printf("%s: -- Can't switch to TAKEOFF_PULL -- Positive tachometer value",
								state_str(state));
				break;
			}
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
			if (*cur_tac > config.braking_length) {
				commands_printf("%s: -- Can't switch to PULL -- Positive tachometer value",
								state_str(state));
				break;
			}
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
			if (*cur_tac > config.braking_length) {
				commands_printf("%s: -- Can't switch to FAST_PULL -- Positive tachometer value",
								state_str(state));
				break;
			}
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
		send_conf();

		break;
	case SEND_POWER_STATS:
		send_stats(*cur_tac, false);

		break;
	case SEND_TEMP_STATS:
		send_stats(*cur_tac, true);

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

			// mc_configuration changed?
			if (set_drive.motor_poles != mc_conf->si_motor_poles ||
				set_drive.gear_ratio != mc_conf->si_gear_ratio ||
				set_drive.wheel_diameter != mc_conf->si_wheel_diameter ||
				set_drive.battery_type != mc_conf->si_battery_type ||
				set_drive.battery_cells != mc_conf->si_battery_cells)
			{
				mc_configuration new_mc_conf = *mc_conf;

				new_mc_conf.si_motor_poles = set_drive.motor_poles;
				new_mc_conf.si_gear_ratio = set_drive.gear_ratio;
				new_mc_conf.si_wheel_diameter = set_drive.wheel_diameter;

				// Update voltage limits according to battery type and cells
				switch (set_drive.battery_type)
				{
				case BATTERY_TYPE_LIION_3_0__4_2:
					new_mc_conf.l_battery_cut_start = 3.1 * (float)set_drive.battery_cells;
					new_mc_conf.l_battery_cut_end = 3.0 * (float)set_drive.battery_cells;
					new_mc_conf.l_max_vin = 4.25 * (float)set_drive.battery_cells;
					new_mc_conf.si_battery_type = set_drive.battery_type;
					new_mc_conf.si_battery_cells = set_drive.battery_cells;
					break;
				case BATTERY_TYPE_LIIRON_2_6__3_6:
					new_mc_conf.l_battery_cut_start = 2.7 * (float)set_drive.battery_cells;
					new_mc_conf.l_battery_cut_end = 2.6 * (float)set_drive.battery_cells;
					new_mc_conf.l_max_vin = 3.65 * (float)set_drive.battery_cells;
					new_mc_conf.si_battery_type = set_drive.battery_type;
					new_mc_conf.si_battery_cells = set_drive.battery_cells;
					break;

				default:
					// Do not touch battery if we do not know the type
					break;
				}

				// TODO: move this code into separate function and use from commands.c
				conf_general_store_mc_configuration(&new_mc_conf, false);
				mc_interface_set_configuration(&new_mc_conf);
				mc_conf = mc_interface_get_configuration();
			}

			config = set_config;

			store_config_to_eeprom(&config);

			send_command_only(SK_COMM_SETTINGS_APPLIED);
#ifdef VERBOSE_TERMINAL
			commands_printf("%s: -- Settings are set -- Have a nice puffs!", state_str(state));
#endif
			// Announce new settings
			send_conf();

			switch (state)
			{
			case MANUAL_BRAKING: // Update braking force
				manual_brake(*cur_tac);
				break;
			case UNINITIALIZED: // Forget about UNINITIALIZED :)
				braking(*cur_tac);
			default:
				break;
			}

			break;
		default:
			commands_printf("%s: -- Can't set configuration -- Only possible from UNINITIALIZED or any BRAKING states",
							state_str(state));
		}

		break;
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
	default:
		commands_printf("SkyPUFF: unknown terminal command, exiting!");
		stop_now = true;
	}

	// Nothing changed during switch?
	if (prev_command == terminal_command)
		terminal_command = DO_NOTHING;
}

static inline void antisex_adjustment(void)
{
	// Change motor current only if we are in pull states
	if (current_motor_state.mode == MOTOR_CURRENT)
		mc_interface_set_current(current_motor_state.param.current * antisex_amps_gain + antisex_amps);
}

static inline void antisex_tick(void)
{
	bool isCurrentStrongEnough = current_motor_state.mode == MOTOR_CURRENT &&
							     current_motor_state.param.current >= config.antisex_min_pull_amps;
	
	// Antisex disabled?
	if(antisex_amps == 0) {
		// Pull in with fast acceleration pulling more then min pull
		if(isCurrentStrongEnough && measurement.filtered_acceleration_tac_mss >= config.antisex_acceleration_on_mss) {
			// Enable pull reduction
			antisex_amps = -config.antisex_reduce_amps;
			antisex_start_time = measurement.time;
			antisex_adjustment();
#ifdef DEBUG_ANTISEX
			commands_printf("%s: antisex ON", state_str(state));
#endif
		}
		return;
	}

	// Antisex is enabled, but should we turn it off?
	if(!isCurrentStrongEnough || 
	   measurement.filtered_acceleration_tac_mss <= config.antisex_acceleration_off_mss || // Strong pull out acceleration?
	   chTimeDiffX(antisex_start_time, measurement.time) >= MS2ST(config.antisex_max_period_ms)) {  // Timed out?
		antisex_amps = 0;
		antisex_adjustment();
#ifdef DEBUG_ANTISEX
		commands_printf("%s: antisex OFF", state_str(state));
#endif
}
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
		
		if(measurement_tick(cur_tac))  // Are speed and acceleration updated?
			antisex_tick();
	
		int abs_tac = abs(cur_tac);

		// terminal command 'alive'?
		if (alive_inc)
		{
			alive_until = loop_step + alive_inc;
			alive_inc = 0;
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

		update_stats_check_faults();

		chThdSleepMilliseconds(1);
	}
}

// Terminal command to change tachometer value
// NOT SAFE
// Debug usage only!
static void terminal_move_zero(int argc, const char **argv)
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

		commands_printf("%s: -- moving zero %.2fm (%d steps) %s, cur pos: %.2fm (%d steps), new pos: %.2fm (%d steps)",
						state_str(state),
						(double)d, steps, d < 0 ? "backward" : "forward",
						(double)tac_steps_to_meters(cur_tac), cur_tac,
						(double)tac_steps_to_meters(new_tac), new_tac);

		mc_interface_set_tachometer_value(new_tac);
	}
	else
	{
		commands_printf("This command requires one argument: 'move_zero -5.2' will move zero 5.2 meters backward");
	}
}

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

	set_example_drive(&set_drive);
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

void terminal_adc2_tick(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	const int delay = 1, loops = 6;

#ifdef VERBOSE_TERMINAL
	commands_printf("%s: -- ADC2 %d impulses with %d delay",
	                state_str(state), loops, delay);
#endif

	for(int i = 0; i < loops; i++) {
		palSetPad(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN);
		chThdSleepMilliseconds(delay);
		palClearPad(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN);
		chThdSleepMilliseconds(delay);
	}
}


void draw_spool_mass_graph(systime_t started, int drawing_ms, float kg)
{
	systime_t cur_time = chVTGetSystemTime();
	systime_t end_time = chTimeAddX(cur_time, MS2ST(drawing_ms));
	systime_t last_measurement_time = 0;

	while(chVTGetSystemTime() < end_time) {
		if(last_measurement_time != measurement.time) {
			last_measurement_time = measurement.time;

			sysinterval_t from_startedI = chTimeDiffX(started, measurement.time);

			// Convert to microseconds cause overflow, so convert in serial way
			float from_startedS = (float)from_startedI / (float)CH_CFG_ST_FREQUENCY;

			int n = 0; // Graph number

			// Speed tachometer (m/s)
			commands_plot_set_graph(n++);
			commands_send_plot_points(from_startedS, measurement.speed_tac_ms);

			// Speed tachometer filtered (m/s)
			commands_plot_set_graph(n++);
			commands_send_plot_points(from_startedS, measurement.filtered_speed_tac_ms);

			// Acceleration tachometer filtered (m/s^2)
			commands_plot_set_graph(n++);
			commands_send_plot_points(from_startedS, measurement.filtered_acceleration_tac_mss);

			// Instant pull (kg)
			commands_plot_set_graph(n++);
			commands_send_plot_points(from_startedS, mc_interface_get_tot_current_directional_filtered() / config.amps_per_kg);

			// Mass (kg) 
			// F = ma, m = F / a
			if(fabs(measurement.filtered_acceleration_tac_mss) > 1) {
				commands_plot_set_graph(n++);
				commands_send_plot_points(from_startedS, kg / measurement.filtered_acceleration_tac_mss);
			}
		}

		chThdSleepMilliseconds(1);
	} 
}

void terminal_measure_spool(int argc, const char **argv)
{
	float kg = 1, secs = 2;
	int use_smooth = 0;

	if (argc < 3)
	{
		commands_printf("%s: -- 'measure_spool' requires at least two arguments: <kg> <seconds> [0/1 smooth motor] -- For example: 'measure_spool 5 3 1'",
						state_str(state));
		return;
	}

	if (sscanf(argv[1], "%f", &kg) == EOF)
	{
		commands_printf("%s: -- Can't parse '%s' as kg value.",
						state_str(state), argv[1]);
		return;
	};

	if (sscanf(argv[2], "%f", &secs) == EOF)
	{
		commands_printf("%s: -- Can't parse '%s' as seconds value.",
						state_str(state), argv[2]);
		return;
	};

	if(argc == 4 && sscanf(argv[3], "%d", &use_smooth) == EOF)
	{
		commands_printf("%s: -- Can't parse '%s' as use_smooth value.",
						state_str(state), argv[3]);
		return;
	};

	if(state == UNINITIALIZED) {
		commands_printf("%s: -- Can't measure spool in UNITIALIZED state!", state_str(state));
		return;
	}

	// Set alive during our tests
	alive_inc = (secs + 1) * 2000;
	chThdSleepMilliseconds(5);

	float amps = kg * config.amps_per_kg;

	// Release motor
	terminal_command = SET_SMOOTH;
	terminal_motor_state.mode = MOTOR_RELEASED;
	terminal_motor_state.param.current = 0;

	chThdSleepMilliseconds(5);
	
	if(state != MANUAL_DEBUG_SMOOTH) {
		commands_printf("%s: -- Can't set MANUAL_DEBUG_SMOOTH state!", state_str(state));
		return;
	}

	commands_init_plot("Seconds", "Speed (m/s)");
	commands_plot_add_graph("Speed unfiltered (m/s)");
	commands_plot_add_graph("Speed tachometer (m/s)");
	commands_plot_add_graph("Acceleration tachometer filtered (m/s^2)");
	commands_plot_add_graph("Pull (kg)");
	commands_plot_add_graph("Spool 'virtual mass' (kg)");

	commands_printf("%s: -- Running forward and backward using %s with %.2fkg (%.1fA) force during %.2fsecs...", 
					state_str(state), use_smooth ? "'smooth motor'" : "'current control'",
					(double)kg, (double)(-amps), (double)secs);

	timeout_reset();

	systime_t started = chVTGetSystemTime();

	if(use_smooth) {
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_CURRENT;
		terminal_motor_state.param.current = amps;

	}
	else
		mc_interface_set_current(amps);

	draw_spool_mass_graph(started, secs * 1000, kg);

	if(use_smooth) {
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_CURRENT;
		terminal_motor_state.param.current = -amps;

	}
	else
		mc_interface_set_current(-amps);

	draw_spool_mass_graph(started, secs * 1500, kg);

	mc_interface_release_motor();

	// Setting MANUAL_BRACKING safe state
	terminal_command = SET_MANUAL_BRAKING;
	chThdSleepMilliseconds(5);
	commands_printf("%s: -- Measure done! Checkout graphs in 'Realtime Data' - 'Experiment'", state_str(state));

}
