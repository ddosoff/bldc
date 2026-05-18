#ifndef CONF_SKYPUFF_H_
#define CONF_SKYPUFF_H_

#include "datatypes.h"

/**
 * Any received command will increment so called
 * alive timeout (loop iterations count) by this value
 */
const int alive_timeout_increment = 1400; // Around 1.4 seconds

// Part of motor configuration needed by skypuff UI
typedef struct {
	float motor_max_current;
	float charge_max_current;
	float discharge_max_current;
	float fet_temp_max;
	float motor_temp_max;
	float v_in_max;
	float v_in_min;
} skypuff_scales;

// Winch settings
typedef struct {
	float amps_per_kg;                  // Winch drive force coefficient
	int pull_applying_period;           // Milliseconds to apply pull force, amps_per_sec will be calculated from this delay
	int braking_applying_period;        // Milliseconds to release pull force when going Manual Braking
	int rope_length;                    // Winch rope length in tachometer steps (used by interface only)
	int braking_length;                 // Tachometer range of braking zone
	int braking_extension_length;       // Increase braking_length for passive winches when car drive 150m from takeoff
	int slowing_length;                 // Range after braking zone to slow down motor when unwinding to zero
	float slow_erpm;                    // Constant erpm in direction to zero
	int rewinding_trigger_length;       // Switch to fast rewinding state after going back this length
	int unwinding_trigger_length;       // Back to unwinding from rewinding if this range unwinded again
	float pull_current;                 // Winch normal pull force, usually pilot weight
	float pre_pull_k;                   // pre_pull_k * pull_current = pull current when pilots stays on the ground
	float takeoff_pull_k;               // takeoff_pull_k * pull_current = pull current during takeoff
	float fast_pull_k;                  // fast_pull_k * pull_current = pull current to get altitude fast
	int takeoff_trigger_length;         // Minimal PRE_PULL movement for transition to TAKEOFF_PULL
	int pre_pull_timeout;               // Milliseconds timeout to save position after PRE_PULL
	int takeoff_period;                 // Time of TAKEOFF_PULL and then switch to normal PULL
	float brake_current;                // Braking zone force, could be set high to charge battery driving away
	float slowing_current;              // Set zero to release motor when slowing or positive value to brake
	float manual_brake_current;         // Manual braking force
	float unwinding_current;            // Unwinding force
	float unwinding_strong_current;     // Due to motor cogging we need more powerfull unwinding near zero speed
	float unwinding_strong_erpm;        // Enable strong current unwinding if current speed is above
	float rewinding_current;            // Rewinding force
	float slow_max_current;             // Max force for constant slow speed
	float manual_slow_max_current;      // Max force for MANUAL_SLOW and MANUAL_SLOW_BACK
	float manual_slow_speed_up_current; // Speed up current for manual constant speed states
	float manual_slow_erpm;             // Constant speed for manual rotation
	float max_speed_ms;                 // Speed scale limit (only affects the interface)

	// Antisex dampering
	float antisex_min_pull_amps;        // Activate antisex pull currection if pulling above this only
	float antisex_reduce_amps;          // Reduce motor amps to this value when antisex is activated
	float antisex_acceleration_on_mss;  // Activate antisex pull reduce if winding acceleration above this
	float antisex_acceleration_off_mss; // Deactivate antisex if current acceleration below this
	int antisex_max_period_ms;          // Do not reduce nominal pull more then this milliseconds
} skypuff_config;

// Drive settings part of mc_configuration
typedef struct {
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

inline static bool deserialize_scales(unsigned char *data, unsigned int len, skypuff_scales *to, int32_t *ind) {
	const int32_t serialized_scales_v1_length = 2 * 5 + 4 * 2;

	int available_bytes = len - *ind;
	if (available_bytes < serialized_scales_v1_length) {
		return false;
	}

	to->motor_max_current = buffer_get_float16(data, 1e1, ind);
	to->discharge_max_current = buffer_get_float16(data, 1e1, ind);
	to->charge_max_current = buffer_get_float16(data, 1e1, ind);
	to->fet_temp_max = buffer_get_float16(data, 1e1, ind);
	to->motor_temp_max = buffer_get_float16(data, 1e1, ind);
	to->v_in_min = buffer_get_float32(data, 1e2, ind);
	to->v_in_max = buffer_get_float32(data, 1e2, ind);
	return true;
}

inline static void serialize_config(uint8_t *buffer, int32_t *ind, skypuff_config *config) {
	buffer_append_float16(buffer, config->amps_per_kg, 1e2, ind);
	buffer_append_uint16(buffer, config->pull_applying_period, ind);
	buffer_append_int32(buffer, config->rope_length, ind);
	buffer_append_int32(buffer, config->braking_length, ind);
	buffer_append_int32(buffer, config->braking_extension_length, ind);

	buffer_append_int32(buffer, config->slowing_length, ind);
	buffer_append_uint16(buffer, config->slow_erpm, ind);
	buffer_append_int32(buffer, config->rewinding_trigger_length, ind);
	buffer_append_int32(buffer, config->unwinding_trigger_length, ind);
	buffer_append_float16(buffer, config->pull_current, 10, ind);

	buffer_append_float16(buffer, config->pre_pull_k, 1e4, ind);
	buffer_append_float16(buffer, config->takeoff_pull_k, 1e4, ind);
	buffer_append_float16(buffer, config->fast_pull_k, 1e4, ind);
	buffer_append_uint16(buffer, config->takeoff_trigger_length, ind);
	buffer_append_uint16(buffer, config->pre_pull_timeout, ind);

	buffer_append_uint16(buffer, config->takeoff_period, ind);
	buffer_append_float16(buffer, config->brake_current, 10, ind);
	buffer_append_float16(buffer, config->slowing_current, 10, ind);
	buffer_append_float16(buffer, config->manual_brake_current, 10, ind);
	buffer_append_float16(buffer, config->unwinding_current, 10, ind);

	buffer_append_float16(buffer, config->rewinding_current, 10, ind);
	buffer_append_float16(buffer, config->slow_max_current, 10, ind);
	buffer_append_float16(buffer, config->manual_slow_max_current, 10, ind);
	buffer_append_float16(buffer, config->manual_slow_speed_up_current, 10, ind);
	buffer_append_uint16(buffer, config->manual_slow_erpm, ind);

	buffer_append_float16(buffer, config->antisex_min_pull_amps, 10, ind);
	buffer_append_float16(buffer, config->antisex_reduce_amps, 10, ind);
	buffer_append_float16(buffer, config->antisex_acceleration_on_mss, 1e2, ind);
	buffer_append_float16(buffer, config->antisex_acceleration_off_mss, 1e2, ind);
	buffer_append_uint16(buffer, config->antisex_max_period_ms, ind);

	buffer_append_float16(buffer, config->max_speed_ms, 1e2, ind);
	buffer_append_uint16(buffer, config->braking_applying_period, ind);
	buffer_append_float16(buffer, config->unwinding_strong_current, 10, ind);
	buffer_append_int16(buffer, config->unwinding_strong_erpm, ind);
}

inline static bool deserialize_config(unsigned char *data, unsigned int len, skypuff_config *to, int32_t *ind) {
	const int32_t serialized_settings_v1_length = 16 + 16 + 10 + 10 + 10 + 10 + 8;

	int available_bytes = len - *ind;
	if (available_bytes < serialized_settings_v1_length) {
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

inline static bool deserialize_drive(unsigned char *data, unsigned int len, skypuff_drive *to, int32_t *ind) {
	const int32_t serialized_drive_v1_length = 1 + 4 * 2 + 2;

	int available_bytes = len - *ind;
	if (available_bytes < serialized_drive_v1_length) {
		return false;
	}

	to->motor_poles = data[(*ind)++];
	to->gear_ratio = buffer_get_float32_auto(data, ind);
	to->wheel_diameter = buffer_get_float32_auto(data, ind);
	to->battery_type = (BATTERY_TYPE) data[(*ind)++];
	to->battery_cells = data[(*ind)++];
	return true;
}

#endif