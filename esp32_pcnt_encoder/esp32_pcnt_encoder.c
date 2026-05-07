/*
	Name: esp32_pcnt_encoder.c
	Description: ESP32 PCNT-backed custom encoder reader for uCNC.
*/

#include "../../cnc.h"
#include "../encoder.h"

#if (MCU == MCU_ESP32 || MCU == MCU_ESP32S3 || MCU == MCU_ESP32C3)

#include "driver/pcnt.h"
#include <stdio.h>
#include <string.h>

#if (UCNC_MODULE_VERSION < 11501 || UCNC_MODULE_VERSION > 99999)
#error "This module is not compatible with the current version of uCNC"
#endif

#ifndef ENC0_PCNT_UNIT
#define ENC0_PCNT_UNIT PCNT_UNIT_0
#endif

#ifndef ENC0_PULSE_GPIO
#error "ENC0_PULSE_GPIO is not defined"
#endif

#ifndef ENC0_DIR_GPIO
#error "ENC0_DIR_GPIO is not defined"
#endif

#if defined(ENC0_INDEX_PCNT_UNIT) && defined(ENC0_INDEX_GPIO)
#define ENC0_INDEX_PCNT_ENABLED 1
#endif

#if defined(ENC0_INDEX_PCNT_FILTER_US) && !defined(ENC0_INDEX_PCNT_FILTER)
#define ENC0_INDEX_PCNT_FILTER ((ENC0_INDEX_PCNT_FILTER_US) * 80)
#endif

#if defined(ENC0_INDEX_PCNT_FILTER) && (ENC0_INDEX_PCNT_FILTER > 1023)
#undef ENC0_INDEX_PCNT_FILTER
#define ENC0_INDEX_PCNT_FILTER 1023
#endif

#ifndef ENC0_PCNT_RECENTER_THRESHOLD
#define ENC0_PCNT_RECENTER_THRESHOLD 20000
#endif

static bool esp32_pcnt_encoder_ready;
static int32_t esp32_pcnt_encoder_offset;
static volatile uint8_t esp32_pcnt_index_pending;
static bool esp32_pcnt_index_have_origin;
static bool esp32_pcnt_index_have_stats;
static int32_t esp32_pcnt_index_last_position;
static int32_t esp32_pcnt_index_last_delta;
static int32_t esp32_pcnt_index_min_delta;
static int32_t esp32_pcnt_index_max_delta;
static uint32_t esp32_pcnt_index_count;
static uint32_t esp32_pcnt_index_abs_delta_sum;
static uint32_t esp32_pcnt_index_hw_count;
static uint32_t esp32_pcnt_index_ignored_count;
static uint32_t esp32_pcnt_index_bad_count;
static uint32_t esp32_pcnt_index_missed_count;
static bool esp32_pcnt_index_resync_after_miss;
static char esp32_pcnt_index_debug_line[128];
static uint32_t esp32_pcnt_index_debug_seq;
static uint32_t esp32_pcnt_index_debug_reported_count;
static uint32_t esp32_pcnt_index_debug_reported_hw_count;
static uint32_t esp32_pcnt_index_debug_reported_ignored_count;
static uint32_t esp32_pcnt_index_debug_reported_bad_count;
static uint32_t esp32_pcnt_index_debug_reported_missed_count;

static uint32_t esp32_pcnt_encoder_index_resolution(void)
{
	float resolution = g_settings.encoders_resolution[ENC0];
	if (resolution >= 1.0f)
	{
		return (uint32_t)(resolution + 0.5f);
	}
#ifdef ENC0_CPR
	return (uint32_t)ENC0_CPR;
#else
	return 0;
#endif
}

static uint32_t esp32_pcnt_encoder_index_min_delta(void)
{
#ifdef ENC0_INDEX_MIN_DELTA
	return (uint32_t)ENC0_INDEX_MIN_DELTA;
#else
	uint32_t resolution = esp32_pcnt_encoder_index_resolution();
	return (resolution) ? ((resolution * 3U) / 4U) : 1U;
#endif
}

static uint32_t esp32_pcnt_encoder_index_max_delta(void)
{
#ifdef ENC0_INDEX_MAX_DELTA
	return (uint32_t)ENC0_INDEX_MAX_DELTA;
#else
	uint32_t resolution = esp32_pcnt_encoder_index_resolution();
	return (resolution) ? ((resolution * 5U) / 4U) : 2147483647UL;
#endif
}

static void esp32_pcnt_encoder_update_index_debug(void)
{
	int32_t live_delta;
	uint32_t avg10;

	if (!esp32_pcnt_index_have_origin && !esp32_pcnt_index_hw_count)
	{
		return;
	}

	if (esp32_pcnt_index_debug_reported_count == esp32_pcnt_index_count &&
		esp32_pcnt_index_debug_reported_hw_count == esp32_pcnt_index_hw_count &&
		esp32_pcnt_index_debug_reported_ignored_count == esp32_pcnt_index_ignored_count &&
		esp32_pcnt_index_debug_reported_bad_count == esp32_pcnt_index_bad_count &&
		esp32_pcnt_index_debug_reported_missed_count == esp32_pcnt_index_missed_count)
	{
		return;
	}

	esp32_pcnt_index_debug_reported_count = esp32_pcnt_index_count;
	esp32_pcnt_index_debug_reported_hw_count = esp32_pcnt_index_hw_count;
	esp32_pcnt_index_debug_reported_ignored_count = esp32_pcnt_index_ignored_count;
	esp32_pcnt_index_debug_reported_bad_count = esp32_pcnt_index_bad_count;
	esp32_pcnt_index_debug_reported_missed_count = esp32_pcnt_index_missed_count;

	live_delta = encoder_get_position(ENC0) - esp32_pcnt_index_last_position;
	avg10 = (esp32_pcnt_index_count) ? (uint32_t)(((uint64_t)esp32_pcnt_index_abs_delta_sum * 10ULL + (esp32_pcnt_index_count / 2U)) / esp32_pcnt_index_count) : 0;

	snprintf(esp32_pcnt_index_debug_line,
			 sizeof(esp32_pcnt_index_debug_line),
			 "ENCIDX EC:%ld ECB:%ld LAST:%ld AVG:%lu.%lu MIN:%ld MAX:%ld N:%lu HW:%lu IGN:%lu BAD:%lu MISS:%lu",
			 (long)encoder_get_position(ENC0),
			 (long)live_delta,
			 (long)esp32_pcnt_index_last_delta,
			 (unsigned long)(avg10 / 10U),
			 (unsigned long)(avg10 % 10U),
			 (long)esp32_pcnt_index_min_delta,
			 (long)esp32_pcnt_index_max_delta,
			 (unsigned long)esp32_pcnt_index_count,
			 (unsigned long)esp32_pcnt_index_hw_count,
			 (unsigned long)esp32_pcnt_index_ignored_count,
			 (unsigned long)esp32_pcnt_index_bad_count,
			 (unsigned long)esp32_pcnt_index_missed_count);

	esp32_pcnt_index_debug_seq++;
}

static void esp32_pcnt_encoder_invoke_virtual_index(void)
{
#ifdef ENC0_INDEX_PCNT_ENABLED
	HOOK_INVOKE(enc0_index);
#endif
}

static void esp32_pcnt_encoder_process_index(void)
{
	uint8_t pending;
	int32_t position;
	int32_t delta;
	uint32_t abs_delta;
	uint32_t min_delta;
	uint32_t max_delta;

	pending = esp32_pcnt_index_pending;
	if (!pending)
	{
		return;
	}

	esp32_pcnt_index_pending = 0;
	position = encoder_get_position(ENC0);

	if (!esp32_pcnt_index_have_origin)
	{
		esp32_pcnt_index_last_position = position;
		esp32_pcnt_index_have_origin = true;
		esp32_pcnt_encoder_update_index_debug();
		return;
	}

	delta = position - esp32_pcnt_index_last_position;
	abs_delta = (uint32_t)ABS(delta);
	min_delta = esp32_pcnt_encoder_index_min_delta();
	max_delta = esp32_pcnt_encoder_index_max_delta();
	if (abs_delta < min_delta)
	{
		esp32_pcnt_index_ignored_count++;
		esp32_pcnt_encoder_update_index_debug();
		return;
	}
	if (abs_delta > max_delta)
	{
		esp32_pcnt_index_missed_count++;
		esp32_pcnt_index_resync_after_miss = false;
		esp32_pcnt_index_last_position = position;
		esp32_pcnt_encoder_update_index_debug();
		return;
	}

	esp32_pcnt_index_resync_after_miss = false;
	esp32_pcnt_index_last_position = position;
	esp32_pcnt_index_last_delta = delta;
	esp32_pcnt_index_count++;
	esp32_pcnt_index_abs_delta_sum += (uint32_t)ABS(delta);

	if (!esp32_pcnt_index_have_stats)
	{
		esp32_pcnt_index_min_delta = delta;
		esp32_pcnt_index_max_delta = delta;
		esp32_pcnt_index_have_stats = true;
	}
	else
	{
		if (delta < esp32_pcnt_index_min_delta)
		{
			esp32_pcnt_index_min_delta = delta;
		}
		if (delta > esp32_pcnt_index_max_delta)
		{
			esp32_pcnt_index_max_delta = delta;
		}
	}

	esp32_pcnt_encoder_update_index_debug();
}

static void esp32_pcnt_encoder_latch_index(void)
{
	if (esp32_pcnt_index_pending < 255)
	{
		esp32_pcnt_index_pending++;
	}
}

#ifndef ENC0_INDEX_PCNT_ENABLED
static void esp32_pcnt_encoder_on_index(void)
{
	esp32_pcnt_index_hw_count++;
	esp32_pcnt_encoder_latch_index();
}
#endif

#ifdef ENC0_INDEX_PCNT_ENABLED
static void encoder_esp32_index_pcnt_init(uint8_t unit, int index_gpio)
{
	pcnt_config_t index_ch = {
		.pulse_gpio_num = index_gpio,
		.ctrl_gpio_num = PCNT_PIN_NOT_USED,
		.lctrl_mode = PCNT_MODE_KEEP,
		.hctrl_mode = PCNT_MODE_KEEP,
		.pos_mode = PCNT_COUNT_INC,
		.neg_mode = PCNT_COUNT_DIS,
		.counter_h_lim = 32767,
		.counter_l_lim = 0,
		.unit = (pcnt_unit_t)unit,
		.channel = PCNT_CHANNEL_0,
	};

	pcnt_unit_config(&index_ch);

#ifdef ENC0_INDEX_PCNT_FILTER
	pcnt_set_filter_value((pcnt_unit_t)unit, ENC0_INDEX_PCNT_FILTER);
	pcnt_filter_enable((pcnt_unit_t)unit);
#elif defined(ENCODER_PCNT_FILTER)
	pcnt_set_filter_value((pcnt_unit_t)unit, ENCODER_PCNT_FILTER);
	pcnt_filter_enable((pcnt_unit_t)unit);
#else
	pcnt_filter_disable((pcnt_unit_t)unit);
#endif

	pcnt_counter_pause((pcnt_unit_t)unit);
	pcnt_counter_clear((pcnt_unit_t)unit);
	pcnt_counter_resume((pcnt_unit_t)unit);
}

static void esp32_pcnt_encoder_drain_index_pcnt(void)
{
	int16_t index_count = 0;

	if (!esp32_pcnt_encoder_ready)
	{
		return;
	}

	pcnt_get_counter_value((pcnt_unit_t)ENC0_INDEX_PCNT_UNIT, &index_count);
	if (index_count <= 0)
	{
		return;
	}

	pcnt_counter_pause((pcnt_unit_t)ENC0_INDEX_PCNT_UNIT);
	pcnt_counter_clear((pcnt_unit_t)ENC0_INDEX_PCNT_UNIT);
	pcnt_counter_resume((pcnt_unit_t)ENC0_INDEX_PCNT_UNIT);

	esp32_pcnt_index_hw_count += (uint32_t)index_count;
	if (index_count > 1)
	{
		esp32_pcnt_index_missed_count += (uint32_t)(index_count - 1);
		esp32_pcnt_index_resync_after_miss = true;
	}
	esp32_pcnt_encoder_latch_index();
	esp32_pcnt_encoder_update_index_debug();
	esp32_pcnt_encoder_invoke_virtual_index();
}
#endif

static void encoder_esp32_pcnt_init(uint8_t unit, int pulse_gpio, int dir_gpio)
{
	pcnt_config_t ch0 = {
		.pulse_gpio_num = pulse_gpio,
		.ctrl_gpio_num = dir_gpio,
		.lctrl_mode = PCNT_MODE_REVERSE,
		.hctrl_mode = PCNT_MODE_KEEP,
		.pos_mode = PCNT_COUNT_INC,
		.neg_mode = PCNT_COUNT_DEC,
		.counter_h_lim = 32767,
		.counter_l_lim = -32768,
		.unit = (pcnt_unit_t)unit,
		.channel = PCNT_CHANNEL_0,
	};

	pcnt_config_t ch1 = {
		.pulse_gpio_num = dir_gpio,
		.ctrl_gpio_num = pulse_gpio,
		.lctrl_mode = PCNT_MODE_KEEP,
		.hctrl_mode = PCNT_MODE_REVERSE,
		.pos_mode = PCNT_COUNT_INC,
		.neg_mode = PCNT_COUNT_DEC,
		.counter_h_lim = 32767,
		.counter_l_lim = -32768,
		.unit = (pcnt_unit_t)unit,
		.channel = PCNT_CHANNEL_1,
	};

	pcnt_unit_config(&ch0);
	pcnt_unit_config(&ch1);

#ifdef ENCODER_PCNT_FILTER
	pcnt_set_filter_value((pcnt_unit_t)unit, ENCODER_PCNT_FILTER);
	pcnt_filter_enable((pcnt_unit_t)unit);
#else
	pcnt_filter_disable((pcnt_unit_t)unit);
#endif

	pcnt_counter_pause((pcnt_unit_t)unit);
	pcnt_counter_clear((pcnt_unit_t)unit);
	pcnt_counter_resume((pcnt_unit_t)unit);
}

int32_t read_encoder_esp32_pcnt(uint8_t unit)
{
	int16_t value = 0;
	int32_t out;

	if (!esp32_pcnt_encoder_ready)
	{
		return 0;
	}

	pcnt_get_counter_value((pcnt_unit_t)unit, &value);

	out = esp32_pcnt_encoder_offset + (int32_t)value;
	if (value >= ENC0_PCNT_RECENTER_THRESHOLD || value <= -ENC0_PCNT_RECENTER_THRESHOLD)
	{
		pcnt_counter_pause((pcnt_unit_t)unit);
		pcnt_counter_clear((pcnt_unit_t)unit);
		pcnt_counter_resume((pcnt_unit_t)unit);
		esp32_pcnt_encoder_offset = out;
	}

	return out;
}

int32_t enc_custom_read(uint8_t i)
{
	switch (i)
	{
	case ENC0:
		return read_encoder_esp32_pcnt(ENC0_PCNT_UNIT);
	default:
		return 0;
	}
}

bool encoder_get_index_stats(uint8_t i, int32_t *last, int32_t *min, int32_t *max, uint32_t *count)
{
	if (i != ENC0 || !esp32_pcnt_index_have_stats)
	{
		return false;
	}

	if (last)
	{
		*last = esp32_pcnt_index_last_delta;
	}
	if (min)
	{
		*min = esp32_pcnt_index_min_delta;
	}
	if (max)
	{
		*max = esp32_pcnt_index_max_delta;
	}
	if (count)
	{
		*count = esp32_pcnt_index_count;
	}

	return true;
}

bool encoder_get_index_live_delta(uint8_t i, int32_t *delta)
{
	if (i != ENC0 || !esp32_pcnt_index_have_origin || !delta)
	{
		return false;
	}

	*delta = encoder_get_position(ENC0) - esp32_pcnt_index_last_position;
	return true;
}

bool encoder_get_index_debug_line(uint8_t i, char *line, uint32_t line_len, uint32_t *seq)
{
	if (i != ENC0 || !line || line_len == 0 || !esp32_pcnt_index_debug_seq)
	{
		return false;
	}

	strncpy(line, esp32_pcnt_index_debug_line, line_len - 1);
	line[line_len - 1] = '\0';
	if (seq)
	{
		*seq = esp32_pcnt_index_debug_seq;
	}
	return true;
}

static bool esp32_pcnt_encoder_dotasks(void *args)
{
	(void)args;
#ifdef ENC0_INDEX_PCNT_ENABLED
	esp32_pcnt_encoder_drain_index_pcnt();
#endif
	esp32_pcnt_encoder_process_index();
	esp32_pcnt_encoder_update_index_debug();
	return EVENT_CONTINUE;
}

CREATE_EVENT_LISTENER(cnc_io_dotasks, esp32_pcnt_encoder_dotasks);

DECL_MODULE(esp32_pcnt_encoder)
{
	encoder_esp32_pcnt_init(ENC0_PCNT_UNIT, ENC0_PULSE_GPIO, ENC0_DIR_GPIO);
#ifdef ENC0_INDEX_PCNT_ENABLED
	encoder_esp32_index_pcnt_init(ENC0_INDEX_PCNT_UNIT, ENC0_INDEX_GPIO);
#else
	HOOK_ATTACH_CALLBACK(enc0_index, esp32_pcnt_encoder_on_index);
#endif
	ADD_EVENT_LISTENER(cnc_io_dotasks, esp32_pcnt_encoder_dotasks);
	esp32_pcnt_encoder_ready = true;
}

#else

DECL_MODULE(esp32_pcnt_encoder)
{
}

#endif
