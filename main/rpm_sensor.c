#include "rpm_sensor.h"

#include <stddef.h>
#include <string.h>

#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "pin_config.h"

#define RPM_MIN_WINDOW_SAMPLES 5
#define RPM_MAX_WINDOW_SAMPLES 25
#define RPM_MIN_EDGES_PER_ESTIMATE 4
#define RPM_SIGNAL_TIMEOUT_US 250000

typedef struct {
    int count;
    int64_t duration_us;
} rpm_window_sample_t;

static const gpio_num_t rpm_pins[POWERTRAIN_WHEEL_COUNT] = {
    [WHEEL_FRONT_LEFT] = PIN_RPM_FL_INPUT,
    [WHEEL_FRONT_RIGHT] = PIN_RPM_FR_INPUT,
    [WHEEL_REAR_LEFT] = PIN_RPM_RL_INPUT,
    [WHEEL_REAR_RIGHT] = PIN_RPM_RR_INPUT,
};

static pcnt_unit_handle_t units[POWERTRAIN_WHEEL_COUNT];
static pcnt_channel_handle_t channels[POWERTRAIN_WHEEL_COUNT];
static rpm_window_sample_t sample_windows[POWERTRAIN_WHEEL_COUNT][RPM_MAX_WINDOW_SAMPLES];
static int64_t last_pulse_at_us[POWERTRAIN_WHEEL_COUNT];
static int64_t last_update_at_us;
static size_t window_index;
static uint16_t configured_pulses_per_revolution = 7;
static rpm_snapshot_t current_snapshot;
static portMUX_TYPE snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

static bool pulses_per_revolution_valid(uint16_t pulses_per_revolution)
{
    return pulses_per_revolution >= 1 && pulses_per_revolution <= 120;
}

esp_err_t rpm_sensor_init(uint16_t pulses_per_revolution)
{
    if (!pulses_per_revolution_valid(pulses_per_revolution)) {
        return ESP_ERR_INVALID_ARG;
    }

    configured_pulses_per_revolution = pulses_per_revolution;

    for (size_t i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        pcnt_unit_config_t unit_config = {
            .low_limit = -1,
            .high_limit = 32767,
        };
        esp_err_t err = pcnt_new_unit(&unit_config, &units[i]);
        if (err != ESP_OK) {
            return err;
        }

        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 500,
        };
        err = pcnt_unit_set_glitch_filter(units[i], &filter_config);
        if (err != ESP_OK) {
            return err;
        }

        pcnt_chan_config_t channel_config = {
            .edge_gpio_num = rpm_pins[i],
            .level_gpio_num = -1,
        };
        err = pcnt_new_channel(units[i], &channel_config, &channels[i]);
        if (err != ESP_OK) {
            return err;
        }

        err = pcnt_channel_set_edge_action(channels[i],
                                           PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                           PCNT_CHANNEL_EDGE_ACTION_HOLD);
        if (err != ESP_OK) {
            return err;
        }

        err = pcnt_unit_enable(units[i]);
        if (err != ESP_OK) {
            return err;
        }
        err = pcnt_unit_clear_count(units[i]);
        if (err != ESP_OK) {
            return err;
        }
        err = pcnt_unit_start(units[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    last_update_at_us = esp_timer_get_time();
    return ESP_OK;
}

void rpm_sensor_set_pulses_per_revolution(uint16_t pulses_per_revolution)
{
    if (!pulses_per_revolution_valid(pulses_per_revolution)) {
        return;
    }

    portENTER_CRITICAL(&state_lock);
    configured_pulses_per_revolution = pulses_per_revolution;
    memset(sample_windows, 0, sizeof(sample_windows));
    memset(last_pulse_at_us, 0, sizeof(last_pulse_at_us));
    window_index = 0;
    portEXIT_CRITICAL(&state_lock);
}

uint16_t rpm_sensor_pulses_per_revolution(void)
{
    portENTER_CRITICAL(&state_lock);
    uint16_t pulses_per_revolution = configured_pulses_per_revolution;
    portEXIT_CRITICAL(&state_lock);
    return pulses_per_revolution;
}

void rpm_sensor_update(void)
{
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - last_update_at_us;
    if (elapsed_us <= 0) {
        return;
    }
    last_update_at_us = now_us;

    float rpm_values[POWERTRAIN_WHEEL_COUNT] = {0};
    float frequency_values[POWERTRAIN_WHEEL_COUNT] = {0};
    uint32_t edge_counts[POWERTRAIN_WHEEL_COUNT] = {0};
    uint32_t window_durations_us[POWERTRAIN_WHEEL_COUNT] = {0};
    bool valid_values[POWERTRAIN_WHEEL_COUNT] = {false};

    int counts[POWERTRAIN_WHEEL_COUNT] = {0};
    bool count_valid[POWERTRAIN_WHEEL_COUNT] = {false};
    for (size_t i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        if (pcnt_unit_get_count(units[i], &counts[i]) != ESP_OK) {
            continue;
        }
        if (pcnt_unit_clear_count(units[i]) != ESP_OK) {
            continue;
        }
        count_valid[i] = counts[i] >= 0;
    }

    portENTER_CRITICAL(&state_lock);
    uint16_t pulses_per_revolution = configured_pulses_per_revolution;
    for (size_t i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        if (!count_valid[i]) {
            continue;
        }

        if (counts[i] > 0) {
            last_pulse_at_us[i] = now_us;
        }

        sample_windows[i][window_index].count = counts[i];
        sample_windows[i][window_index].duration_us = elapsed_us;

        int total_count = 0;
        int64_t total_duration_us = 0;
        for (size_t samples_used = 0; samples_used < RPM_MAX_WINDOW_SAMPLES; samples_used++) {
            size_t sample_index =
                (window_index + RPM_MAX_WINDOW_SAMPLES - samples_used) % RPM_MAX_WINDOW_SAMPLES;
            total_count += sample_windows[i][sample_index].count;
            total_duration_us += sample_windows[i][sample_index].duration_us;
            if (samples_used + 1 >= RPM_MIN_WINDOW_SAMPLES &&
                total_count >= RPM_MIN_EDGES_PER_ESTIMATE) {
                break;
            }
        }

        valid_values[i] = last_pulse_at_us[i] > 0 &&
                          (now_us - last_pulse_at_us[i]) <= RPM_SIGNAL_TIMEOUT_US;
        if (valid_values[i] && total_duration_us > 0) {
            frequency_values[i] = ((float)total_count * 1000000.0f) /
                                  (float)total_duration_us;
            rpm_values[i] = (frequency_values[i] * 60.0f) /
                            (float)pulses_per_revolution;
            edge_counts[i] = (uint32_t)total_count;
            window_durations_us[i] = total_duration_us > UINT32_MAX
                                         ? UINT32_MAX
                                         : (uint32_t)total_duration_us;
        }
    }

    window_index = (window_index + 1) % RPM_MAX_WINDOW_SAMPLES;
    portEXIT_CRITICAL(&state_lock);

    portENTER_CRITICAL(&snapshot_lock);
    for (size_t i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        current_snapshot.rpm[i] = rpm_values[i];
        current_snapshot.frequency_hz[i] = frequency_values[i];
        current_snapshot.edge_count[i] = edge_counts[i];
        current_snapshot.window_us[i] = window_durations_us[i];
        current_snapshot.valid[i] = valid_values[i];
    }
    current_snapshot.updated_at_us = now_us;
    current_snapshot.pulses_per_revolution = pulses_per_revolution;
    portEXIT_CRITICAL(&snapshot_lock);
}

void rpm_sensor_get_snapshot(rpm_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&snapshot_lock);
    *snapshot = current_snapshot;
    portEXIT_CRITICAL(&snapshot_lock);
}
