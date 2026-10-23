#include "powertrain_internal.h"

#include <stdlib.h>

#include "cornering_control.h"
#include "default_config.h"
#include "esp_timer.h"
#include "independent_steering.h"
#include "rpm_sensor.h"
#include "servo_output.h"
#include "steering_input_filter.h"

typedef struct {
    bool speed_valid;
    bool speed_held;
    float vehicle_speed_mps;
    float command;
    float ackermann_strength_percent;
    float angle_deg[STEERING_SERVO_COUNT];
} steering_telemetry_t;

static int32_t steering_filtered_q16;
static steering_input_filter_t steering_input_filter;
static steering_telemetry_t steering_telemetry;

static uint16_t clamp_steering_pulse(int32_t pulse_us)
{
    uint16_t minimum_us = controller.steering_cal.left_us < controller.steering_cal.right_us
                              ? controller.steering_cal.left_us
                              : controller.steering_cal.right_us;
    uint16_t maximum_us = controller.steering_cal.left_us > controller.steering_cal.right_us
                              ? controller.steering_cal.left_us
                              : controller.steering_cal.right_us;
    if (pulse_us < minimum_us) return minimum_us;
    if (pulse_us > maximum_us) return maximum_us;
    return (uint16_t)pulse_us;
}

static void update_vehicle_speed_estimate(void)
{
    rpm_snapshot_t rpm = {0};
    float speed_mps = 0.0f;
    if (controller.rpm_initialized) rpm_sensor_get_snapshot(&rpm);
    bool fresh = rpm.updated_at_us > 0 &&
                 esp_timer_get_time() - rpm.updated_at_us <= 100000;
    bool measured = fresh && cornering_vehicle_speed_from_rear_rpm(
        &rpm, controller.drive_config.steering.geometry.wheel_diameter_tenths_mm / 10000.0f, &speed_mps);
    bool had_speed = steering_telemetry.speed_valid || steering_telemetry.speed_held;
    steering_telemetry.speed_valid = measured;
    steering_telemetry.speed_held = !measured && had_speed &&
                                    controller.system_state == SYSTEM_DRIVE_ARMED;
    if (measured) steering_telemetry.vehicle_speed_mps = speed_mps;
    else if (!steering_telemetry.speed_held) steering_telemetry.vehicle_speed_mps = 0.0f;
    steering_telemetry.ackermann_strength_percent = ackermann_strength_percent(
        &controller.drive_config.steering, steering_telemetry.vehicle_speed_mps * 3.6f);
}

void powertrain_steering_update(void)
{
    uint16_t target_us = controller.steering_cal.center_us;
    rc_channel_sample_t sample;
    bool input_valid = false;

    if (!throttle_matches_failsafe() &&
        rc_input_get(RC_CHANNEL_STEERING, &sample)) {
        uint16_t filtered_input_us = controller.steering_cal.center_us;
        bool filter_valid = steering_input_filter_update(&steering_input_filter,
                                                         sample.pulse_us,
                                                         controller.steering_cal.deadband_us,
                                                         sample.updated_at_us,
                                                         &filtered_input_us);
        int32_t center_delta = (int32_t)filtered_input_us - controller.steering_cal.center_us;

        if (!filter_valid || abs(center_delta) <= controller.steering_cal.deadband_us) {
            target_us = controller.steering_cal.center_us;
        } else {
            target_us = clamp_steering_pulse(filtered_input_us);
        }
        input_valid = filter_valid;
    } else {
        steering_input_filter_reset(&steering_input_filter);
    }

    update_vehicle_speed_estimate();
    int32_t target_q16 = (int32_t)target_us << 16;
    if (!input_valid || controller.drive_config.steering_smoothing_ms == 0) {
        steering_filtered_q16 = target_q16;
    } else {
        int32_t difference_q16 = target_q16 - steering_filtered_q16;
        int32_t denominator_ms = controller.drive_config.steering_smoothing_ms +
                                 DRIVE_UPDATE_INTERVAL_MS;
        steering_filtered_q16 +=
            (int32_t)(((int64_t)difference_q16 * DRIVE_UPDATE_INTERVAL_MS) /
                      denominator_ms);
    }

    uint16_t output_us = (uint16_t)((steering_filtered_q16 + (1 << 15)) >> 16);
    output_us = clamp_steering_pulse(output_us);
    int32_t delta = (int32_t)output_us - controller.steering_cal.center_us;
    bool right = pulse_is_on_endpoint_side(output_us, controller.steering_cal.right_us,
                                           controller.steering_cal.center_us);
    int32_t span = abs((int32_t)(right ? controller.steering_cal.right_us : controller.steering_cal.left_us) -
                       controller.steering_cal.center_us);
    steering_telemetry.command = span > 0 ? (float)abs(delta) / span * (right ? 1.0f : -1.0f) : 0.0f;
    uint16_t pulses[STEERING_SERVO_COUNT];
    independent_steering_mix(&controller.drive_config.steering, steering_telemetry.command,
                             steering_telemetry.ackermann_strength_percent, pulses, steering_telemetry.angle_deg);
    servo_output_set_pulses(pulses);
}

void powertrain_steering_recenter(void)
{
    steering_filtered_q16 = (int32_t)controller.steering_cal.center_us << 16;
    steering_input_filter_reset(&steering_input_filter);
    uint16_t centers[STEERING_SERVO_COUNT];
    independent_steering_mix(&controller.drive_config.steering, 0.0f, 0.0f, centers, steering_telemetry.angle_deg);
    steering_telemetry.command = 0.0f;
    servo_output_set_pulses(centers);
}

void powertrain_steering_init(void)
{
    steering_input_filter_init(&steering_input_filter);
    steering_filtered_q16 = (int32_t)controller.steering_cal.center_us << 16;
}

void powertrain_steering_snapshot(powertrain_remote_snapshot_t *snapshot)
{
    servo_output_get_pulses(snapshot->steering_servo_us);
    snapshot->steering_filter_us = steering_input_filter.output_us;
    snapshot->steering_filter_active =
        steering_input_filter_status(&steering_input_filter) ==
        STEERING_INPUT_FILTER_ACTIVE;
    snapshot->steering_rejected_spikes = steering_input_filter.rejected_spike_count;
    snapshot->vehicle_speed_mps = steering_telemetry.vehicle_speed_mps;
    snapshot->vehicle_speed_valid = steering_telemetry.speed_valid;
    snapshot->vehicle_speed_held = steering_telemetry.speed_held;
    snapshot->steering_command = steering_telemetry.command;
    snapshot->ackermann_strength_percent = steering_telemetry.ackermann_strength_percent;
    snapshot->steering_calibration = controller.steering_cal;
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++)
        snapshot->steering_angle_deg[i] = steering_telemetry.angle_deg[i];
}
