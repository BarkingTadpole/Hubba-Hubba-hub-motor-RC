#include "powertrain_internal.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "drivetrain_control.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"
#include "rpm_sensor.h"

#define MONITOR_DURATION_US 30000000
#define MONITOR_INTERVAL_MS 250

static const char *wheel_names[POWERTRAIN_WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};

void powertrain_print_status(void)
{
    powertrain_remote_snapshot_t snapshot;
    if (!powertrain_get_remote_snapshot(&snapshot)) return;
    controller_lock();
    throttle_calibration_t throttle = controller.throttle_cal;
    steering_calibration_t steering = controller.steering_cal;
    arm_calibration_t arm = controller.arm_cal;
    controller_unlock();
    powertrain_timing_snapshot_t timing;
    powertrain_get_timing_snapshot(&timing);
    printf("\nState: %s, drivetrain=%s, arm-cycle=%s, inhibit=0x%lx\n",
           system_state_name(snapshot.state), drivetrain_mode_name(snapshot.config.drivetrain_mode),
           snapshot.arm_cycle_ready ? "ready" : "required", (unsigned long)snapshot.drive_inhibit_flags);
    printf("  arm latch=%s; inhibit events=%lu\n", snapshot.config.permanent_arm_latch_enabled ? "on" : "off",
           (unsigned long)snapshot.drive_inhibit_count);
    printf("  reverse limit=%u%%; tank throttle limit=%u%%; toe=%+.1f deg per wheel (+in/-out)\n",
           snapshot.config.reverse_limit_percent, snapshot.config.tank_limit_percent,
           snapshot.config.steering.toe_tenths_deg / 10.0);
    if (snapshot.config.drivetrain_mode == DRIVETRAIN_TANK) {
        printf("  TANK: CH1 selects spin; either CH2 direction supplies power; front wheels hold toe\n");
        printf("    rear speed is wheel surface speed, not vehicle translation; Ackermann bypassed\n");
    }
    printf("  throttle full/neutral/reverse=%u/%u/%u us (%s)\n",
           throttle.full_throttle_us, throttle.neutral_us, throttle.full_reverse_us,
           throttle.loaded_from_nvs ? "nvs" : "defaults; calibration required");
    printf("  CH1 left/center/right=%u/%u/%u us (%s), smoothing=%u ms\n",
           steering.left_us, steering.center_us, steering.right_us,
           steering.loaded_from_nvs ? "nvs" : "defaults", snapshot.config.steering_smoothing_ms);
    printf("  CH4 RUN/STOP1/STOP2=%u/%u/%u us (%s)\n", arm.run_us, arm.stop_1_us, arm.stop_2_us,
           arm.loaded_from_nvs ? "nvs" : "defaults; calibration required");
    printf("  rear speed=%.2f km/h (%s), Ackermann strength=%.1f%%, command=%+.3f\n",
           (double)(snapshot.vehicle_speed_mps * 3.6f), snapshot.vehicle_speed_valid ? "live" :
           snapshot.vehicle_speed_held ? "held; rear RPM missing" : "unavailable; using 0 km/h point",
           (double)snapshot.ackermann_strength_percent, (double)snapshot.steering_command);
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        const steering_servo_config_t *servo = &snapshot.config.steering.servo[i];
        printf("  %s servo: left/center/right=%u/%u/%u us, output=%u us\n",
               i == STEERING_LEFT ? "left" : "right", servo->left_us, servo->center_us,
               servo->right_us, snapshot.steering_servo_us[i]);
        printf("    trim=%+d us, neutral=%d us (endpoints fixed)\n",
               (int)snapshot.config.steering.trim_us[i],
               (int)servo->center_us + snapshot.config.steering.trim_us[i]);
        printf("    endpoint angles left/right=%.1f/%.1f deg; model target=%+.2f deg (not measured)\n",
               servo->left_tenths_deg / 10.0, servo->right_tenths_deg / 10.0,
               (double)snapshot.steering_angle_deg[i]);
        printf("    wheel angle calibration: %u measured points%s\n",
               snapshot.config.steering.wheel_calibration[i].count,
               snapshot.config.steering.wheel_calibration[i].count ? "" : " (UNMEASURED endpoint approximation)");
    }
    const steering_geometry_t *g = &snapshot.config.steering.geometry;
    const steering_linkage_limits_t *linkage = &snapshot.config.steering.linkage;
    printf("  linkage shaft travel from trimmed neutral: inside=%.1f outside=%.1f deg\n",
           linkage->inside_tenths_deg / 10.0, linkage->outside_tenths_deg / 10.0);
    printf("  geometry: wheelbase=%.1f track=%.1f wheel diameter=%.1f mm; requested left/right=%.1f/%.1f deg\n",
           g->wheelbase_tenths_mm / 10.0, g->track_tenths_mm / 10.0,
           g->wheel_diameter_tenths_mm / 10.0, g->max_left_tenths_deg / 10.0, g->max_right_tenths_deg / 10.0);
    printf("  CH1 deadband=%u us; curve (km/h:Ackermann strength):", steering.deadband_us);
    for (uint16_t i = 0; i < snapshot.config.steering.point_count; i++) {
        ackermann_point_t point = snapshot.config.steering.points[i];
        printf(" %.1f:%+.1f", point.speed_tenths_kmh / 10.0, point.strength_tenths_percent / 10.0);
    }
    printf("\n  steering range=%.1f%%, input=%+.3f, limited=%+.3f; curve (km/h:range):",
           snapshot.steering_limit_percent, snapshot.steering_command, snapshot.steering_limited_command);
    for (uint16_t i = 0; i < snapshot.config.steering_limit.point_count; i++) {
        steering_limit_point_t point = snapshot.config.steering_limit.points[i];
        printf(" %.1f:%.1f", point.speed_tenths_kmh / 10.0, point.limit_tenths_percent / 10.0);
    }
    printf("\n  steering limiter bypasses tank, wheel measurement and unavailable/held speed\n");
    printf("  filter=%s, rejected=%lu, rear RPM=%.1f%s/%.1f%s, PPR=%u\n",
           snapshot.steering_filter_active ? "active" : "warmup", (unsigned long)snapshot.steering_rejected_spikes,
           (double)snapshot.rpm.rpm[0], snapshot.rpm.valid[0] ? "" : "?",
           (double)snapshot.rpm.rpm[1], snapshot.rpm.valid[1] ? "" : "?", snapshot.rpm.pulses_per_revolution);
    printf("  execution max sensor/drive=%lu/%lu us, overruns=%lu/%lu, stack min=%lu/%lu bytes\n",
           (unsigned long)timing.sensor_max, (unsigned long)timing.drive_max, (unsigned long)timing.sensor_overruns,
           (unsigned long)timing.drive_overruns, (unsigned long)timing.sensor_stack, (unsigned long)timing.drive_stack);
    printf("  heap free/min=%lu/%lu bytes\n", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
    printf("  outputs:");
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        printf(" %s=%u/%u", wheel_names[wheel], snapshot.outputs.throttle_us[wheel],
               snapshot.outputs.reverse_us[wheel]);
    }
    printf("\n\n");
}

static void monitor_channel(rc_channel_id_t channel, const char *name)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    printf("Monitoring %s PWM for 30 seconds.\n", name);
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        if (rc_input_get(channel, &sample)) {
            printf("%s: %u us\n", name, sample.pulse_us);
        } else {
            printf("%s: missing\n", name);
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_throttle(void)
{
    monitor_channel(RC_CHANNEL_THROTTLE, "throttle");
}

void powertrain_monitor_steering(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    while (esp_timer_get_time() < end_at_us) {
        powertrain_remote_snapshot_t snapshot;
        if (powertrain_get_remote_snapshot(&snapshot)) {
            printf("CH1=%u%s median=%u (%s) left/right=%u/%u us speed=%.1f km/h (%s) Ackermann strength=%.1f%% rejected=%lu\n",
                   snapshot.steering_input.pulse_us, snapshot.steering_input.valid ? "" : " missing",
                   snapshot.steering_filter_us, snapshot.steering_filter_active ? "active" : "warmup",
                   snapshot.steering_servo_us[0], snapshot.steering_servo_us[1],
                   (double)(snapshot.vehicle_speed_mps * 3.6f), snapshot.vehicle_speed_valid ? "live" :
                   snapshot.vehicle_speed_held ? "held" : "unavailable",
                   (double)snapshot.ackermann_strength_percent, (unsigned long)snapshot.steering_rejected_spikes);
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_arm(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    printf("Monitoring receiver arm/stop PWM for 30 seconds.\n");
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        if (rc_input_get(RC_CHANNEL_ARM, &sample)) {
            controller_lock();
            bool rearm_ready = controller.arm_cycle_ready;
            bool drive_armed = controller.system_state == SYSTEM_DRIVE_ARMED;
            bool permanent_latch = controller.drive_config.permanent_arm_latch_enabled;
            uint32_t inhibit_flags = controller.drive_inhibit_flags;
            const char *position = arm_position_name(sample.pulse_us);
            bool run_requested = arm_requests_run(sample.pulse_us);
            controller_unlock();
            printf("arm: %u us, position=%s, request=%s, "
                   "armed=%s, policy=%s, outputs=%s, initial_cycle=%s\n",
                   sample.pulse_us,
                   position,
                   run_requested ? "RUN" : "STOP",
                   drive_armed ? "yes" : "no",
                   permanent_latch ? "permanent" : "STOP-to-disarm",
                   inhibit_flags == 0 ? "active" : drive_inhibit_reason(inhibit_flags),
                   rearm_ready ? "ready" : "required");
        } else {
            int64_t now_us = esp_timer_get_time();
            printf("arm: missing, last=%u us, age=%lu ms, GPIO%d level=%d\n",
                   sample.pulse_us,
                   (unsigned long)rc_sample_age_ms(&sample, now_us),
                   PIN_RC_ARM_INPUT,
                   gpio_get_level(PIN_RC_ARM_INPUT));
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_rpm(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    while (esp_timer_get_time() < end_at_us) {
        rpm_snapshot_t rpm = {0};
        if (controller.rpm_initialized) rpm_sensor_get_snapshot(&rpm);
        printf("RPM conversion: %u pulses/mechanical-revolution\n",
               rpm.pulses_per_revolution);
        for (size_t wheel = 0; wheel < REAR_RPM_COUNT; wheel++) {
            printf("  %s: rpm=%.1f%s frequency=%.2f Hz edges=%lu window=%lu us\n",
                   wheel_names[wheel + WHEEL_REAR_LEFT], rpm.rpm[wheel],
                   rpm.valid[wheel] ? "" : " missing",
                   rpm.frequency_hz[wheel],
                   (unsigned long)rpm.edge_count[wheel],
                   (unsigned long)rpm.window_us[wheel]);
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_print_help(void)
{
    printf("\nCommands:\n");
    printf("  status | arm | disarm | help\n");
    printf("    disarm holds safe outputs until restart only when arm-latch is on\n");
    printf("    disarm config clears the latch and requires a physical rearm cycle\n");
    printf("  cal receiver | cal steering | cal arm\n");
    printf("  cal esc arm | cal esc max | cal esc min | cal manual | cal cancel\n");
    printf("  monitor throttle | steering | arm | rpm\n");
    printf("  config drivetrain <awd|fwd|rwd|tank>\n");
    printf("  config reverse <0-100>\n");
    printf("  config tank limit <0-100> (tank only, default 25%%; independent of reverse limit)\n");
    printf("    tank: CH1 direction, either CH2 direction power; center both to arm\n");
    printf("  config drive smoothing <0-100> (0=off, 100=smoothest)\n");
    printf("  config failsafe off | <pulse_us> <window_us>\n");
    printf("  config rpm poles <even 2-60>\n");
    printf("  config rpm ppr <1-120>\n");
    printf("  config steering smoothing <0-500>\n");
    printf("  config steering input <left_us> <center_us> <right_us> <deadband_us>\n");
    printf("  config steering servo <left|right> <left_us> <center_us> <right_us> <left_deg> <right_deg>\n");
    printf("    servo PWM 500-2500 us at 50 Hz; 180 deg shaft travel is not road-wheel angle\n");
    printf("    stated mounts: both servos CCW/higher PWM for left, CW/lower PWM for right\n");
    printf("  config steering trim <left|right> <signed_us> (neutral offset; >=50 us inside endpoints)\n");
    printf("  config steering toe <signed_degrees> (per wheel, +in/-out, one decimal; calibrated/linkage range)\n");
    printf("  cal wheel <left|right> | cal capture | cal cancel (physical CH1 measurement, saved CH4 STOP, CH2 neutral, toe 0)\n");
    printf("  config steering wheel <left|right> <max_left_deg> <max_right_deg> <pwm:signed_wheel_deg>... (3-9 points)\n");
    printf("  config steering wheel <left|right> clear (retain endpoint approximation, mark unmeasured)\n");
    printf("  config steering linkage <inside_deg> <outside_deg> (shaft travel from trimmed neutral, 0-90, one decimal)\n");
    printf("  config steering geometry <wheelbase_mm> <track_mm> <wheel_diameter_mm> <max_left_deg> <max_right_deg>\n");
    printf("  config steering ackermann-strength <km/h:strength> ... (2-8 points, first speed 0, strength 0-100%%)\n");
    printf("  config arm-latch <on|off>\n");
    printf("  config steering limit <speed_kmh:range_percent>... (2-8 points, first speed 0; missing RPM disables limiter)\n");
}
