#include "powertrain_internal.h"

#include <stdio.h>
#include <string.h>

#include "config_store.h"
#include "calibration_validation.h"
#include "drivetrain_control.h"
#include "independent_steering.h"
#include "rpm_sensor.h"

static esp_err_t save_drive_config_or_report(const char *success_message)
{
    esp_err_t err = config_store_save_drive(&controller.drive_config);
    if (err == ESP_OK) {
        controller.drive_config.loaded_from_nvs = true;
        printf("%s\n", success_message);
    } else {
        printf("ERR: failed to save drive configuration: %s\n", esp_err_to_name(err));
    }
    return err;
}

static bool configuration_is_allowed(void)
{
    return controller.system_state == SYSTEM_DISARMED && !controller.calibration_in_progress;
}

bool powertrain_set_drivetrain_mode(drivetrain_mode_t mode)
{
    controller_lock();
    if (!configuration_is_allowed() || !drivetrain_mode_valid(mode)) {
        printf("ERR: drivetrain mode must be AWD, FWD, RWD, or TANK while DISARMED\n");
        controller_unlock();
        return false;
    }

    drivetrain_mode_t previous_mode = controller.drive_config.drivetrain_mode;
    controller.drive_config.drivetrain_mode = mode;
    char message[64];
    snprintf(message, sizeof(message), "OK: drivetrain mode set to %s",
             drivetrain_mode_name(mode));
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    if (!saved) {
        controller.drive_config.drivetrain_mode = previous_mode;
    } else {
        reset_drive_state();
        powertrain_steering_recenter();
    }
    controller_unlock();
    return saved;
}

bool powertrain_set_tank_limit(uint8_t percent)
{
    controller_lock();
    if (!configuration_is_allowed() || percent > 100) {
        printf("ERR: tank limit must be 0-100 and changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    uint8_t previous = controller.drive_config.tank_limit_percent;
    controller.drive_config.tank_limit_percent = percent;
    char message[80];
    snprintf(message, sizeof(message), "OK: tank throttle limit set to %u%%", percent);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    if (!saved) controller.drive_config.tank_limit_percent = previous;
    controller_unlock();
    return saved;
}

bool powertrain_set_reverse_limit(uint8_t percent)
{
    controller_lock();
    if (!configuration_is_allowed() || percent > 100) {
        printf("ERR: reverse limit must be 0-100 and changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    controller.drive_config.reverse_limit_percent = percent;
    char message[64];
    snprintf(message, sizeof(message), "OK: reverse limit set to %u%%", percent);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_drive_smoothing(uint8_t percent)
{
    controller_lock();
    if (!configuration_is_allowed() || percent > 100) {
        printf("ERR: drive smoothing must be 0-100 and changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    controller.drive_config.drive_smoothing_percent = percent;
    char message[96];
    if (percent == 0) {
        snprintf(message, sizeof(message),
                 "OK: acceleration/deceleration ramp disabled");
    } else {
        snprintf(message, sizeof(message),
                 "OK: drive smoothing set to %u%%", percent);
    }
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_failsafe(uint16_t pulse_us, uint16_t window_us)
{
    controller_lock();
    if (!configuration_is_allowed() || pulse_us < 800 || pulse_us > 2200 ||
        window_us < 1 || window_us > 100) {
        printf("ERR: failsafe pulse must be 800-2200 us, window 1-100 us, while DISARMED\n");
        controller_unlock();
        return false;
    }
    controller.drive_config.receiver_failsafe_us = pulse_us;
    controller.drive_config.receiver_failsafe_window_us = window_us;
    controller.drive_config.receiver_failsafe_enabled = true;
    char message[96];
    snprintf(message, sizeof(message), "OK: throttle failsafe set to %u +/- %u us",
             pulse_us, window_us);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_failsafe_enabled(bool enabled)
{
    controller_lock();
    if (!configuration_is_allowed()) {
        printf("ERR: throttle-pulse failsafe can only be changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    controller.drive_config.receiver_failsafe_enabled = enabled;
    bool saved = save_drive_config_or_report(
                     enabled ? "OK: throttle-pulse failsafe enabled"
                             : "OK: throttle-pulse failsafe disabled") == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_motor_poles(uint8_t poles)
{
    controller_lock();
    if (!configuration_is_allowed() || poles < 2 || poles > 60 || (poles % 2) != 0) {
        printf("ERR: motor pole count must be even, 2-60, and changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    controller.drive_config.motor_poles = poles;
    controller.drive_config.rpm_pulses_per_revolution = poles / 2;
    if (controller.rpm_initialized) {
        rpm_sensor_set_pulses_per_revolution(controller.drive_config.rpm_pulses_per_revolution);
    }
    char message[160];
    snprintf(message, sizeof(message),
             "OK: motor poles=%u and RPM PPR=%u (legacy one-pulse-per-electrical-revolution assumption)",
             poles, controller.drive_config.rpm_pulses_per_revolution);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_rpm_pulses_per_revolution(uint16_t pulses_per_revolution)
{
    controller_lock();
    if (!configuration_is_allowed() || pulses_per_revolution < 1 ||
        pulses_per_revolution > 120) {
        printf("ERR: RPM pulses per mechanical revolution must be 1-120 while DISARMED\n");
        controller_unlock();
        return false;
    }
    controller.drive_config.rpm_pulses_per_revolution = pulses_per_revolution;
    if (controller.rpm_initialized) {
        rpm_sensor_set_pulses_per_revolution(pulses_per_revolution);
    }
    char message[96];
    snprintf(message, sizeof(message),
             "OK: RPM conversion set to %u pulses per mechanical revolution",
             pulses_per_revolution);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_smoothing(uint16_t smoothing_ms)
{
    controller_lock();
    if (!configuration_is_allowed() || smoothing_ms > 500) {
        printf("ERR: steering smoothing must be 0-500 ms while DISARMED\n");
        controller_unlock();
        return false;
    }
    uint16_t previous = controller.drive_config.steering_smoothing_ms;
    controller.drive_config.steering_smoothing_ms = smoothing_ms;
    char message[96];
    snprintf(message, sizeof(message), "OK: steering smoothing time constant set to %u ms",
             smoothing_ms);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    if (!saved) controller.drive_config.steering_smoothing_ms = previous;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_servo(uint8_t side, const steering_servo_config_t *servo)
{
    controller_lock();
    if (!configuration_is_allowed() || side >= STEERING_SERVO_COUNT ||
        !steering_servo_config_valid(servo)) {
        printf("ERR: servo endpoints must bracket center by at least 50 us, within 500-2500 us, while DISARMED\n");
        controller_unlock();
        return false;
    }
    independent_steering_config_t previous = controller.drive_config.steering;
    controller.drive_config.steering.servo[side] = *servo;
    bool saved = independent_steering_config_valid(&controller.drive_config.steering) &&
                 save_drive_config_or_report("OK: steering servo PWM/angle calibration saved") == ESP_OK;
    if (!saved) printf("ERR: requested angles, trimmed neutral and toe must fit calibrated/linkage travel; configuration not changed\n");
    if (!saved) controller.drive_config.steering = previous;
    controller_unlock();
    return saved;
}

bool powertrain_set_wheel_calibration(uint8_t side, const wheel_angle_calibration_t *calibration,
                                     uint16_t max_left, uint16_t max_right)
{
    controller_lock();
    if (!configuration_is_allowed() || side >= STEERING_SERVO_COUNT || calibration == NULL ||
        (calibration->count != 0 && (calibration->count < 3 || calibration->count > WHEEL_CALIBRATION_MAX_POINTS))) {
        controller_unlock();
        return false;
    }
    independent_steering_config_t previous = controller.drive_config.steering, next = previous;
    next.wheel_calibration[side] = *calibration;
    if (calibration->count > 0) {
        const wheel_angle_point_t *first = &calibration->points[0], *last = &calibration->points[calibration->count - 1];
        next.servo[side].left_us = first->pulse_us;
        next.servo[side].right_us = last->pulse_us;
        next.servo[side].left_tenths_deg = -first->angle_tenths_deg;
        next.servo[side].right_tenths_deg = last->angle_tenths_deg;
        next.geometry.max_left_tenths_deg = max_left;
        next.geometry.max_right_tenths_deg = max_right;
        /* Measurements cannot silently reverse the physical direction mapping. */
        if ((next.servo[side].left_us > next.servo[side].right_us) !=
            (previous.servo[side].left_us > previous.servo[side].right_us)) {
            controller_unlock();
            return false;
        }
    }
    if (!independent_steering_config_valid(&next)) {
        printf("ERR: wheel measurements must be monotonic, include trimmed neutral at 0 degrees, and fit geometry/toe\n");
        controller_unlock();
        return false;
    }
    controller.drive_config.steering = next;
    bool saved = save_drive_config_or_report(calibration->count ? "OK: measured wheel calibration saved" :
                                             "OK: wheel measurements cleared; endpoint approximation retained") == ESP_OK;
    if (!saved) controller.drive_config.steering = previous;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_trim(uint8_t side, int16_t trim_us)
{
    controller_lock();
    if (!configuration_is_allowed() || side >= STEERING_SERVO_COUNT) {
        printf("ERR: select left/right servo trim while DISARMED and outside calibration\n");
        controller_unlock();
        return false;
    }
    independent_steering_config_t next = controller.drive_config.steering;
    next.trim_us[side] = trim_us;
    if (!independent_steering_config_valid(&next)) {
        printf("ERR: center + trim must stay >=50 us inside endpoints and configured toe must fit linkage travel\n");
        controller_unlock();
        return false;
    }
    int16_t previous = controller.drive_config.steering.trim_us[side];
    controller.drive_config.steering.trim_us[side] = trim_us;
    char message[96];
    snprintf(message, sizeof(message), "OK: %s servo trim set to %+d us (neutral %d us)",
             side == STEERING_LEFT ? "left" : "right", (int)trim_us,
             (int)next.servo[side].center_us + trim_us);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    if (!saved) controller.drive_config.steering.trim_us[side] = previous;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_linkage(const steering_linkage_limits_t *limits)
{
    controller_lock();
    if (!configuration_is_allowed() || !steering_linkage_limits_valid(limits)) {
        printf("ERR: linkage travel must be 0-90 degrees from trimmed neutral, while DISARMED and outside calibration\n");
        controller_unlock();
        return false;
    }
    independent_steering_config_t next = controller.drive_config.steering;
    next.linkage = *limits;
    if (!independent_steering_config_valid(&next)) {
        printf("ERR: linkage limits must also contain configured toe; configuration not changed\n");
        controller_unlock();
        return false;
    }
    steering_linkage_limits_t previous = controller.drive_config.steering.linkage;
    controller.drive_config.steering.linkage = *limits;
    bool saved = save_drive_config_or_report("OK: inside/outside linkage travel limits saved") == ESP_OK;
    if (!saved) controller.drive_config.steering.linkage = previous;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_toe(int16_t tenths_deg)
{
    controller_lock();
    independent_steering_config_t next = controller.drive_config.steering;
    next.toe_tenths_deg = tenths_deg;
    if (!configuration_is_allowed() || !independent_steering_config_valid(&next)) {
        printf("ERR: toe must fit both wheels' calibrated/linkage travel, while DISARMED\n");
        controller_unlock();
        return false;
    }
    int16_t previous = controller.drive_config.steering.toe_tenths_deg;
    controller.drive_config.steering.toe_tenths_deg = tenths_deg;
    char message[96];
    snprintf(message, sizeof(message), "OK: toe set to %+.1f degrees per wheel (+in, -out)", tenths_deg / 10.0);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    if (!saved) controller.drive_config.steering.toe_tenths_deg = previous;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_geometry(const steering_geometry_t *geometry)
{
    controller_lock();
    if (!configuration_is_allowed() || !steering_geometry_valid(geometry)) {
        printf("ERR: invalid steering geometry or controller not DISARMED\n");
        controller_unlock();
        return false;
    }
    steering_geometry_t previous = controller.drive_config.steering.geometry;
    controller.drive_config.steering.geometry = *geometry;
    bool saved = independent_steering_config_valid(&controller.drive_config.steering) &&
                 save_drive_config_or_report("OK: steering geometry saved") == ESP_OK;
    if (!saved) {
        controller.drive_config.steering.geometry = previous;
        printf("ERR: requested steering limits must fit both servos; configuration not changed\n");
    }
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_input(const steering_calibration_t *calibration)
{
    controller_lock();
    if (!configuration_is_allowed() || calibration == NULL ||
        !calibration_steering_valid(calibration->left_us, calibration->center_us,
                                    calibration->right_us, calibration->deadband_us)) {
        printf("ERR: invalid steering input calibration or controller not DISARMED\n");
        controller_unlock();
        return false;
    }
    bool saved = config_store_save_steering(calibration) == ESP_OK;
    if (saved) {
        controller.steering_cal = *calibration;
        controller.steering_cal.loaded_from_nvs = true;
        powertrain_steering_recenter();
        printf("OK: steering input calibration saved\n");
    } else printf("ERR: failed to save steering input calibration\n");
    controller_unlock();
    return saved;
}

bool powertrain_set_ackermann_curve(const ackermann_point_t *points, uint16_t count)
{
    controller_lock();
    if (!configuration_is_allowed() || !ackermann_curve_valid(points, count)) {
        printf("ERR: use 2-8 ascending speed points, starting at 0, speeds 0-300 km/h and strengths 0..100 while DISARMED\n");
        controller_unlock();
        return false;
    }
    independent_steering_config_t previous = controller.drive_config.steering;
    memset(controller.drive_config.steering.points, 0, sizeof(controller.drive_config.steering.points));
    memcpy(controller.drive_config.steering.points, points, count * sizeof(*points));
    controller.drive_config.steering.point_count = count;
    bool saved = save_drive_config_or_report("OK: complete Ackermann curve saved") == ESP_OK;
    if (!saved) controller.drive_config.steering = previous;
    controller_unlock();
    return saved;
}

bool powertrain_set_permanent_arm_latch_enabled(bool enabled)
{
    controller_lock();
    if (!configuration_is_allowed()) {
        printf("ERR: permanent arm latch can only be changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    bool previous = controller.drive_config.permanent_arm_latch_enabled;
    controller.drive_config.permanent_arm_latch_enabled = enabled;
    bool saved = save_drive_config_or_report(
                     enabled
                         ? "OK: permanent arm latch enabled"
                         : "OK: permanent arm latch disabled; safety events will disarm") ==
                 ESP_OK;
    if (!saved) {
        controller.drive_config.permanent_arm_latch_enabled = previous;
    }
    controller_unlock();
    return saved;
}
