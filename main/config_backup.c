#include "config_backup.h"

#include <string.h>
#include "calibration_validation.h"
#include "drivetrain_control.h"
#include "independent_steering.h"
#include "steering_limit.h"

static const uint8_t magic[8] = {'R','C','V','C','U','C','F','G'};

static uint32_t checksum(const uint8_t *data, size_t size)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

static void put16(uint8_t **cursor, uint16_t value)
{
    *(*cursor)++ = (uint8_t)value;
    *(*cursor)++ = (uint8_t)(value >> 8);
}

static uint16_t get16(const uint8_t **cursor)
{
    uint16_t value = (*cursor)[0] | ((uint16_t)(*cursor)[1] << 8);
    *cursor += 2;
    return value;
}

static void get_values(const uint8_t **cursor, uint16_t *values, size_t count)
{
    for (size_t i = 0; i < count; i++) values[i] = get16(cursor);
}

bool configuration_bundle_valid(const configuration_bundle_t *b)
{
    if (b == NULL) return false;
    const drive_config_t *c = &b->drive;
    return calibration_endpoints_valid(b->throttle.full_throttle_us, b->throttle.neutral_us,
               b->throttle.full_reverse_us) && b->throttle.deadband_us >= 10 && b->throttle.deadband_us <= 150 &&
           calibration_steering_valid(b->input.left_us, b->input.center_us, b->input.right_us, b->input.deadband_us) &&
           calibration_arm_positions_valid(b->arm.run_us, b->arm.stop_1_us, b->arm.stop_2_us) &&
           drivetrain_mode_valid(c->drivetrain_mode) && c->reverse_limit_percent <= 100 &&
           c->tank_limit_percent <= 100 && c->drive_smoothing_percent <= 100 &&
           calibration_pulse_valid(c->receiver_failsafe_us) && c->receiver_failsafe_window_us >= 1 &&
           c->receiver_failsafe_window_us <= 100 && c->motor_poles >= 2 && c->motor_poles <= 60 &&
           c->motor_poles % 2 == 0 && c->rpm_pulses_per_revolution >= 1 && c->rpm_pulses_per_revolution <= 120 &&
           c->steering_smoothing_ms <= 500 && independent_steering_config_valid(&c->steering) &&
           steering_limit_config_valid(&c->steering_limit);
}

bool config_backup_encode(const configuration_bundle_t *b, uint8_t *data, size_t size)
{
    if (data == NULL || size != CONFIG_BACKUP_SIZE || !configuration_bundle_valid(b)) return false;
    memset(data, 0, size);
    memcpy(data, magic, sizeof(magic));
    uint8_t *p = data + 8;
    put16(&p, 1); put16(&p, CONFIG_BACKUP_SIZE - 16);
    p = data + 16;
    put16(&p, (b->throttle.loaded_from_nvs ? 1 : 0) | (b->input.loaded_from_nvs ? 2 : 0) |
              (b->arm.loaded_from_nvs ? 4 : 0));
    put16(&p, b->throttle.full_throttle_us); put16(&p, b->throttle.neutral_us);
    put16(&p, b->throttle.full_reverse_us); put16(&p, b->throttle.deadband_us);
    put16(&p, b->input.left_us); put16(&p, b->input.center_us);
    put16(&p, b->input.right_us); put16(&p, b->input.deadband_us);
    put16(&p, b->arm.run_us); put16(&p, b->arm.stop_1_us); put16(&p, b->arm.stop_2_us);
    const drive_config_t *c = &b->drive;
    put16(&p, c->drivetrain_mode); put16(&p, c->reverse_limit_percent); put16(&p, c->tank_limit_percent);
    put16(&p, c->drive_smoothing_percent); put16(&p, c->receiver_failsafe_enabled);
    put16(&p, c->receiver_failsafe_us); put16(&p, c->receiver_failsafe_window_us);
    put16(&p, c->motor_poles); put16(&p, c->rpm_pulses_per_revolution);
    put16(&p, c->steering_smoothing_ms); put16(&p, c->permanent_arm_latch_enabled);
    const independent_steering_config_t *s = &c->steering;
    put16(&p, s->version);
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        const steering_servo_config_t *servo = &s->servo[i];
        put16(&p, servo->left_us); put16(&p, servo->center_us); put16(&p, servo->right_us);
        put16(&p, servo->left_tenths_deg); put16(&p, servo->right_tenths_deg);
    }
    put16(&p, s->geometry.wheelbase_tenths_mm); put16(&p, s->geometry.track_tenths_mm);
    put16(&p, s->geometry.wheel_diameter_tenths_mm); put16(&p, s->geometry.max_left_tenths_deg);
    put16(&p, s->geometry.max_right_tenths_deg); put16(&p, s->point_count);
    for (size_t i = 0; i < ACKERMANN_MAX_POINTS; i++) {
        put16(&p, i < s->point_count ? s->points[i].speed_tenths_kmh : 0);
        put16(&p, i < s->point_count ? s->points[i].strength_tenths_percent : 0);
    }
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) put16(&p, (uint16_t)s->trim_us[i]);
    put16(&p, s->linkage.inside_tenths_deg); put16(&p, s->linkage.outside_tenths_deg);
    put16(&p, (uint16_t)s->toe_tenths_deg);
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        const wheel_angle_calibration_t *cal = &s->wheel_calibration[i];
        put16(&p, cal->count);
        for (size_t j = 0; j < WHEEL_CALIBRATION_MAX_POINTS; j++) {
            put16(&p, j < cal->count ? cal->points[j].pulse_us : 0);
            put16(&p, j < cal->count ? (uint16_t)cal->points[j].angle_tenths_deg : 0);
        }
    }
    put16(&p, c->steering_limit.version); put16(&p, c->steering_limit.point_count);
    for (size_t i = 0; i < STEERING_LIMIT_MAX_POINTS; i++) {
        put16(&p, i < c->steering_limit.point_count ? c->steering_limit.points[i].speed_tenths_kmh : 0);
        put16(&p, i < c->steering_limit.point_count ? c->steering_limit.points[i].limit_tenths_percent : 0);
    }
    if ((size_t)(p - data) != size) return false;
    uint32_t crc = checksum(data + 16, size - 16);
    for (unsigned i = 0; i < 4; i++) data[12 + i] = (uint8_t)(crc >> (i * 8));
    return true;
}

bool config_backup_decode(const uint8_t *data, size_t size, configuration_bundle_t *b)
{
    if (data == NULL || b == NULL || size != CONFIG_BACKUP_SIZE || memcmp(data, magic, sizeof(magic)) != 0) return false;
    const uint8_t *p = data + 8;
    if (get16(&p) != 1 || get16(&p) != CONFIG_BACKUP_SIZE - 16) return false;
    uint32_t expected_crc = 0;
    for (unsigned i = 0; i < 4; i++) expected_crc |= (uint32_t)data[12 + i] << (i * 8);
    if (checksum(data + 16, size - 16) != expected_crc) return false;
    configuration_bundle_t next = {0};
    p = data + 16;
    uint16_t flags = get16(&p);
    if (flags > 7) return false;
    uint16_t values[11];
    get_values(&p, values, 4);
    next.throttle = (throttle_calibration_t){values[0],values[1],values[2],values[3],flags & 1};
    get_values(&p, values, 4);
    next.input = (steering_calibration_t){values[0],values[1],values[2],values[3],(flags & 2) != 0};
    get_values(&p, values, 3);
    next.arm = (arm_calibration_t){values[0],values[1],values[2],(flags & 4) != 0};
    get_values(&p, values, 11);
    if (values[0] > DRIVETRAIN_TANK || values[1] > 100 || values[2] > 100 || values[3] > 100 ||
        values[4] > 1 || values[7] > 60 || values[10] > 1) return false;
    drive_config_t *c = &next.drive;
    c->drivetrain_mode = (drivetrain_mode_t)values[0]; c->reverse_limit_percent = values[1];
    c->tank_limit_percent = values[2]; c->drive_smoothing_percent = values[3];
    c->receiver_failsafe_enabled = values[4]; c->receiver_failsafe_us = values[5];
    c->receiver_failsafe_window_us = values[6]; c->motor_poles = values[7];
    c->rpm_pulses_per_revolution = values[8]; c->steering_smoothing_ms = values[9];
    c->permanent_arm_latch_enabled = values[10]; c->loaded_from_nvs = true;
    independent_steering_config_t *s = &c->steering;
    s->version = get16(&p);
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        get_values(&p, values, 5);
        s->servo[i] = (steering_servo_config_t){values[0],values[1],values[2],values[3],values[4]};
    }
    get_values(&p, values, 5);
    s->geometry = (steering_geometry_t){values[0],values[1],values[2],values[3],values[4]};
    s->point_count = get16(&p);
    for (size_t i = 0; i < ACKERMANN_MAX_POINTS; i++) {
        get_values(&p, values, 2); s->points[i] = (ackermann_point_t){values[0],values[1]};
    }
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) s->trim_us[i] = (int16_t)get16(&p);
    get_values(&p, values, 2); s->linkage = (steering_linkage_limits_t){values[0],values[1]};
    s->toe_tenths_deg = (int16_t)get16(&p);
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        s->wheel_calibration[i].count = get16(&p);
        for (size_t j = 0; j < WHEEL_CALIBRATION_MAX_POINTS; j++) {
            get_values(&p, values, 2);
            s->wheel_calibration[i].points[j] = (wheel_angle_point_t){values[0],(int16_t)values[1]};
        }
    }
    c->steering_limit.version = get16(&p); c->steering_limit.point_count = get16(&p);
    for (size_t i = 0; i < STEERING_LIMIT_MAX_POINTS; i++) {
        get_values(&p, values, 2);
        c->steering_limit.points[i] = (steering_limit_point_t){values[0],values[1]};
    }
    if ((size_t)(p - data) != size || !configuration_bundle_valid(&next)) return false;
    *b = next;
    return true;
}
