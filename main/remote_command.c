#include "remote_command.h"
#include "independent_steering.h"
#include "calibration_validation.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_error(char *error, size_t error_size, const char *message)
{
    if (error != NULL && error_size > 0) {
        snprintf(error, error_size, "%s", message);
    }
}

static bool parse_unsigned(const char *text, uint32_t maximum, uint32_t *value)
{
    if (text[0] < '0' || text[0] > '9') return false;
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, 10);
    if (text[0] == '\0' || end == NULL || *end != '\0' || parsed > maximum) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static bool parse_steering_trim(const char *text, int16_t *value)
{
    const char *digits = text + (text[0] == '-' || text[0] == '+');
    if (*digits < '0' || *digits > '9') return false;
    char *end;
    errno = 0;
    long parsed = strtol(text, &end, 10);
    if (errno == ERANGE || *end != '\0' ||
        parsed < -STEERING_TRIM_MAX_US || parsed > STEERING_TRIM_MAX_US) return false;
    *value = (int16_t)parsed;
    return true;
}

/* Decode bounded fixed-point fields before narrowing to persisted integers. */
static bool parse_fixed_values(const char *text, uint16_t *values, size_t count, bool tenths)
{
    for (size_t i = 0; i < count; i++) {
        while (isspace((unsigned char)*text)) text++;
        char *end;
        float value = strtof(text, &end);
        float scaled = value * (tenths ? 10.0f : 1.0f);
        if (end == text || (*end != '\0' && !isspace((unsigned char)*end)) ||
            !isfinite(scaled) || scaled < 0.0f || scaled > UINT16_MAX ||
            fabsf(scaled - roundf(scaled)) > 0.001f) return false;
        values[i] = (uint16_t)lroundf(scaled);
        text = end;
    }
    while (isspace((unsigned char)*text)) text++;
    return *text == '\0';
}

static bool parse_wheel_calibration(const char *text, remote_command_t *command)
{
    if (strcmp(text, "clear") == 0) return true;
    uint16_t maxima[2];
    for (size_t i = 0; i < 2; i++) {
        char *end;
        float value = strtof(text, &end) * 10.0f;
        if (end == text || !isspace((unsigned char)*end) || !isfinite(value) ||
            value < 10 || value > 850 || fabsf(value - roundf(value)) > .001f) return false;
        maxima[i] = lroundf(value);
        text = end;
        while (isspace((unsigned char)*text)) text++;
    }
    command->geometry.max_left_tenths_deg = maxima[0];
    command->geometry.max_right_tenths_deg = maxima[1];
    wheel_angle_calibration_t *cal = &command->wheel_calibration;
    while (*text != '\0') {
        if (cal->count >= WHEEL_CALIBRATION_MAX_POINTS || !isdigit((unsigned char)*text)) return false;
        char *end;
        unsigned long pulse = strtoul(text, &end, 10);
        if (*end != ':' || pulse < SERVO_PULSE_MIN_US || pulse > SERVO_PULSE_MAX_US) return false;
        text = end + 1;
        float angle = strtof(text, &end) * 10.0f;
        if (end == text || (*end != '\0' && !isspace((unsigned char)*end)) || !isfinite(angle) ||
            angle < -850 || angle > 850 || fabsf(angle - roundf(angle)) > .001f) return false;
        cal->points[cal->count++] = (wheel_angle_point_t){pulse, lroundf(angle)};
        text = end;
        while (isspace((unsigned char)*text)) text++;
    }
    return cal->count >= 3;
}

bool remote_command_parse(const char *line,
                          remote_command_t *command,
                          char *error,
                          size_t error_size)
{
    if (line == NULL || command == NULL) {
        set_error(error, error_size, "missing command");
        return false;
    }
    memset(command, 0, sizeof(*command));

    if (strcmp(line, "disarm config") == 0) {
        command->operation = REMOTE_DISARM_FOR_CONFIG;
        return true;
    }
    if (strcmp(line, "cal receiver") == 0) {
        command->operation = REMOTE_CALIBRATE_THROTTLE;
        return true;
    }
    if (strcmp(line, "cal steering") == 0) {
        command->operation = REMOTE_CALIBRATE_STEERING;
        return true;
    }
    if (strcmp(line, "cal arm") == 0) {
        command->operation = REMOTE_CALIBRATE_ARM;
        return true;
    }
    if (strcmp(line, "cal wheel left") == 0 || strcmp(line, "cal wheel right") == 0) {
        command->operation = strcmp(line, "cal wheel left") == 0 ? REMOTE_CALIBRATE_WHEEL_LEFT : REMOTE_CALIBRATE_WHEEL_RIGHT;
        return true;
    }
    if (strcmp(line, "cal capture") == 0) {
        command->operation = REMOTE_CALIBRATE_CAPTURE;
        return true;
    }
    if (strcmp(line, "cal esc arm") == 0) {
        command->operation = REMOTE_CALIBRATE_ESC_ARM;
        return true;
    }
    if (strcmp(line, "cal esc max") == 0) {
        command->operation = REMOTE_CALIBRATE_ESC_MAX;
        return true;
    }
    if (strcmp(line, "cal esc min") == 0) {
        command->operation = REMOTE_CALIBRATE_ESC_MIN;
        return true;
    }
    if (strcmp(line, "cal manual") == 0) {
        command->operation = REMOTE_CALIBRATE_ESC_MANUAL;
        return true;
    }
    if (strcmp(line, "cal cancel") == 0) {
        command->operation = REMOTE_CALIBRATE_CANCEL;
        return true;
    }
    if (strcmp(line, "config drivetrain awd") == 0 ||
        strcmp(line, "config drivetrain fwd") == 0 ||
        strcmp(line, "config drivetrain rwd") == 0 ||
        strcmp(line, "config drivetrain tank") == 0) {
        command->operation = REMOTE_CONFIG_DRIVETRAIN;
        command->drivetrain_mode = line[18] == 'a' ? DRIVETRAIN_AWD
                                   : line[18] == 'f' ? DRIVETRAIN_FWD
                                   : line[18] == 'r' ? DRIVETRAIN_RWD : DRIVETRAIN_TANK;
        return true;
    }
    if (strncmp(line, "config tank limit ", 18) == 0) {
        command->operation = REMOTE_CONFIG_TANK_LIMIT;
        if (parse_unsigned(line + 18, 100, &command->unsigned_values[0])) return true;
        set_error(error, error_size, "usage: config tank limit <0-100>");
        return false;
    }
    if (strncmp(line, "config reverse ", 15) == 0) {
        command->operation = REMOTE_CONFIG_REVERSE_LIMIT;
        if (parse_unsigned(line + 15, 100, &command->unsigned_values[0])) {
            return true;
        }
        set_error(error, error_size, "usage: config reverse <0-100>");
        return false;
    }
    if (strncmp(line, "config drive smoothing ", 23) == 0) {
        command->operation = REMOTE_CONFIG_DRIVE_SMOOTHING;
        if (parse_unsigned(line + 23, 100, &command->unsigned_values[0])) {
            return true;
        }
        set_error(error, error_size,
                  "usage: config drive smoothing <0-100>");
        return false;
    }
    if (strcmp(line, "config failsafe off") == 0) {
        command->operation = REMOTE_CONFIG_FAILSAFE_ENABLED;
        command->enabled = false;
        return true;
    }
    if (strncmp(line, "config failsafe ", 16) == 0) {
        unsigned int pulse_us = 0;
        unsigned int window_us = 0;
        char extra = '\0';
        if (sscanf(line + 16, "%u %u %c", &pulse_us, &window_us, &extra) == 2 &&
            pulse_us >= 800 && pulse_us <= 2200 &&
            window_us >= 1 && window_us <= 100) {
            command->operation = REMOTE_CONFIG_FAILSAFE;
            command->unsigned_values[0] = pulse_us;
            command->unsigned_values[1] = window_us;
            return true;
        }
        set_error(error, error_size,
                  "usage: config failsafe off | <800-2200> <1-100>");
        return false;
    }
    if (strncmp(line, "config rpm poles ", 17) == 0) {
        uint32_t poles = 0;
        if (parse_unsigned(line + 17, 60, &poles) && poles >= 2 &&
            (poles % 2) == 0) {
            command->operation = REMOTE_CONFIG_RPM_POLES;
            command->unsigned_values[0] = poles;
            return true;
        }
        set_error(error, error_size, "usage: config rpm poles <even 2-60>");
        return false;
    }
    if (strncmp(line, "config rpm ppr ", 15) == 0) {
        uint32_t ppr = 0;
        if (parse_unsigned(line + 15, 120, &ppr) && ppr >= 1) {
            command->operation = REMOTE_CONFIG_RPM_PPR;
            command->unsigned_values[0] = ppr;
            return true;
        }
        set_error(error, error_size, "usage: config rpm ppr <1-120>");
        return false;
    }
    if (strncmp(line, "config steering smoothing ", 26) == 0) {
        command->operation = REMOTE_CONFIG_STEERING_SMOOTHING;
        if (parse_unsigned(line + 26, 500, &command->unsigned_values[0])) {
            return true;
        }
        set_error(error, error_size,
                  "usage: config steering smoothing <0-500>");
        return false;
    }
    if (strncmp(line, "config steering wheel ", 22) == 0) {
        const char *text = line + 22;
        bool left = strncmp(text, "left ", 5) == 0, right = strncmp(text, "right ", 6) == 0;
        if ((left || right) && parse_wheel_calibration(text + (left ? 5 : 6), command)) {
            command->operation = REMOTE_CONFIG_WHEEL_CALIBRATION;
            command->unsigned_values[0] = left ? STEERING_LEFT : STEERING_RIGHT;
            return true;
        }
        set_error(error, error_size, "use config steering wheel <left|right> <max_left_deg> <max_right_deg> <pwm:signed_wheel_deg>... (3-9 points), or <left|right> clear");
        return false;
    }
    if (strncmp(line, "config steering toe ", 20) == 0) {
        const char *cursor = line + 20;
        char *end;
        float value = strtof(cursor, &end), tenths = value * 10.0f;
        if (end != cursor && *end == '\0' && isfinite(tenths) &&
            tenths >= INT16_MIN && tenths <= INT16_MAX &&
            fabsf(tenths - roundf(tenths)) <= 0.001f) {
            command->operation = REMOTE_CONFIG_STEERING_TOE;
            command->toe_tenths_deg = (int16_t)lroundf(tenths);
            return true;
        }
        set_error(error, error_size, "use config steering toe <signed_degrees>; per wheel, +in/-out, one decimal; must fit calibrated/linkage travel");
        return false;
    }
    if (strncmp(line, "config steering trim ", 21) == 0) {
        const char *cursor = line + 21;
        bool left = strncmp(cursor, "left ", 5) == 0;
        bool right = strncmp(cursor, "right ", 6) == 0;
        if ((left || right) && parse_steering_trim(cursor + (left ? 5 : 6), &command->steering_trim_us)) {
            command->operation = REMOTE_CONFIG_STEERING_TRIM;
            command->unsigned_values[0] = left ? STEERING_LEFT : STEERING_RIGHT;
            return true;
        }
        set_error(error, error_size, "use config steering trim <left|right> <signed_us>; integer -1900..1900, trimmed center must stay >=50 us inside endpoints");
        return false;
    }
    if (strncmp(line, "config steering servo ", 22) == 0) {
        const char *cursor = line + 22;
        bool left = strncmp(cursor, "left ", 5) == 0;
        bool right = strncmp(cursor, "right ", 6) == 0;
        uint16_t values[5];
        /* PWM fields are integers; angle fields allow one decimal. */
        if ((left || right) && parse_fixed_values(cursor + (left ? 5 : 6), values, 5, true) &&
            values[0] % 10 == 0 && values[1] % 10 == 0 && values[2] % 10 == 0) {
            command->servo = (steering_servo_config_t){
                values[0] / 10, values[1] / 10, values[2] / 10, values[3], values[4]};
            if (steering_servo_config_valid(&command->servo)) {
                command->operation = REMOTE_CONFIG_STEERING_SERVO;
                command->unsigned_values[0] = left ? STEERING_LEFT : STEERING_RIGHT;
                return true;
            }
        }
        set_error(error, error_size, "use config steering servo <left|right> <left_us> <center_us> <right_us> <left_deg> <right_deg>; PWM 500-2500, road-wheel angles 1-85");
        return false;
    }
    if (strncmp(line, "config steering linkage ", 24) == 0) {
        uint16_t values[2];
        if (parse_fixed_values(line + 24, values, 2, true)) {
            command->linkage = (steering_linkage_limits_t){values[0], values[1]};
            if (steering_linkage_limits_valid(&command->linkage)) {
                command->operation = REMOTE_CONFIG_STEERING_LINKAGE;
                return true;
            }
        }
        set_error(error, error_size, "use config steering linkage <inside_deg> <outside_deg>; shaft travel from trimmed neutral, 0-90 degrees, one decimal");
        return false;
    }
    if (strncmp(line, "config steering geometry ", 25) == 0) {
        uint16_t values[5];
        if (parse_fixed_values(line + 25, values, 5, true)) {
            command->geometry = (steering_geometry_t){values[0], values[1], values[2], values[3], values[4]};
            if (steering_geometry_valid(&command->geometry)) {
                command->operation = REMOTE_CONFIG_STEERING_GEOMETRY;
                return true;
            }
        }
        set_error(error, error_size, "use config steering geometry <wheelbase_mm> <track_mm> <diameter_mm> <max_left_deg> <max_right_deg>; 50-2000 / 50-2000 / 10-500 mm, angles 1-85, one decimal");
        return false;
    }
    if (strncmp(line, "config steering input ", 22) == 0) {
        uint16_t values[4];
        if (parse_fixed_values(line + 22, values, 4, false) &&
            calibration_steering_valid(values[0], values[1], values[2], values[3])) {
            command->operation = REMOTE_CONFIG_STEERING_INPUT;
            command->steering_input = (steering_calibration_t){
                .left_us = values[0], .center_us = values[1], .right_us = values[2], .deadband_us = values[3]};
            return true;
        }
        set_error(error, error_size, "use config steering input <left_us> <center_us> <right_us> <deadband_us>; 800-2200, center bracketed >=150us, deadband 0-150 and less than each span");
        return false;
    }
    if (strncmp(line, "config steering ackermann-strength ", 35) == 0) {
        const char *cursor = line + 35;
        while (*cursor != '\0') {
            if (command->point_count >= ACKERMANN_MAX_POINTS) goto invalid_ackermann;
            char *end;
            float speed = strtof(cursor, &end);
            if (end == cursor || *end != ':' || !isfinite(speed) || speed < 0 || speed > 300) goto invalid_ackermann;
            cursor = end + 1;
            float percent = strtof(cursor, &end);
            if (end == cursor || !isfinite(percent) || percent < 0 || percent > 100 ||
                (*end != '\0' && !isspace((unsigned char)*end))) goto invalid_ackermann;
            /* Reject extra precision rather than silently changing a user's curve. */
            float speed10 = speed * 10.0f, percent10 = percent * 10.0f;
            if (fabsf(speed10 - roundf(speed10)) > 0.001f ||
                fabsf(percent10 - roundf(percent10)) > 0.001f) goto invalid_ackermann;
            command->points[command->point_count++] = (ackermann_point_t){
                (uint16_t)lroundf(speed10), (uint16_t)lroundf(percent10),
            };
            cursor = end;
            while (isspace((unsigned char)*cursor)) cursor++;
        }
        if (*cursor == '\0' && ackermann_curve_valid(command->points, command->point_count)) {
            command->operation = REMOTE_CONFIG_ACKERMANN;
            return true;
        }
invalid_ackermann:
        set_error(error, error_size, "use 2-8 speed:percent points, first speed 0, ascending speeds 0-300 km/h, Ackermann strength 0..100, at most one decimal");
        return false;
    }
    if (strcmp(line, "config arm-latch on") == 0 ||
        strcmp(line, "config arm-latch off") == 0) {
        command->operation = REMOTE_CONFIG_ARM_LATCH;
        command->enabled = strcmp(line, "config arm-latch on") == 0;
        return true;
    }
    set_error(error, error_size,
              "unsupported command; remote access allows configuration, "
              "calibration, and disarm config only");
    return false;
}
