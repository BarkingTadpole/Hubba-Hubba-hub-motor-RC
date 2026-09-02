#include "remote_command.h"

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
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, 10);
    if (text[0] == '\0' || end == NULL || *end != '\0' || parsed > maximum) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
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
    if (strcmp(line, "cal tv") == 0) {
        command->operation = REMOTE_CALIBRATE_TV;
        return true;
    }
    if (strcmp(line, "cal imu") == 0) {
        command->operation = REMOTE_CALIBRATE_IMU;
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
        strcmp(line, "config drivetrain rwd") == 0) {
        command->operation = REMOTE_CONFIG_DRIVETRAIN;
        command->drivetrain_mode = line[18] == 'a' ? DRIVETRAIN_AWD
                                   : line[18] == 'f' ? DRIVETRAIN_FWD
                                                     : DRIVETRAIN_RWD;
        return true;
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
    if (strncmp(line, "config steering trim ", 21) == 0) {
        float trim = 0.0f;
        char extra = '\0';
        if (sscanf(line + 21, "%f %c", &trim, &extra) == 1 &&
            trim >= -15.0f && trim <= 15.0f) {
            command->operation = REMOTE_CONFIG_STEERING_TRIM;
            command->float_values[0] = trim;
            return true;
        }
        set_error(error, error_size, "usage: config steering trim <-15..15>");
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
    if (strcmp(line, "config steering speed-limit on") == 0 ||
        strcmp(line, "config steering speed-limit off") == 0) {
        command->operation = REMOTE_CONFIG_STEERING_SPEED_LIMIT;
        command->enabled = strcmp(line, "config steering speed-limit on") == 0;
        return true;
    }
    if (strncmp(line, "config steering lateral-g ", 26) == 0) {
        float lateral_g = 0.0f;
        char extra = '\0';
        if (sscanf(line + 26, "%f %c", &lateral_g, &extra) == 1 &&
            lateral_g >= 0.2f && lateral_g <= 3.0f) {
            command->operation = REMOTE_CONFIG_STEERING_LATERAL_G;
            command->float_values[0] = lateral_g;
            return true;
        }
        set_error(error, error_size,
                  "usage: config steering lateral-g <0.2-3.0>");
        return false;
    }
    if (strcmp(line, "tv enable") == 0 || strcmp(line, "tv disable") == 0) {
        command->operation = REMOTE_CONFIG_TV_ENABLED;
        command->enabled = strcmp(line, "tv enable") == 0;
        return true;
    }
    if (strncmp(line, "config tv authority ", 20) == 0) {
        command->operation = REMOTE_CONFIG_TV_AUTHORITY;
        if (parse_unsigned(line + 20, 25, &command->unsigned_values[0])) {
            return true;
        }
        set_error(error, error_size, "usage: config tv authority <0-25>");
        return false;
    }
    if (strncmp(line, "config tv front-relief ", 23) == 0) {
        command->operation = REMOTE_CONFIG_TV_FRONT_RELIEF;
        if (parse_unsigned(line + 23, 50, &command->unsigned_values[0])) {
            return true;
        }
        set_error(error, error_size, "usage: config tv front-relief <0-50>");
        return false;
    }
    if (strncmp(line, "config tv gains ", 16) == 0) {
        char extra = '\0';
        int fields = sscanf(line + 16, "%f %f %f %f %f %c",
                            &command->float_values[0], &command->float_values[1],
                            &command->float_values[2], &command->float_values[3],
                            &command->float_values[4], &extra);
        if (fields == 5 && command->float_values[0] >= 0.0f &&
            command->float_values[0] <= 500.0f &&
            command->float_values[1] >= 0.0f && command->float_values[1] <= 1.0f &&
            command->float_values[2] >= 0.0f && command->float_values[2] <= 0.01f &&
            command->float_values[3] >= 0.0f && command->float_values[3] <= 0.01f &&
            command->float_values[4] >= 0.0f && command->float_values[4] <= 5.0f) {
            command->operation = REMOTE_CONFIG_TV_GAINS;
            return true;
        }
        set_error(error, error_size,
                  "usage: config tv gains <yaw 0-500> <turn_rpm 0-1> "
                  "<yaw_kp 0-.01> <yaw_ki 0-.01> <rpm_kp 0-5>");
        return false;
    }
    if (strcmp(line, "config imu yaw-sign -1") == 0 ||
        strcmp(line, "config imu yaw-sign 1") == 0) {
        command->operation = REMOTE_CONFIG_IMU_YAW_SIGN;
        command->signed_value = line[20] == '-' ? -1 : 1;
        return true;
    }
    if (strcmp(line, "config arm-latch on") == 0 ||
        strcmp(line, "config arm-latch off") == 0) {
        command->operation = REMOTE_CONFIG_ARM_LATCH;
        command->enabled = strcmp(line, "config arm-latch on") == 0;
        return true;
    }
    if (strncmp(line, "config logging rate ", 20) == 0) {
        command->operation = REMOTE_CONFIG_LOGGING_RATE;
        if (parse_unsigned(line + 20, 50, &command->unsigned_values[0]) &&
            command->unsigned_values[0] >= 1) {
            return true;
        }
        set_error(error, error_size, "usage: config logging rate <1-50>");
        return false;
    }

    set_error(error, error_size,
              "unsupported command; remote access allows configuration, "
              "calibration, tv enable/disable, and disarm config only");
    return false;
}
