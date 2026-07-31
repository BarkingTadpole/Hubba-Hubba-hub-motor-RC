#include "cli.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "powertrain_controller.h"

#define CLI_LINE_MAX 160

static void trim_and_lower(char *line)
{
    char *start = line;
    while (isspace((unsigned char)*start)) {
        start++;
    }
    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }

    size_t length = strlen(line);
    while (length > 0 && isspace((unsigned char)line[length - 1])) {
        line[--length] = '\0';
    }
    for (size_t i = 0; line[i] != '\0'; i++) {
        line[i] = (char)tolower((unsigned char)line[i]);
    }
}

static bool parse_unsigned(const char *text, unsigned long *value)
{
    char *end = NULL;
    *value = strtoul(text, &end, 10);
    return text[0] != '\0' && end != NULL && *end == '\0';
}

static void handle_command(char *line)
{
    trim_and_lower(line);
    if (line[0] == '\0') return;

    if (strcmp(line, "status") == 0) {
        powertrain_print_status();
    } else if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        powertrain_print_help();
    } else if (strcmp(line, "arm") == 0) {
        powertrain_arm_from_cli();
    } else if (strcmp(line, "disarm") == 0 || strcmp(line, "stop") == 0) {
        powertrain_disarm();
    } else if (strcmp(line, "cal receiver") == 0) {
        powertrain_calibrate_throttle();
    } else if (strcmp(line, "cal steering") == 0) {
        powertrain_calibrate_steering();
    } else if (strcmp(line, "cal arm") == 0) {
        powertrain_calibrate_arm();
    } else if (strcmp(line, "cal tv") == 0) {
        powertrain_calibrate_tv_mode();
    } else if (strcmp(line, "cal imu") == 0) {
        powertrain_calibrate_imu();
    } else if (strcmp(line, "cal esc arm") == 0) {
        powertrain_cal_esc_arm();
    } else if (strcmp(line, "cal esc max") == 0) {
        powertrain_cal_esc_max();
    } else if (strcmp(line, "cal esc min") == 0) {
        powertrain_cal_esc_min();
    } else if (strcmp(line, "cal manual") == 0) {
        powertrain_cal_manual();
    } else if (strcmp(line, "cal cancel") == 0) {
        powertrain_cal_cancel();
    } else if (strcmp(line, "monitor throttle") == 0 ||
               strcmp(line, "mon throttle") == 0) {
        powertrain_monitor_throttle();
    } else if (strcmp(line, "monitor steering") == 0 ||
               strcmp(line, "mon steering") == 0) {
        powertrain_monitor_steering();
    } else if (strcmp(line, "monitor arm") == 0 ||
               strcmp(line, "mon arm") == 0) {
        powertrain_monitor_arm();
    } else if (strcmp(line, "monitor tv") == 0 ||
               strcmp(line, "mon tv") == 0) {
        powertrain_monitor_tv_mode();
    } else if (strcmp(line, "monitor rpm") == 0 ||
               strcmp(line, "mon rpm") == 0) {
        powertrain_monitor_rpm();
    } else if (strcmp(line, "monitor imu") == 0 ||
               strcmp(line, "mon imu") == 0) {
        powertrain_monitor_imu();
    } else if (strcmp(line, "monitor vector") == 0 ||
               strcmp(line, "mon vector") == 0) {
        powertrain_monitor_vectoring();
    } else if (strcmp(line, "tv enable") == 0) {
        powertrain_set_tv_enabled(true);
    } else if (strcmp(line, "tv disable") == 0) {
        powertrain_set_tv_enabled(false);
    } else if (strncmp(line, "config reverse ", 15) == 0) {
        unsigned long percent = 0;
        if (!parse_unsigned(line + 15, &percent) || percent > 100) {
            printf("ERR: usage is 'config reverse <0-100>'\n");
        } else {
            powertrain_set_reverse_limit((uint8_t)percent);
        }
    } else if (strncmp(line, "config failsafe ", 16) == 0) {
        unsigned int pulse_us = 0;
        unsigned int window_us = 0;
        char extra = '\0';
        if (sscanf(line + 16, "%u %u %c", &pulse_us, &window_us, &extra) != 2) {
            printf("ERR: usage is 'config failsafe <pulse_us> <window_us>'\n");
        } else {
            powertrain_set_failsafe((uint16_t)pulse_us, (uint16_t)window_us);
        }
    } else if (strncmp(line, "config rpm poles ", 17) == 0) {
        unsigned long poles = 0;
        if (!parse_unsigned(line + 17, &poles) || poles > 255) {
            printf("ERR: usage is 'config rpm poles <even 2-60>'\n");
        } else {
            powertrain_set_motor_poles((uint8_t)poles);
        }
    } else if (strncmp(line, "config tv authority ", 20) == 0) {
        unsigned long percent = 0;
        if (!parse_unsigned(line + 20, &percent) || percent > 255) {
            printf("ERR: usage is 'config tv authority <0-25>'\n");
        } else {
            powertrain_set_tv_authority((uint8_t)percent);
        }
    } else if (strncmp(line, "config tv gains ", 16) == 0) {
        float yaw_gain = 0.0f;
        float turn_rpm_gain = 0.0f;
        float yaw_kp = 0.0f;
        float yaw_ki = 0.0f;
        float rpm_kp = 0.0f;
        char extra = '\0';
        int fields = sscanf(line + 16, "%f %f %f %f %f %c",
                            &yaw_gain, &turn_rpm_gain, &yaw_kp, &yaw_ki, &rpm_kp, &extra);
        if (fields != 5) {
            printf("ERR: usage is 'config tv gains <yaw_gain> <turn_rpm_gain> "
                   "<yaw_kp> <yaw_ki> <rpm_kp>'\n");
        } else {
            powertrain_set_tv_gains(yaw_gain, turn_rpm_gain, yaw_kp, yaw_ki, rpm_kp);
        }
    } else if (strncmp(line, "config imu yaw-sign ", 20) == 0) {
        char *end = NULL;
        long sign = strtol(line + 20, &end, 10);
        if (end == line + 20 || *end != '\0' || (sign != -1 && sign != 1)) {
            printf("ERR: usage is 'config imu yaw-sign <-1|1>'\n");
        } else {
            powertrain_set_imu_yaw_sign((int8_t)sign);
        }
    } else {
        printf("ERR: unknown command '%s'. Type 'help'.\n", line);
    }
}

static void cli_task(void *arg)
{
    (void)arg;
    char line[CLI_LINE_MAX];
    size_t length = 0;

    powertrain_print_help();
    printf("> ");
    fflush(stdout);

    while (true) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (length == 0) {
                continue;
            }
            putchar('\n');
            line[length] = '\0';
            handle_command(line);
            length = 0;
            printf("> ");
            fflush(stdout);
            continue;
        }
        if (c == 0x08 || c == 0x7f) {
            if (length > 0) {
                length--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }
        if (isprint((unsigned char)c) && length < CLI_LINE_MAX - 1) {
            line[length++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }
}

void cli_start(void)
{
    xTaskCreate(cli_task, "serial_cli", 6144, NULL, 4, NULL);
}
