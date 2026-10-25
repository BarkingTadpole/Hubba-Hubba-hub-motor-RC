#include "cli.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "powertrain_controller.h"
#include "remote_command.h"

#define CLI_LINE_MAX 384

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
    } else if (strcmp(line, "monitor throttle") == 0 ||
               strcmp(line, "mon throttle") == 0) {
        powertrain_monitor_throttle();
    } else if (strcmp(line, "monitor steering") == 0 ||
               strcmp(line, "mon steering") == 0) {
        powertrain_monitor_steering();
    } else if (strcmp(line, "monitor arm") == 0 ||
               strcmp(line, "mon arm") == 0) {
        powertrain_monitor_arm();
    } else if (strcmp(line, "monitor rpm") == 0 ||
               strcmp(line, "mon rpm") == 0) {
        powertrain_monitor_rpm();
    } else if (strncmp(line, "config ", 7) == 0 ||
               strncmp(line, "cal esc ", 8) == 0 ||
               strncmp(line, "cal wheel ", 10) == 0 || strcmp(line, "cal capture") == 0 ||
               strcmp(line, "cal manual") == 0 || strcmp(line, "cal cancel") == 0 ||
               strcmp(line, "disarm config") == 0) {
        remote_command_t command;
        char error[192];
        if (remote_command_parse(line, &command, error, sizeof(error))) {
            powertrain_apply_command(&command);
        } else {
            printf("ERR: %s\n", error);
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

esp_err_t cli_start(void)
{
    return xTaskCreate(cli_task, "serial_cli", 6144, NULL, 4, NULL) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}
