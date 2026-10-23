"""Link the actual powertrain modules against simulated I/O, time and RTOS services.
The C harness steps the production control task, without ESP32 hardware.
"""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
STUBS = {
    'esp_err.h': '''#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL 1
#define ESP_ERR_NO_MEM 2
const char *esp_err_to_name(esp_err_t err);
''',
    'esp_timer.h': '#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n',
    'esp_log.h': '#pragma once\n#define ESP_LOGW(tag, ...) ((void)(tag))\n',
    'esp_heap_caps.h': '''#pragma once
#include <stddef.h>
#define MALLOC_CAP_8BIT 0
size_t heap_caps_get_free_size(int caps);
size_t heap_caps_get_minimum_free_size(int caps);
''',
    'freertos/FreeRTOS.h': '''#pragma once
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define configASSERT(x) assert(x)
#define pdMS_TO_TICKS(x) (x)
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
''',
    'freertos/semphr.h': '''#pragma once
#include "FreeRTOS.h"
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void);
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t mutex, TickType_t wait);
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t mutex);
''',
    'freertos/task.h': '''#pragma once
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskDelayUntil(TickType_t *last, TickType_t ticks);
void vTaskDelay(TickType_t ticks);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task);
BaseType_t xTaskCreate(void (*task)(void *), const char *name, uint32_t stack,
                      void *arg, UBaseType_t priority, TaskHandle_t *handle);
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *), const char *name, uint32_t stack,
                      void *arg, UBaseType_t priority, TaskHandle_t *handle, int core);
''',
    'driver/gpio.h': '#pragma once\ntypedef int gpio_num_t;\nint gpio_get_level(gpio_num_t gpio);\n' +
                     ''.join(f'#define GPIO_NUM_{i} {i}\n' for i in range(40)),
}

with tempfile.TemporaryDirectory() as directory:
    temp = Path(directory)
    for name, content in STUBS.items():
        target = temp / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    executable = temp / 'test_controller'
    modules = ['powertrain_commands', 'remote_command', 'powertrain_calibration', 'powertrain_settings', 'powertrain_diagnostics',
               'powertrain_steering', 'independent_steering', 'steering_input_filter',
               'cornering_control', 'drivetrain_control']
    subprocess.run([*shlex.split(os.environ.get('CC', 'cc')), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(temp), '-I', str(ROOT/'main'), '-I', str(ROOT/'main/include'),
                    str(ROOT/'tests/host/test_controller.c'),
                    *[str(ROOT/'main'/f'{module}.c') for module in modules],
                    '-lm', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
