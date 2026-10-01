"""Run the production logger lifecycle/worker with host I/O and deterministic RTOS stubs.

CSV sensor formatting is stubbed; open/write/stop/flush/read/clear code is taken
unchanged from telemetry_log.c. No ESP32 or flash hardware is exercised.
Set ZIG to a Zig executable, or CC to a native C compiler.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "main/telemetry_log.c").read_text(encoding="utf-8")
status_header = (ROOT / "main/include/telemetry_log.h").read_text(encoding="utf-8")
implementation = source[source.index("static esp_err_t open_log_file_locked"):
                        source.index("\n#else\n")]
defines = "\n".join(re.findall(r"^#define LOG_.*$", source, re.M))
harness = r'''
#include <assert.h>
#include <errno.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "telemetry_log.h"
DEFINES
#undef LOG_BASE_PATH
#define LOG_BASE_PATH "."
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define DEFAULT_TELEMETRY_LOG_RATE_HZ 1
#define WL_INVALID_HANDLE -1
#define portMAX_DELAY 0
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
typedef int wl_handle_t;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;
typedef struct { bool format_if_mount_failed; int max_files; int allocation_unit_size; } esp_vfs_fat_mount_config_t;
#define VFS_FAT_MOUNT_DEFAULT_CONFIG() ((esp_vfs_fat_mount_config_t){0})
typedef struct { struct { uint8_t telemetry_log_rate_hz; } config; } powertrain_remote_snapshot_t;
static bool powertrain_get_remote_snapshot(powertrain_remote_snapshot_t *s) { s->config.telemetry_log_rate_hz=1; return true; }
static int64_t clock_us;
static uint64_t storage_free = 1000000;
static bool locked, sync_failure, create_failure;
static unsigned sync_calls, format_calls, task_ticks, tick_limit;
static jmp_buf task_exit;
static int64_t esp_timer_get_time(void) { return clock_us; }
static uint32_t esp_random(void) { return 42; }
static SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
static void xSemaphoreTake(void *m, int wait) { (void)m; (void)wait; assert(!locked); locked=true; }
static void xSemaphoreGive(void *m) { (void)m; assert(locked); locked=false; }
static int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, int stack, void *arg, int priority, void **handle, int core) {
    (void)fn; (void)name; (void)stack; (void)arg; (void)priority; (void)core;
    if (create_failure) return 0;
    *handle=(void *)1; return pdPASS;
}
static void vTaskDelay(int ticks) { clock_us += (int64_t)ticks*1000; if (++task_ticks > tick_limit) longjmp(task_exit, 1); }
static int esp_vfs_fat_info(const char *base, uint64_t *total, uint64_t *free_bytes) { (void)base; *total=2000000; *free_bytes=storage_free; return ESP_OK; }
static int esp_vfs_fat_spiflash_mount_rw_wl(const char *base, const char *label, const void *config, int *handle) {
    (void)base; (void)label; (void)config; *handle=1; return ESP_OK;
}
static int esp_vfs_fat_spiflash_format_rw_wl(const char *base, const char *label) { (void)base; (void)label; storage_free=1000000; return ESP_OK; }
static int test_fsync(int fd) { (void)fd; sync_calls++; if (sync_failure) {errno=EIO; return -1;} return 0; }
#define fsync test_fsync
static SemaphoreHandle_t log_mutex;
static FILE *log_file;
static wl_handle_t wl_handle = WL_INVALID_HANDLE;
static TaskHandle_t log_task_handle;
static char log_stdio_buffer[LOG_STDIO_BUFFER_SIZE];
static telemetry_log_status_t log_status;
static uint32_t recording_generation;
static const char CSV_HEADER[] = "schema,boot_id,sample_seq,uptime_ms\n";
static void (*format_hook)(void);
static bool format_row(char *buffer, size_t size, uint32_t boot, uint32_t seq) {
    format_calls++;
    if (format_hook) { void (*hook)(void)=format_hook; format_hook=NULL; hook(); }
    snprintf(buffer,size,"1,%u,%u,%lld\n",boot,seq,(long long)clock_us/1000);
    return true;
}
IMPLEMENTATION
static void run_ticks(unsigned ticks) {
    task_ticks=0; tick_limit=ticks;
    if (!setjmp(task_exit)) log_task(NULL);
    assert(!locked);
}
static void stop_during_format(void) { assert(telemetry_log_set_recording(false)==ESP_OK); }
static void stop_start_during_format(void) { stop_during_format(); assert(telemetry_log_set_recording(true)==ESP_OK); }
static void clear_start_during_format(void) { assert(telemetry_log_clear()==ESP_OK); assert(telemetry_log_set_recording(true)==ESP_OK); }
static void simulate_reboot(void) {
    if (log_file) fclose(log_file);
    log_file=NULL; log_mutex=NULL; log_task_handle=NULL; log_status=(telemetry_log_status_t){0}; recording_generation=0;
    assert(telemetry_log_init()==ESP_OK);
}
int main(void) {
    assert(telemetry_log_set_recording(true)==ESP_ERR_INVALID_STATE);
    assert(telemetry_log_init()==ESP_OK);
    assert(!log_status.recording && log_status.sample_count==0);
    run_ticks(3);
    assert(format_calls==0 && log_status.sample_count==0);
    assert(telemetry_log_set_recording(true)==ESP_OK);
    run_ticks(2);
    assert(log_status.recording && log_status.sample_count==2);
    unsigned saved=log_status.sample_count, sync_before=sync_calls;
    assert(telemetry_log_set_recording(false)==ESP_OK);
    assert(!log_status.recording && sync_calls>sync_before);
    run_ticks(3); assert(log_status.sample_count==saved);
    uint8_t csv[512]; size_t bytes; uint32_t size;
    assert(telemetry_log_read(0,csv,sizeof(csv),&bytes,&size)==ESP_OK && bytes>strlen(CSV_HEADER));
    assert(telemetry_log_set_recording(true)==ESP_OK);
    run_ticks(1); assert(log_status.sample_count==saved+1);
    simulate_reboot(); assert(!log_status.recording && log_status.sample_count==saved+1);
    run_ticks(2); assert(log_status.sample_count==saved+1);
    assert(telemetry_log_set_recording(true)==ESP_OK);
    format_hook=stop_during_format; run_ticks(1); assert(log_status.sample_count==saved+1 && !log_status.recording);
    assert(telemetry_log_set_recording(true)==ESP_OK);
    format_hook=stop_start_during_format; run_ticks(1); assert(log_status.sample_count==saved+1);
    run_ticks(1); assert(log_status.sample_count==saved+2);
    format_hook=clear_start_during_format; run_ticks(1); assert(log_status.sample_count==0);
    run_ticks(1); assert(log_status.sample_count==1);
    assert(telemetry_log_clear()==ESP_OK); assert(!log_status.recording && log_status.sample_count==0);
    run_ticks(2); assert(log_status.sample_count==0);
    storage_free=0;
    assert(telemetry_log_set_recording(true)==ESP_ERR_INVALID_STATE && log_status.full);
    assert(telemetry_log_clear()==ESP_OK && !log_status.full && !log_status.recording);
    assert(telemetry_log_set_recording(true)==ESP_OK); run_ticks(1);
    sync_failure=true;
    assert(telemetry_log_set_recording(false)==ESP_FAIL && !log_status.recording && log_status.faulted);
    saved=log_status.sample_count; run_ticks(2); assert(log_status.sample_count==saved);
    assert(telemetry_log_set_recording(true)==ESP_ERR_INVALID_STATE);
    sync_failure=false;
    assert(telemetry_log_read(0,csv,sizeof(csv),&bytes,&size)==ESP_OK);
    assert(telemetry_log_clear()==ESP_OK && !log_status.recording && !log_status.faulted);
    fclose(log_file); log_file=NULL; unlink(LOG_FILE_PATH);
    puts("Logger lifecycle passed: idle boot/reboot, start/append/stop/save, in-flight rows, clear/recovery, full/fault guards.");
    return 0;
}
'''.replace("DEFINES", defines).replace("IMPLEMENTATION", implementation)

with tempfile.TemporaryDirectory(prefix="rc-logger-test-") as directory:
    output = Path(directory)
    (output / "esp_err.h").write_text("\n".join([
        "typedef int esp_err_t;", "#define ESP_OK 0", "#define ESP_FAIL -1",
        "#define ESP_ERR_INVALID_STATE 1", "#define ESP_ERR_INVALID_ARG 2",
        "#define ESP_ERR_NO_MEM 3", "#define ESP_ERR_NOT_SUPPORTED 4",
    ]))
    (output / "telemetry_log.h").write_text(status_header)
    (output / "test.c").write_text(harness)
    binary = output / ("test.exe" if os.name == "nt" else "test")
    compiler = [os.environ["ZIG"], "cc"] if "ZIG" in os.environ else [os.environ.get("CC", "cc")]
    subprocess.run(compiler + ["-std=c11", "-D_POSIX_C_SOURCE=200809L", str(output / "test.c"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], cwd=output, check=True)
