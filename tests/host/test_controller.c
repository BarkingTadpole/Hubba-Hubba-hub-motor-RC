/* Include the complete scheduler so the harness can step its production task.
 * All extracted modules are separately compiled and linked by test_controller.py. */
#include <assert.h>
#include <setjmp.h>
#include <math.h>
#include <string.h>
#include "driver/gpio.h"
#include "powertrain_controller.c"
#include "cli.c"

static int64_t now = 1000000;
static int lock_depth;
static bool stepping, save_fails;
static jmp_buf tick_finished;
static rc_channel_sample_t inputs[RC_CHANNEL_COUNT];
static rpm_snapshot_t fake_rpm;
static uint16_t servo_pwm[STEERING_SERVO_COUNT];
static wheel_output_command_t esc_pwm;

int64_t esp_timer_get_time(void) { return now; }
const char *esp_err_to_name(esp_err_t err) { return err == ESP_OK ? "OK" : "SIMULATED_FAILURE"; }
int gpio_get_level(gpio_num_t gpio) { (void)gpio; return 0; }
size_t heap_caps_get_free_size(int caps) { (void)caps; return 100000; }
size_t heap_caps_get_minimum_free_size(int caps) { return heap_caps_get_free_size(caps); }
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void) { return &lock_depth; }
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t mutex, TickType_t wait)
{ (void)mutex; (void)wait; lock_depth++; return pdTRUE; }
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t mutex)
{ (void)mutex; assert(lock_depth > 0); lock_depth--; return pdTRUE; }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(now / 1000); }
BaseType_t xTaskDelayUntil(TickType_t *last, TickType_t ticks)
{
    (void)last;
    assert(stepping && ticks == 20 && lock_depth == 0);
    longjmp(tick_finished, 1);
}
void vTaskDelay(TickType_t ticks)
{ assert(lock_depth == 0); now += (int64_t)ticks * 1000; }
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task) { (void)task; return 4000; }
BaseType_t xTaskCreate(void (*task)(void *), const char *name, uint32_t stack,
                      void *arg, UBaseType_t priority, TaskHandle_t *handle)
{
    (void)task; (void)name; (void)stack; (void)arg; (void)priority;
    if (handle) *handle = &lock_depth;
    return pdPASS;
}
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *), const char *name, uint32_t stack,
                      void *arg, UBaseType_t priority, TaskHandle_t *handle, int core)
{ assert(core == 1); return xTaskCreate(task, name, stack, arg, priority, handle); }

esp_err_t rc_input_init(void) { return ESP_OK; }
bool rc_input_get(rc_channel_id_t id, rc_channel_sample_t *sample)
{ *sample = inputs[id]; return sample->valid; }
bool rc_input_is_valid(rc_channel_id_t id) { return inputs[id].valid; }
esp_err_t rpm_sensor_init(uint16_t ppr) { fake_rpm.pulses_per_revolution = ppr; return ESP_OK; }
void rpm_sensor_set_pulses_per_revolution(uint16_t ppr) { fake_rpm.pulses_per_revolution = ppr; }
void rpm_sensor_get_snapshot(rpm_snapshot_t *sample) { *sample = fake_rpm; }
void rpm_sensor_update(void) {}
esp_err_t servo_output_init(const uint16_t *centers)
{ memcpy(servo_pwm, centers, sizeof(servo_pwm)); return ESP_OK; }
void servo_output_set_pulses(const uint16_t *pulses) { memcpy(servo_pwm, pulses, sizeof(servo_pwm)); }
void servo_output_get_pulses(uint16_t *pulses) { memcpy(pulses, servo_pwm, sizeof(servo_pwm)); }
void esc_output_set_all(uint16_t throttle, uint16_t reverse)
{
    for (int i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        esc_pwm.throttle_us[i] = throttle < 1100 ? 1100 : throttle > 1940 ? 1940 : throttle;
        esc_pwm.reverse_us[i] = reverse;
    }
}
void esc_output_set_safe(void) { esc_output_set_all(1100, 1100); }
esp_err_t esc_output_init(const esc_limits_t *limits)
{ assert(limits->throttle_min_us == 1100); esc_output_set_safe(); return ESP_OK; }
void esc_output_set_wheels(const wheel_output_command_t *command) { esc_pwm = *command; }
void esc_output_get(wheel_output_command_t *command) { *command = esc_pwm; }

esp_err_t config_store_init(void) { return ESP_OK; }
void config_store_load_throttle(throttle_calibration_t *cal)
{ *cal = (throttle_calibration_t){.full_throttle_us=978,.neutral_us=1514,.full_reverse_us=2044,.deadband_us=80,.loaded_from_nvs=true}; }
void config_store_load_steering(steering_calibration_t *cal)
{ *cal = (steering_calibration_t){.left_us=2050,.center_us=1518,.right_us=987,.deadband_us=30,.loaded_from_nvs=true}; }
void config_store_load_arm(arm_calibration_t *cal)
{ *cal = (arm_calibration_t){.run_us=983,.stop_1_us=2049,.stop_2_us=1515,.loaded_from_nvs=true}; }
void config_store_load_drive(drive_config_t *config)
{
    *config = (drive_config_t){.reverse_limit_percent=100,.tank_limit_percent=25,.drive_smoothing_percent=100,
        .receiver_failsafe_enabled=true,.receiver_failsafe_us=1565,.receiver_failsafe_window_us=10,
        .motor_poles=14,.rpm_pulses_per_revolution=7,.permanent_arm_latch_enabled=true};
    independent_steering_defaults(&config->steering);
    config->steering.trim_us[0] = 25;
    config->steering.trim_us[1] = -35;
    config->steering.toe_tenths_deg = 30;
}
esp_err_t config_store_save_drive(const drive_config_t *config)
{ (void)config; assert(lock_depth > 0); return save_fails ? ESP_FAIL : ESP_OK; }
esp_err_t config_store_save_throttle(const throttle_calibration_t *cal)
{ (void)cal; return save_fails ? ESP_FAIL : ESP_OK; }
esp_err_t config_store_save_steering(const steering_calibration_t *cal)
{ (void)cal; return save_fails ? ESP_FAIL : ESP_OK; }
esp_err_t config_store_save_arm(const arm_calibration_t *cal)
{ (void)cal; return save_fails ? ESP_FAIL : ESP_OK; }

static void tick(uint32_t elapsed_ms)
{
    now += (int64_t)elapsed_ms * 1000;
    for (int i = 0; i < RC_CHANNEL_COUNT; i++) if (inputs[i].valid) inputs[i].updated_at_us = now;
    stepping = true;
    if (setjmp(tick_finished) == 0) powertrain_task(NULL);
    stepping = false;
    assert(lock_depth == 0);
}
static powertrain_remote_snapshot_t snapshot(void)
{
    powertrain_remote_snapshot_t result;
    assert(powertrain_get_remote_snapshot(&result));
    return result;
}
static void safe_outputs(void)
{
    for (int i = 0; i < 4; i++) assert(esc_pwm.throttle_us[i] == 1100 && esc_pwm.reverse_us[i] == 1100);
}
static void frame(uint16_t pulse)
{ inputs[RC_CHANNEL_STEERING].pulse_us = pulse; inputs[RC_CHANNEL_STEERING].valid = true; tick(20); }
static void rearm(void)
{
    if (controller.drive_config.drivetrain_mode == DRIVETRAIN_TANK) {
        frame(1518); frame(1518); frame(1518);
    }
    inputs[RC_CHANNEL_THROTTLE] = (rc_channel_sample_t){1514, now, true};
    inputs[RC_CHANNEL_ARM] = (rc_channel_sample_t){2049, now, true};
    tick(20); inputs[RC_CHANNEL_ARM].pulse_us = 983; tick(20); tick(250);
    assert(controller.system_state == SYSTEM_DRIVE_ARMED);
}

static void test_steering_and_settings(void)
{
    controller.drive_config.steering.points[0].strength_tenths_percent = 500;
    frame(987); assert(servo_pwm[0] == 1500 && servo_pwm[1] == 1500);
    controller_lock(); powertrain_steering_update(); controller_unlock();
    assert(servo_pwm[0] == 1500 && servo_pwm[1] == 1500); // duplicate timestamp
    frame(987); frame(987); assert(servo_pwm[0] > 1000 && servo_pwm[0] < 1100 && servo_pwm[1] == 1000);
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20);
    assert(servo_pwm[0] == 1500 && servo_pwm[1] == 1500);
    frame(2050); frame(2050); frame(2050); assert(servo_pwm[0] == 2000 && servo_pwm[1] > 1900 && servo_pwm[1] < 2000);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1565; tick(20);
    assert(servo_pwm[0] == 1500 && servo_pwm[1] == 1500);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1514;
    assert(powertrain_set_steering_smoothing(500));
    frame(987); frame(987); frame(987); assert(servo_pwm[1] > 1000 && servo_pwm[1] < 1500);
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20);
    assert(servo_pwm[0] == 1500 && servo_pwm[1] == 1500);
    fake_rpm = (rpm_snapshot_t){.rpm={1000,1000},.valid={true,true},.updated_at_us=now};
    tick(20); assert(snapshot().vehicle_speed_valid);
    float speed = snapshot().vehicle_speed_mps;
    assert(fabsf(speed * 3.6f - 20.169f) < .002f);
    controller.system_state = SYSTEM_DRIVE_ARMED;
    tick(110); assert(!snapshot().vehicle_speed_valid && snapshot().vehicle_speed_held);
    assert(snapshot().vehicle_speed_mps == speed);
    fake_rpm.updated_at_us = now; fake_rpm.valid[1] = false;
    tick(20); assert(snapshot().vehicle_speed_held);
    assert(powertrain_disarm_for_configuration()); tick(20);
    assert(!snapshot().vehicle_speed_held && snapshot().vehicle_speed_mps == 0);
    assert(snapshot().ackermann_strength_percent == 50);
    ackermann_point_t points[] = {{0,700},{1000,200}};
    steering_servo_config_t servo = {1000,1490,2000,300,300};
    independent_steering_config_t original = controller.drive_config.steering;
    steering_geometry_t geometry = original.geometry;
    geometry.wheel_diameter_tenths_mm = 1200;
    steering_calibration_t input_before = controller.steering_cal;
    steering_calibration_t input = input_before; input.deadband_us = 20;
    controller.system_state = SYSTEM_DRIVE_ARMED;
    assert(!powertrain_set_ackermann_curve(points,2) && !powertrain_set_steering_servo(1,&servo));
    assert(!powertrain_set_steering_geometry(&geometry) && !powertrain_set_steering_input(&input));
    controller.system_state = SYSTEM_DISARMED; controller.calibration_in_progress = true;
    assert(!powertrain_set_ackermann_curve(points,2));
    assert(!powertrain_set_steering_geometry(&geometry) && !powertrain_set_steering_input(&input));
    controller.calibration_in_progress = false; save_fails = true;
    assert(!powertrain_set_ackermann_curve(points,2) && !powertrain_set_steering_servo(1,&servo));
    assert(memcmp(&original,&controller.drive_config.steering,sizeof(original)) == 0);
    assert(!powertrain_set_steering_geometry(&geometry));
    assert(!powertrain_set_steering_input(&input));
    assert(memcmp(&input_before, &controller.steering_cal, sizeof(input)) == 0);
    assert(!powertrain_set_steering_smoothing(42));
    assert(controller.drive_config.steering_smoothing_ms == 500);
    save_fails = false;
    assert(powertrain_set_steering_geometry(&geometry));
    assert(powertrain_set_steering_input(&input));
    assert(snapshot().steering_calibration.deadband_us == 20);
    fake_rpm.valid[1] = true; fake_rpm.updated_at_us = now; tick(20);
    assert(fabsf(snapshot().vehicle_speed_mps / speed - 120.0f/107.0f) < .001f);
    geometry.max_left_tenths_deg = 310;
    assert(!powertrain_set_steering_geometry(&geometry));
    assert(controller.drive_config.steering.geometry.max_left_tenths_deg == 300);
    assert(powertrain_set_ackermann_curve(points,2) && powertrain_set_steering_servo(1,&servo));
    assert(snapshot().config.steering.servo[1].center_us == 1490);
    // Equal motor commands apply independently of steering, RPM and curve values.
    current_base_throttle_us = 1700; active_direction = DRIVE_DIRECTION_REVERSE;
    for (int mode = DRIVETRAIN_AWD; mode <= DRIVETRAIN_RWD; mode++) {
        controller.drive_config.drivetrain_mode = mode; set_drive_outputs();
        for (int i = 0; i < 4; i++) {
            bool driven = drivetrain_wheel_is_driven(mode,i);
            assert(esc_pwm.throttle_us[i] == (driven ? 1700 : 1100));
            assert(esc_pwm.reverse_us[i] == (driven ? 1940 : 1100));
        }
    }
    controller.drive_config.drivetrain_mode = 99; set_drive_outputs(); safe_outputs();
    controller.drive_config.drivetrain_mode = DRIVETRAIN_AWD;
}

static void serial_command(const char *text)
{
    char line[CLI_LINE_MAX];
    snprintf(line, sizeof(line), "%s", text);
    handle_command(line);
}

static void test_servo_trim(void)
{
    drive_config_t original = controller.drive_config;
    assert(powertrain_set_steering_trim(STEERING_LEFT, 25));
    assert(powertrain_set_steering_trim(STEERING_RIGHT, -35));
    frame(1518);
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1455);
    assert(snapshot().config.steering.trim_us[0] == 25 && snapshot().config.steering.trim_us[1] == -35);
    assert(snapshot().steering_angle_deg[0] == 0 && snapshot().steering_angle_deg[1] == 0);
    frame(987); frame(987); frame(987);
    assert(servo_pwm[0] != 1525 && servo_pwm[1] != 1455);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1565; tick(20);
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1455); safe_outputs();
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1514;
    frame(987); frame(987);
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1455); // filter warmup
    frame(987);
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20);
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1455);
    independent_steering_config_t trimmed = controller.drive_config.steering;
    assert(!powertrain_set_steering_trim(2, 0));
    assert(!powertrain_set_steering_trim(STEERING_LEFT, 451));
    assert(!powertrain_set_steering_trim(STEERING_RIGHT, -441));
    for (int state = SYSTEM_DRIVE_ARMED; state <= SYSTEM_ESC_CAL_MANUAL; state++) {
        controller.system_state = state;
        assert(!powertrain_set_steering_trim(STEERING_LEFT, 0));
    }
    controller.system_state = SYSTEM_DISARMED;
    controller.calibration_in_progress = true;
    assert(!powertrain_set_steering_trim(STEERING_RIGHT, 0));
    controller.calibration_in_progress = false;
    save_fails = true;
    assert(!powertrain_set_steering_trim(STEERING_LEFT, 0));
    assert(!powertrain_set_steering_trim(STEERING_RIGHT, 0));
    save_fails = false;
    assert(memcmp(&trimmed, &controller.drive_config.steering, sizeof(trimmed)) == 0);
    tick(20); assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1455);
    // Existing endpoint commands preserve trim, and reject conflicting travel.
    steering_servo_config_t servo = trimmed.servo[0];
    servo.left_us = 1550;
    assert(!powertrain_set_steering_servo(STEERING_LEFT, &servo));
    assert(memcmp(&trimmed, &controller.drive_config.steering, sizeof(trimmed)) == 0);
    servo.left_us = 2100;
    assert(powertrain_set_steering_servo(STEERING_LEFT, &servo));
    assert(controller.drive_config.steering.trim_us[0] == 25);
    assert(powertrain_set_steering_input(&controller.steering_cal));
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1455); // explicit recenter
    assert(powertrain_set_steering_trim(STEERING_LEFT, 0));
    assert(powertrain_set_steering_trim(STEERING_RIGHT, 0));
    tick(20); assert(servo_pwm[0] == 1500 && servo_pwm[1] == 1490);
    controller.drive_config = original;
    frame(1518);
    safe_outputs();
}

static void test_linkage_limits(void)
{
    drive_config_t original = controller.drive_config;
    independent_steering_defaults(&controller.drive_config.steering);
    controller.drive_config.steering_smoothing_ms = 0;
    steering_servo_config_t full_range = {2500,1500,500,300,300};
    assert(powertrain_set_steering_servo(0, &full_range));
    assert(powertrain_set_steering_servo(1, &full_range));
    steering_linkage_limits_t limits = {900,680};
    assert(powertrain_set_steering_linkage(&limits));
    frame(2050); frame(2050); frame(2050);
    assert(servo_pwm[0] == 2255 && servo_pwm[1] == 2255); safe_outputs();
    assert(fabsf(snapshot().steering_angle_deg[0] + 22.65f) < .001f);
    frame(987); frame(987); frame(987);
    assert(servo_pwm[0] == 745 && servo_pwm[1] == 745); safe_outputs();
    assert(fabsf(snapshot().steering_angle_deg[1] - 22.65f) < .001f);
    assert(snapshot().config.steering.linkage.outside_tenths_deg == 680);
    independent_steering_config_t before = controller.drive_config.steering;
    limits = (steering_linkage_limits_t){450,225};
    for (int state = SYSTEM_DRIVE_ARMED; state <= SYSTEM_ESC_CAL_MANUAL; state++) {
        controller.system_state = state;
        assert(!powertrain_set_steering_linkage(&limits));
    }
    controller.system_state = SYSTEM_DISARMED; controller.calibration_in_progress = true;
    assert(!powertrain_set_steering_linkage(&limits));
    controller.calibration_in_progress = false; save_fails = true;
    assert(!powertrain_set_steering_linkage(&limits));
    save_fails = false;
    assert(!powertrain_set_steering_linkage(NULL));
    assert(!powertrain_set_steering_linkage(&(steering_linkage_limits_t){901,680}));
    assert(!powertrain_set_steering_linkage(&(steering_linkage_limits_t){900,901}));
    assert(memcmp(&before, &controller.drive_config.steering, sizeof(before)) == 0);
    tick(20); assert(servo_pwm[0] == 745 && servo_pwm[1] == 745);
    assert(powertrain_set_steering_linkage(&limits));
    tick(20); assert(servo_pwm[0] == 1250 && servo_pwm[1] == 1250);
    assert(snapshot().config.steering.linkage.outside_tenths_deg == 225);
    assert(powertrain_set_steering_trim(0, 25));
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20);
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1500);
    frame(2050); frame(2050); frame(2050);
    assert(servo_pwm[0] <= 2025 && servo_pwm[1] <= 1750);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1565; tick(20);
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1500); safe_outputs();
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1514;
    limits.inside_tenths_deg = 0;
    assert(powertrain_set_steering_linkage(&limits));
    frame(987); frame(987); frame(987);
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1500);
    controller.drive_config = original;
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20);
}

static void test_toe_settings(void)
{
    drive_config_t original = controller.drive_config;
    independent_steering_defaults(&controller.drive_config.steering);
    controller.drive_config.steering_smoothing_ms = 0;
    for (int toe = -300; toe <= 300; toe += 600) {
        assert(powertrain_set_steering_toe(toe));
        inputs[RC_CHANNEL_STEERING].valid = false; tick(20);
        assert(snapshot().config.steering.toe_tenths_deg == toe);
        assert(snapshot().steering_angle_deg[0] == toe / 10.0f);
        assert(snapshot().steering_angle_deg[1] == -toe / 10.0f);
        assert(servo_pwm[0] == (toe < 0 ? 2000 : 1000));
        assert(servo_pwm[1] == (toe < 0 ? 1000 : 2000));
    }
    assert(powertrain_set_steering_toe(30));
    frame(1518); frame(1518); frame(1518);
    assert(servo_pwm[0] == 1450 && servo_pwm[1] == 1550);
    assert(snapshot().steering_angle_deg[0] == 3 && snapshot().steering_angle_deg[1] == -3);
    frame(987); frame(987); frame(987);
    assert(servo_pwm[0] == 1000 && servo_pwm[1] == 1100);
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20);
    assert(servo_pwm[0] == 1450 && servo_pwm[1] == 1550);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1565;
    frame(2050); frame(2050); frame(2050);
    assert(servo_pwm[0] == 1450 && servo_pwm[1] == 1550);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1514;
    independent_steering_config_t before = controller.drive_config.steering;
    for (int state = SYSTEM_DRIVE_ARMED; state <= SYSTEM_ESC_CAL_MANUAL; state++) {
        controller.system_state = state;
        assert(!powertrain_set_steering_toe(-15));
    }
    controller.system_state = SYSTEM_DISARMED; controller.calibration_in_progress = true;
    assert(!powertrain_set_steering_toe(-15));
    controller.calibration_in_progress = false; save_fails = true;
    assert(!powertrain_set_steering_toe(-15)); save_fails = false;
    assert(!powertrain_set_steering_toe(301) && !powertrain_set_steering_toe(-301));
    assert(!powertrain_set_steering_linkage(&(steering_linkage_limits_t){900,0}));
    assert(!powertrain_set_steering_linkage(&(steering_linkage_limits_t){900,1}));
    assert(memcmp(&before, &controller.drive_config.steering, sizeof(before)) == 0);
    assert(powertrain_set_steering_linkage(&(steering_linkage_limits_t){0,900}));
    assert(!powertrain_set_steering_toe(-1));
    assert(powertrain_set_steering_toe(300));
    assert(powertrain_set_steering_toe(30));
    assert(powertrain_set_steering_linkage(&(steering_linkage_limits_t){900,50}));
    before = controller.drive_config.steering;
    assert(!powertrain_set_steering_trim(STEERING_LEFT, 100)); // Valid neutral, but toe would exceed the 55 us outside cap.
    steering_servo_config_t wider = before.servo[0]; wider.right_us = 500;
    assert(!powertrain_set_steering_servo(STEERING_LEFT, &wider));
    assert(memcmp(&before, &controller.drive_config.steering, sizeof(before)) == 0);
    assert(powertrain_set_steering_toe(-15));
    powertrain_steering_recenter();
    assert(servo_pwm[0] == 1525 && servo_pwm[1] == 1475);
    controller.drive_config = original;
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20); safe_outputs();
}

static void test_shared_commands(void)
{
    drive_config_t original = controller.drive_config;
    steering_calibration_t original_input = controller.steering_cal;
    const char *commands[] = {
        "config drivetrain rwd", "config reverse 20", "config drive smoothing 50",
        "config drivetrain tank", "config tank limit 37", "config steering toe -1.5",
        "config failsafe off", "config failsafe 1570 12", "config rpm poles 16",
        "config rpm ppr 9", "config steering smoothing 123", "config arm-latch on",
        "config steering geometry 450 310 110 25 28",
        "config steering input 2050 1518 987 20",
        "config steering servo right 1000 1510 2100 35 40",
        "config steering servo left 2500 1500 500 30 30",
        "config steering servo right 2500 1500 500 30 30",
        "config steering trim left 25", "config steering trim right -35",
        "config steering linkage 88.5 67.5",
        "config steering ackermann-strength 0:60 30:20 100:10",
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        controller.drive_config = original;
        controller.steering_cal = original_input;
        serial_command(commands[i]);
        drive_config_t serial_config = snapshot().config;
        steering_calibration_t serial_input = controller.steering_cal;
        controller.steering_cal = original_input;
        controller.drive_config = original;
        remote_command_t parsed;
        char error[192];
        assert(remote_command_parse(commands[i], &parsed, error, sizeof(error)));
        assert(powertrain_apply_command(&parsed));
        assert(memcmp(&serial_config, &controller.drive_config, sizeof(serial_config)) == 0);
        assert(serial_input.left_us == controller.steering_cal.left_us &&
               serial_input.center_us == controller.steering_cal.center_us &&
               serial_input.right_us == controller.steering_cal.right_us &&
               serial_input.deadband_us == controller.steering_cal.deadband_us &&
               serial_input.loaded_from_nvs == controller.steering_cal.loaded_from_nvs);
    }
    controller.drive_config = original;
    controller.steering_cal = original_input;
    // Reject overflow before narrowing to a PWM value in both command paths.
    serial_command("config failsafe 67050 10");
    serial_command("config reverse -1");
    serial_command("config tv authority 10");
    assert(memcmp(&original, &controller.drive_config, sizeof(original)) == 0);
    serial_command("  CONFIG DRIVETRAIN FWD  \r\n");
    assert(controller.drive_config.drivetrain_mode == DRIVETRAIN_FWD);
    controller.system_state = SYSTEM_DRIVE_ARMED;
    serial_command("config drivetrain rwd");
    assert(controller.drive_config.drivetrain_mode == DRIVETRAIN_FWD);
    serial_command("disarm config");
    assert(controller.system_state == SYSTEM_DISARMED);
    serial_command("cal esc arm");
    assert(controller.system_state == SYSTEM_ESC_CAL_ARMED);
    serial_command("cal cancel");
    assert(controller.system_state == SYSTEM_DISARMED);
    controller.drive_config = original;
    rpm_sensor_set_pulses_per_revolution(original.rpm_pulses_per_revolution);
}

static void test_arming_and_direction(void)
{
    reset_drive_state(); controller.arm_cycle_ready = false;
    inputs[RC_CHANNEL_ARM].pulse_us = 983;
    tick(300); tick(300); assert(controller.system_state == SYSTEM_DISARMED);
    inputs[RC_CHANNEL_ARM].pulse_us = 2049; tick(20);
    inputs[RC_CHANNEL_ARM].pulse_us = 983; tick(20); tick(249);
    assert(controller.system_state == SYSTEM_DISARMED);
    tick(1); assert(controller.system_state == SYSTEM_DRIVE_ARMED); safe_outputs();
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978; tick(20); assert(esc_pwm.throttle_us[0] == 1112);
    inputs[RC_CHANNEL_ARM].pulse_us = 1515; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DRIVE_ARMED);
    inputs[RC_CHANNEL_ARM].pulse_us = 983; tick(20); assert(esc_pwm.throttle_us[0] == 1112);
    inputs[RC_CHANNEL_ARM].pulse_us = 1750; tick(20); tick(59);
    assert(!(controller.drive_inhibit_flags & DRIVE_INHIBIT_CH4));
    tick(1); safe_outputs(); assert(controller.drive_inhibit_flags & DRIVE_INHIBIT_CH4);
    inputs[RC_CHANNEL_ARM].pulse_us = 983;
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1565; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DRIVE_ARMED);
    assert(powertrain_disarm_for_configuration());
    assert(powertrain_set_permanent_arm_latch_enabled(false)); rearm();
    inputs[RC_CHANNEL_THROTTLE].valid = false; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DISARMED && !controller.arm_cycle_ready);
    rearm(); inputs[RC_CHANNEL_ARM].pulse_us = 2049; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DISARMED);
    assert(powertrain_set_drive_smoothing(0)); rearm();
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978; tick(20); assert(esc_pwm.throttle_us[0] == 1940);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 2044; tick(20); safe_outputs();
    tick(119); safe_outputs(); assert(active_direction == DRIVE_DIRECTION_FORWARD);
    tick(1); safe_outputs(); assert(active_direction == DRIVE_DIRECTION_REVERSE);
    tick(20); assert(esc_pwm.throttle_us[0] == 1940 && esc_pwm.reverse_us[0] == 1940);
    assert(powertrain_disarm_for_configuration()); safe_outputs();
}

static void tank_outputs(uint16_t throttle, bool left)
{
    for (int i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        bool left_side = i == WHEEL_FRONT_LEFT || i == WHEEL_REAR_LEFT;
        assert(esc_pwm.throttle_us[i] == throttle);
        assert(esc_pwm.reverse_us[i] == (left == left_side ? 1940 : 1100));
    }
}

static void test_tank_mode(void)
{
    drive_config_t original = controller.drive_config;
    steering_calibration_t original_input = controller.steering_cal;
    independent_steering_defaults(&controller.drive_config.steering);
    assert(powertrain_set_steering_toe(20));
    assert(powertrain_set_drivetrain_mode(DRIVETRAIN_TANK));
    assert(powertrain_set_tank_limit(25));
    assert(powertrain_set_reverse_limit(0)); // Tank limit is independent, including the reversing side.
    assert(powertrain_set_drive_smoothing(100));
    assert(powertrain_set_steering_smoothing(500)); // Cannot delay tank direction or neutral.
    assert(powertrain_set_permanent_arm_latch_enabled(true));
    inputs[RC_CHANNEL_THROTTLE] = (rc_channel_sample_t){1514,now,true};
    inputs[RC_CHANNEL_ARM] = (rc_channel_sample_t){2049,now,true};
    inputs[RC_CHANNEL_STEERING].valid = false;
    tick(20); inputs[RC_CHANNEL_ARM].pulse_us = 983; tick(20); tick(300);
    assert(controller.system_state == SYSTEM_DISARMED); safe_outputs();
    powertrain_arm_from_cli(); assert(controller.system_state == SYSTEM_DISARMED);
    frame(2050); frame(2050); frame(2050); tick(300);
    assert(controller.system_state == SYSTEM_DISARMED); safe_outputs();
    powertrain_arm_from_cli(); assert(controller.system_state == SYSTEM_DISARMED);
    frame(1518); frame(1518); frame(1518); rearm(); safe_outputs();
    assert(servo_pwm[0] == 1467 && servo_pwm[1] == 1533);
    assert(snapshot().ackermann_strength_percent == 0);
    assert(!powertrain_set_drivetrain_mode(DRIVETRAIN_AWD) && !powertrain_set_tank_limit(100));
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978; tick(20); safe_outputs(); // CH1 centered.
    frame(2050); frame(2050);
    assert(direction_hold_until_us == now + 120000); safe_outputs();
    tick(119); safe_outputs(); assert(active_direction == DRIVE_DIRECTION_FORWARD);
    tick(1); tank_outputs(1100, true); // Reverse lines published only at minimum throttle.
    tick(20); tank_outputs(1112, true);
    for (int i = 0; i < 20; i++) tick(20);
    tank_outputs(1310, true); // 1100 + 25% of 840.
    assert(servo_pwm[0] == 1467 && servo_pwm[1] == 1533);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 2044; tick(20); tank_outputs(1310, true);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1779; tick(20); tick(20); tank_outputs(1153, true);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1246; tick(20); tank_outputs(1153, true);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978;
    for (int i = 0; i < 20; i++) tick(20);
    tank_outputs(1310, true);
    frame(1518); tank_outputs(1210, true); // Raw center stops an old median command immediately.
    tick(20); tick(20); tank_outputs(1100, true);
    frame(987); frame(987); tick(120); tank_outputs(1100, false);
    for (int i = 0; i < 20; i++) tick(20);
    tank_outputs(1310, false);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1514;
    tick(20); tick(20); tick(20); tank_outputs(1100, false);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978;
    for (int i = 0; i < 20; i++) tick(20);
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DRIVE_ARMED && (controller.drive_inhibit_flags & DRIVE_INHIBIT_STEERING_LOST));
    assert(strstr(drive_inhibit_reason(controller.drive_inhibit_flags), "CH1") != NULL);
    frame(987); frame(987); safe_outputs(); // Warmup cannot power motors.
    frame(987); safe_outputs(); assert(controller.drive_inhibit_flags == 0);
    tick(120); tank_outputs(1100, false); tick(20); tank_outputs(1112, false);
    for (int fault = 0; fault < 3; fault++) {
        if (fault == 0) inputs[RC_CHANNEL_ARM].pulse_us = 2049;
        if (fault == 1) inputs[RC_CHANNEL_THROTTLE].valid = false;
        if (fault == 2) inputs[RC_CHANNEL_THROTTLE].pulse_us = 1565;
        tick(20); safe_outputs(); assert(controller.system_state == SYSTEM_DRIVE_ARMED);
        inputs[RC_CHANNEL_ARM].pulse_us = 983;
        inputs[RC_CHANNEL_THROTTLE] = (rc_channel_sample_t){1514,now,true};
        frame(1518); frame(1518); frame(1518); safe_outputs();
    }
    powertrain_disarm(); safe_outputs(); tick(20);
    assert(controller.drive_inhibit_flags & DRIVE_INHIBIT_CLI);
    assert(powertrain_disarm_for_configuration());
    drive_config_t before = controller.drive_config;
    save_fails = true;
    assert(!powertrain_set_tank_limit(70) && !powertrain_set_drivetrain_mode(DRIVETRAIN_AWD));
    save_fails = false;
    assert(!powertrain_set_tank_limit(101));
    assert(memcmp(&before, &controller.drive_config, sizeof(before)) == 0);
    controller.calibration_in_progress = true;
    assert(!powertrain_set_tank_limit(70) && !powertrain_set_drivetrain_mode(DRIVETRAIN_AWD));
    controller.calibration_in_progress = false;
    // Full-power reversal: first finish decelerating, then wait the entire 120 ms.
    assert(powertrain_set_tank_limit(100)); rearm();
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978; frame(2050); frame(2050); tick(120);
    for (int i = 0; i < 75; i++) tick(20);
    tank_outputs(1940, true);
    frame(987);
    for (int i = 0; i < 7; i++) { tick(20); assert(active_direction == DRIVE_DIRECTION_TANK_LEFT); }
    tank_outputs(1140, true); assert(direction_hold_until_us == 0);
    tick(20); tank_outputs(1100, true); assert(direction_hold_until_us == now + 120000);
    tick(119); tank_outputs(1100, true);
    tick(1); tank_outputs(1100, false);
    tick(20); tank_outputs(1112, false);
    assert(powertrain_disarm_for_configuration());
    assert(powertrain_set_drive_smoothing(0)); rearm();
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978; frame(2050); frame(2050);
    tick(119); safe_outputs(); tick(1); tank_outputs(1100, true);
    tick(20); tank_outputs(1940, true);
    frame(987); frame(987); tank_outputs(1100, true);
    tick(119); tank_outputs(1100, true); tick(1); tank_outputs(1100, false);
    tick(20); tank_outputs(1940, false);
    assert(powertrain_disarm_for_configuration());
    assert(powertrain_set_tank_limit(0)); rearm();
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978; frame(2050); frame(2050); tick(300); safe_outputs();
    // Non-latching CH1 faults fully disarm and demand a new physical cycle.
    assert(powertrain_disarm_for_configuration());
    assert(powertrain_set_tank_limit(25)); assert(powertrain_set_permanent_arm_latch_enabled(false));
    frame(1518); frame(1518); rearm();
    inputs[RC_CHANNEL_STEERING].valid = false; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DISARMED && !controller.arm_cycle_ready);
    frame(1518); frame(1518); frame(1518); tick(300);
    assert(controller.system_state == SYSTEM_DISARMED);
    rearm(); inputs[RC_CHANNEL_ARM].pulse_us = 2049; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DISARMED);
    // Calibration retains its existing all-motor, forward-only behavior in tank mode.
    assert(powertrain_cal_esc_arm()); assert(powertrain_cal_esc_max());
    for (int i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) assert(esc_pwm.throttle_us[i] == 1940 && esc_pwm.reverse_us[i] == 1100);
    assert(!powertrain_set_tank_limit(70) && !powertrain_set_drivetrain_mode(DRIVETRAIN_AWD));
    assert(powertrain_cal_cancel()); safe_outputs();
    // Rotation follows calibrated CH1 left/right, not a hard-coded PWM sign.
    steering_calibration_t inverted_input = original_input;
    inverted_input.left_us = 987; inverted_input.right_us = 2050;
    assert(powertrain_set_steering_input(&inverted_input));
    rearm(); inputs[RC_CHANNEL_THROTTLE].pulse_us = 978;
    frame(987); frame(987); tick(120); tank_outputs(1100, true);
    tick(20); tank_outputs(1310, true);
    assert(powertrain_disarm_for_configuration());
    assert(powertrain_set_steering_input(&original_input));
    controller.drive_config = original;
    powertrain_steering_recenter();
}

static void test_calibration_workflows(void)
{
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1514; inputs[RC_CHANNEL_ARM].pulse_us = 2049;
    assert(powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_THROTTLE));
    assert(snapshot().calibration_active);
    assert(!powertrain_set_reverse_limit(20));
    assert(powertrain_remote_calibration_capture());
    assert(snapshot().calibration_step == 1 && snapshot().calibration_values_us[0] == 1514);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 978; assert(powertrain_remote_calibration_capture());
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 2044; assert(powertrain_remote_calibration_capture());
    assert(!snapshot().calibration_active && controller.throttle_cal.full_reverse_us == 2044);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1514;
    assert(powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_STEERING));
    inputs[RC_CHANNEL_STEERING] = (rc_channel_sample_t){1518,now,true};
    assert(powertrain_remote_calibration_capture());
    inputs[RC_CHANNEL_STEERING].pulse_us = 2050; assert(powertrain_remote_calibration_capture());
    inputs[RC_CHANNEL_STEERING].pulse_us = 987; assert(powertrain_remote_calibration_capture());
    assert(servo_pwm[0] == 1500 && servo_pwm[1] == 1490);
    assert(controller.steering_cal.deadband_us == 20); // Guided capture keeps tuning.
    assert(snapshot().steering_angle_deg[0] == 0 && snapshot().steering_angle_deg[1] == 0);
    assert(powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_ARM));
    inputs[RC_CHANNEL_ARM].pulse_us = 983; assert(powertrain_remote_calibration_capture());
    inputs[RC_CHANNEL_ARM].pulse_us = 2049; assert(powertrain_remote_calibration_capture());
    inputs[RC_CHANNEL_ARM].pulse_us = 1515; assert(powertrain_remote_calibration_capture());
    assert(!snapshot().calibration_active && !controller.arm_cycle_ready);
    assert(powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_STEERING));
    tick(120001); assert(!snapshot().calibration_active); safe_outputs();
    assert(powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_STEERING));
    assert(powertrain_cal_cancel()); assert(!snapshot().calibration_active); safe_outputs();
    inputs[RC_CHANNEL_ARM].pulse_us = 983;
    assert(!powertrain_cal_esc_arm()); // RUN cannot enter ESC endpoint calibration
    inputs[RC_CHANNEL_ARM].pulse_us = 2049;
    assert(powertrain_cal_esc_arm()); assert(snapshot().calibration_kind == POWERTRAIN_CALIBRATION_ESC);
    assert(powertrain_cal_esc_max()); assert(esc_pwm.throttle_us[0] == 1940 && esc_pwm.reverse_us[0] == 1100);
    assert(powertrain_cal_esc_min()); safe_outputs();
    tick(30001); assert(controller.system_state == SYSTEM_DISARMED); safe_outputs();
    assert(powertrain_cal_esc_arm()); assert(powertrain_cal_esc_max());
    inputs[RC_CHANNEL_ARM].valid = false; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DISARMED);
    inputs[RC_CHANNEL_ARM].valid = true;
    assert(powertrain_cal_manual()); inputs[RC_CHANNEL_THROTTLE].pulse_us = 1800; tick(20);
    assert(esc_pwm.throttle_us[0] == 1800 && esc_pwm.reverse_us[0] == 1100);
    inputs[RC_CHANNEL_THROTTLE].pulse_us = 1565; tick(20); safe_outputs();
    assert(controller.system_state == SYSTEM_DISARMED);
    powertrain_print_status(); powertrain_print_help();
}

static void test_wheel_calibration(void)
{
    drive_config_t original = controller.drive_config;
    independent_steering_defaults(&controller.drive_config.steering);
    controller.drive_config.drivetrain_mode = DRIVETRAIN_TANK; // Measurement still follows physical CH1.
    controller.drive_config.steering_smoothing_ms = 0;
    inputs[RC_CHANNEL_THROTTLE] = (rc_channel_sample_t){1514,now,true};
    inputs[RC_CHANNEL_ARM] = (rc_channel_sample_t){2049,now,true};
    frame(1518); frame(1518); frame(1518);
    controller.arm_cal.loaded_from_nvs = false;
    assert(!powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_WHEEL_LEFT));
    controller.arm_cal.loaded_from_nvs = true;
    controller.drive_config.steering.toe_tenths_deg = 10;
    assert(!powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_WHEEL_LEFT));
    controller.drive_config.steering.toe_tenths_deg = 0;
    frame(2050); assert(!powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_WHEEL_LEFT));
    frame(1518); frame(1518); frame(1518);
    wheel_angle_calibration_t points = {5, {{2000,-250},{1750,-100},{1500,0},{1200,80},{1000,280}}};
    for (int side = 0; side < 2; side++) {
        powertrain_calibration_kind_t kind = side ? POWERTRAIN_CALIBRATION_WHEEL_RIGHT : POWERTRAIN_CALIBRATION_WHEEL_LEFT;
        assert(powertrain_remote_calibration_start(kind));
        assert(snapshot().calibration_active && snapshot().calibration_kind == kind);
        assert(!powertrain_set_wheel_calibration(side, &points, 250, 280));
        assert(!powertrain_set_steering_trim(side, 10));
        frame(2050); frame(2050); frame(2050);
        assert(servo_pwm[side] == 2000 && servo_pwm[1-side] == 1500); safe_outputs();
        uint32_t sequence = snapshot().wheel_capture_sequence;
        assert(powertrain_remote_calibration_capture());
        assert(snapshot().calibration_values_us[0] == 2000 && snapshot().wheel_capture_sequence == sequence + 1);
        frame(987); frame(987); frame(987);
        assert(servo_pwm[side] == 1000 && servo_pwm[1-side] == 1500); safe_outputs();
        frame(1518); frame(1518); frame(1518);
        assert(powertrain_cal_cancel());
        assert(!snapshot().calibration_active && !controller.arm_cycle_ready); safe_outputs();
    }
    // Every loss/exit guard terminates the measurement and keeps motor outputs safe.
    for (int fault = 0; fault < 7; fault++) {
        inputs[RC_CHANNEL_THROTTLE] = (rc_channel_sample_t){1514,now,true};
        inputs[RC_CHANNEL_ARM] = (rc_channel_sample_t){2049,now,true};
        frame(1518); frame(1518); frame(1518);
        assert(powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_WHEEL_LEFT));
        if (fault == 0) inputs[RC_CHANNEL_ARM].pulse_us = 983;
        if (fault == 1) inputs[RC_CHANNEL_ARM].valid = false;
        if (fault == 2) inputs[RC_CHANNEL_THROTTLE].pulse_us = 978;
        if (fault == 3) inputs[RC_CHANNEL_THROTTLE].valid = false;
        if (fault == 4) inputs[RC_CHANNEL_STEERING].valid = false;
        if (fault == 5) inputs[RC_CHANNEL_THROTTLE].pulse_us = 1565;
        tick(fault == 6 ? 120001 : 20);
        assert(!snapshot().calibration_active && controller.system_state == SYSTEM_DISARMED);
        assert(!controller.arm_cycle_ready || inputs[RC_CHANNEL_ARM].pulse_us == 2049);
        safe_outputs();
    }
    inputs[RC_CHANNEL_THROTTLE] = (rc_channel_sample_t){1514,now,true};
    inputs[RC_CHANNEL_ARM] = (rc_channel_sample_t){2049,now,true};
    frame(1518); frame(1518); frame(1518);
    independent_steering_config_t before = controller.drive_config.steering;
    assert(!powertrain_set_wheel_calibration(0, &points, 300, 300)); // Measured range is smaller.
    save_fails = true; assert(!powertrain_set_wheel_calibration(0, &points, 250, 280)); save_fails = false;
    assert(memcmp(&before, &controller.drive_config.steering, sizeof(before)) == 0);
    for (int state = SYSTEM_DRIVE_ARMED; state <= SYSTEM_ESC_CAL_MANUAL; state++) {
        controller.system_state = state;
        assert(!powertrain_set_wheel_calibration(0, &points, 250, 280));
    }
    controller.system_state = SYSTEM_DISARMED;
    assert(powertrain_set_wheel_calibration(0, &points, 250, 280));
    assert(snapshot().config.steering.wheel_calibration[0].count == 5);
    assert(snapshot().config.steering.geometry.max_left_tenths_deg == 250);
    assert(!powertrain_set_steering_trim(0, 1));
    steering_servo_config_t changed = controller.drive_config.steering.servo[0]; changed.left_us = 2100;
    assert(!powertrain_set_steering_servo(0, &changed));
    wheel_angle_calibration_t clear = {0};
    assert(powertrain_set_wheel_calibration(0, &clear, 0, 0));
    assert(powertrain_set_steering_trim(0, 1));
    char command[] = "config steering wheel right 25 28 2000:-25 1750:-10 1500:0 1200:8 1000:28";
    handle_command(command);
    assert(snapshot().config.steering.wheel_calibration[1].count == 5);
    controller.drive_config = original;
    powertrain_steering_recenter();
}

int main(void)
{
    inputs[RC_CHANNEL_THROTTLE] = (rc_channel_sample_t){1514,now,true};
    inputs[RC_CHANNEL_ARM] = (rc_channel_sample_t){2049,now,true};
    assert(powertrain_controller_init() == ESP_OK); safe_outputs();
    assert(servo_pwm[0] == 1473 && servo_pwm[1] == 1519); // saved trims plus toe at PWM startup
    assert(powertrain_set_steering_toe(0));
    assert(powertrain_set_steering_trim(STEERING_LEFT, 0));
    assert(powertrain_set_steering_trim(STEERING_RIGHT, 0));
    assert(powertrain_controller_start() == ESP_OK);
    assert(cli_start() == ESP_OK);
    test_steering_and_settings();
    test_servo_trim();
    test_linkage_limits();
    test_toe_settings();
    test_shared_commands();
    test_arming_and_direction();
    test_tank_mode();
    test_calibration_workflows();
    test_wheel_calibration();
    assert(lock_depth == 0);
    puts("Controller modules passed: steering/toe/speed, config rollback, normal/tank outputs and limits, arming/failsafes, direction holds, calibration/cancel/timeouts and snapshots.");
}
