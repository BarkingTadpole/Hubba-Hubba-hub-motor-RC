#include "remote_command.h"

#include <string.h>

#define CHECK(condition)            \
    do {                            \
        if (!(condition)) {         \
            __builtin_trap();       \
        }                           \
    } while (0)

static remote_command_t parse_valid(const char *line)
{
    remote_command_t command;
    char error[128] = {0};
    CHECK(remote_command_parse(line, &command, error, sizeof(error)));
    CHECK(error[0] == '\0');
    return command;
}

static void expect_invalid(const char *line)
{
    remote_command_t command;
    char error[128] = {0};
    CHECK(!remote_command_parse(line, &command, error, sizeof(error)));
    CHECK(error[0] != '\0');
}

static void test_drivetrain_and_scalar_commands(void)
{
    remote_command_t command = parse_valid("disarm config");
    CHECK(command.operation == REMOTE_DISARM_FOR_CONFIG);

    command = parse_valid("config drivetrain awd");
    CHECK(command.operation == REMOTE_CONFIG_DRIVETRAIN);
    CHECK(command.drivetrain_mode == DRIVETRAIN_AWD);

    command = parse_valid("config drivetrain fwd");
    CHECK(command.drivetrain_mode == DRIVETRAIN_FWD);
    command = parse_valid("config drivetrain rwd");
    CHECK(command.drivetrain_mode == DRIVETRAIN_RWD);

    command = parse_valid("config reverse 10");
    CHECK(command.operation == REMOTE_CONFIG_REVERSE_LIMIT);
    CHECK(command.unsigned_values[0] == 10);

    command = parse_valid("config drive smoothing 0");
    CHECK(command.operation == REMOTE_CONFIG_DRIVE_SMOOTHING);
    CHECK(command.unsigned_values[0] == 0);

    command = parse_valid("config drive smoothing 100");
    CHECK(command.operation == REMOTE_CONFIG_DRIVE_SMOOTHING);
    CHECK(command.unsigned_values[0] == 100);

    command = parse_valid("config rpm poles 14");
    CHECK(command.operation == REMOTE_CONFIG_RPM_POLES);
    CHECK(command.unsigned_values[0] == 14);

    command = parse_valid("config rpm ppr 7");
    CHECK(command.operation == REMOTE_CONFIG_RPM_PPR);
    CHECK(command.unsigned_values[0] == 7);

    command = parse_valid("config steering smoothing 60");
    CHECK(command.operation == REMOTE_CONFIG_STEERING_SMOOTHING);
    CHECK(command.unsigned_values[0] == 60);

    command = parse_valid("config logging rate 5");
    CHECK(command.operation == REMOTE_CONFIG_LOGGING_RATE);
    CHECK(command.unsigned_values[0] == 5);
}

static void test_failsafe_and_boolean_commands(void)
{
    remote_command_t command = parse_valid("config failsafe 1565 10");
    CHECK(command.operation == REMOTE_CONFIG_FAILSAFE);
    CHECK(command.unsigned_values[0] == 1565);
    CHECK(command.unsigned_values[1] == 10);

    command = parse_valid("config failsafe off");
    CHECK(command.operation == REMOTE_CONFIG_FAILSAFE_ENABLED);
    CHECK(!command.enabled);

    command = parse_valid("config steering speed-limit on");
    CHECK(command.operation == REMOTE_CONFIG_STEERING_SPEED_LIMIT);
    CHECK(command.enabled);

    command = parse_valid("config steering speed-limit off");
    CHECK(!command.enabled);

    command = parse_valid("config arm-latch on");
    CHECK(command.operation == REMOTE_CONFIG_ARM_LATCH);
    CHECK(command.enabled);

    command = parse_valid("config arm-latch off");
    CHECK(command.operation == REMOTE_CONFIG_ARM_LATCH);
    CHECK(!command.enabled);

    command = parse_valid("tv enable");
    CHECK(command.operation == REMOTE_CONFIG_TV_ENABLED);
    CHECK(command.enabled);

    command = parse_valid("tv disable");
    CHECK(!command.enabled);
}

static void test_float_and_vectoring_commands(void)
{
    remote_command_t command = parse_valid("config steering trim -2.5");
    CHECK(command.operation == REMOTE_CONFIG_STEERING_TRIM);
    CHECK(command.float_values[0] < -2.49f && command.float_values[0] > -2.51f);

    command = parse_valid("config steering lateral-g 1.25");
    CHECK(command.operation == REMOTE_CONFIG_STEERING_LATERAL_G);
    CHECK(command.float_values[0] > 1.24f && command.float_values[0] < 1.26f);

    command = parse_valid("config tv authority 5");
    CHECK(command.operation == REMOTE_CONFIG_TV_AUTHORITY);
    CHECK(command.unsigned_values[0] == 5);

    command = parse_valid("config tv front-relief 20");
    CHECK(command.operation == REMOTE_CONFIG_TV_FRONT_RELIEF);
    CHECK(command.unsigned_values[0] == 20);

    command = parse_valid("config tv gains 180 0.2 0.00025 0.00004 0.2");
    CHECK(command.operation == REMOTE_CONFIG_TV_GAINS);
    CHECK(command.float_values[0] > 179.9f && command.float_values[0] < 180.1f);

    command = parse_valid("config imu yaw-sign -1");
    CHECK(command.operation == REMOTE_CONFIG_IMU_YAW_SIGN);
    CHECK(command.signed_value == -1);
}

static void test_calibration_commands(void)
{
    remote_command_t command = parse_valid("cal receiver");
    CHECK(command.operation == REMOTE_CALIBRATE_THROTTLE);
    command = parse_valid("cal steering");
    CHECK(command.operation == REMOTE_CALIBRATE_STEERING);
    command = parse_valid("cal arm");
    CHECK(command.operation == REMOTE_CALIBRATE_ARM);
    command = parse_valid("cal tv");
    CHECK(command.operation == REMOTE_CALIBRATE_TV);
    command = parse_valid("cal imu");
    CHECK(command.operation == REMOTE_CALIBRATE_IMU);
    command = parse_valid("cal capture");
    CHECK(command.operation == REMOTE_CALIBRATE_CAPTURE);
    command = parse_valid("cal esc arm");
    CHECK(command.operation == REMOTE_CALIBRATE_ESC_ARM);
    command = parse_valid("cal esc max");
    CHECK(command.operation == REMOTE_CALIBRATE_ESC_MAX);
    command = parse_valid("cal esc min");
    CHECK(command.operation == REMOTE_CALIBRATE_ESC_MIN);
    command = parse_valid("cal manual");
    CHECK(command.operation == REMOTE_CALIBRATE_ESC_MANUAL);
    command = parse_valid("cal cancel");
    CHECK(command.operation == REMOTE_CALIBRATE_CANCEL);
}

static void test_rejects_drive_and_malformed_configuration(void)
{
    expect_invalid("arm");
    expect_invalid("disarm");
    expect_invalid("disarm now");
    expect_invalid("cal");
    expect_invalid("cal esc");
    expect_invalid("cal receiver extra");
    expect_invalid("cal capture now");
    expect_invalid("cal unknown");
    expect_invalid("monitor rpm");
    expect_invalid("config reverse 101");
    expect_invalid("config drive smoothing 101");
    expect_invalid("config drive smoothing 50 extra");
    expect_invalid("config failsafe 799 10");
    expect_invalid("config rpm poles 7");
    expect_invalid("config rpm ppr 0");
    expect_invalid("config steering trim nan");
    expect_invalid("config steering smoothing 501");
    expect_invalid("config steering lateral-g 3.1");
    expect_invalid("config tv authority 26");
    expect_invalid("config tv front-relief 51");
    expect_invalid("config tv gains 501 0.2 0.00025 0.00004 0.2");
    expect_invalid("config imu yaw-sign 0");
    expect_invalid("config arm-latch maybe");
    expect_invalid("config logging rate 0");
    expect_invalid("config logging rate 51");
    expect_invalid("config reverse 10 extra");
}

int main(void)
{
    test_drivetrain_and_scalar_commands();
    test_failsafe_and_boolean_commands();
    test_float_and_vectoring_commands();
    test_calibration_commands();
    test_rejects_drive_and_malformed_configuration();
    return 0;
}
