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
    remote_command_t limit = parse_valid("config steering limit 0:100 40:60.5 80:0");
    CHECK(limit.operation == REMOTE_CONFIG_STEERING_LIMIT && limit.steering_limit.point_count == 3);
    CHECK(limit.steering_limit.points[1].limit_tenths_percent == 605);
    const char *bad_limit[] = {"", "0:100", "1:100 40:60", "0:100 0:60", "0:100 301:60",
        "0:100 40:101", "0:100 40:-1", "0:nan 40:60", "0:100 inf:60", "0:100 40:60 extra",
        "0:100 40:60.01", "0:100 40.01:60", "0:100 1:90 2:80 3:70 4:60 5:50 6:40 7:30 8:20"};
    for (unsigned i = 0; i < sizeof(bad_limit) / sizeof(bad_limit[0]); i++) {
        char line[240];
        strcpy(line, "config steering limit "); strcat(line, bad_limit[i]);
        expect_invalid(line);
    }
    remote_command_t wheel = parse_valid("config steering wheel left 25 28 2000:-25 1750:-10.5 1500:0 1000:28");
    CHECK(wheel.operation == REMOTE_CONFIG_WHEEL_CALIBRATION && wheel.unsigned_values[0] == STEERING_LEFT);
    CHECK(wheel.wheel_calibration.count == 4 && wheel.wheel_calibration.points[1].angle_tenths_deg == -105);
    CHECK(wheel.geometry.max_left_tenths_deg == 250 && wheel.geometry.max_right_tenths_deg == 280);
    wheel = parse_valid("config steering wheel right clear");
    CHECK(wheel.wheel_calibration.count == 0 && wheel.unsigned_values[0] == STEERING_RIGHT);
    CHECK(parse_valid("cal wheel left").operation == REMOTE_CALIBRATE_WHEEL_LEFT);
    CHECK(parse_valid("cal wheel right").operation == REMOTE_CALIBRATE_WHEEL_RIGHT);
    const char *bad_wheel[] = {"cal wheel", "cal wheel rear", "cal wheel left 1500", "config steering wheel left clear extra",
        "config steering wheel left 30 30 2000:-30 1500:0", "config steering wheel left 30 30 2000:-30 1500:0 499:30",
        "config steering wheel left 30 30 2000:-30 1500:0 1000:85.1", "config steering wheel left nan 30 2000:-30 1500:0 1000:30",
        "config steering wheel left 30 30 2000:-30 1500:0 1000:nan", "config steering wheel left 30 30 2000:-30 1500:0 1000:1.01",
        "config steering wheel left 30 30 2000:-30 1500:0 1000.5:30", "config steering wheel left 30 30 2000:-30 1500:0 1000:30 extra",
        "config steering wheel left 30 30 2000:-30 1900:-25 1800:-20 1700:-15 1600:-10 1500:0 1400:10 1300:15 1200:20 1000:30"};
    for (unsigned i = 0; i < sizeof(bad_wheel) / sizeof(bad_wheel[0]); i++) expect_invalid(bad_wheel[i]);
    remote_command_t command = parse_valid("disarm config");
    CHECK(command.operation == REMOTE_DISARM_FOR_CONFIG);

    command = parse_valid("config drivetrain awd");
    CHECK(command.operation == REMOTE_CONFIG_DRIVETRAIN);
    CHECK(command.drivetrain_mode == DRIVETRAIN_AWD);

    command = parse_valid("config drivetrain fwd");
    CHECK(command.drivetrain_mode == DRIVETRAIN_FWD);
    command = parse_valid("config drivetrain rwd");
    CHECK(command.drivetrain_mode == DRIVETRAIN_RWD);
    command = parse_valid("config drivetrain tank");
    CHECK(command.drivetrain_mode == DRIVETRAIN_TANK);
    command = parse_valid("config tank limit 25");
    CHECK(command.operation == REMOTE_CONFIG_TANK_LIMIT && command.unsigned_values[0] == 25);
    command = parse_valid("config tank limit 0");
    CHECK(command.unsigned_values[0] == 0);
    command = parse_valid("config tank limit 100");
    CHECK(command.unsigned_values[0] == 100);
    command = parse_valid("config steering toe +1.5");
    CHECK(command.operation == REMOTE_CONFIG_STEERING_TOE && command.toe_tenths_deg == 15);
    command = parse_valid("config steering toe -30");
    CHECK(command.toe_tenths_deg == -300);
    command = parse_valid("config steering toe 22.6");
    CHECK(command.toe_tenths_deg == 226);
    command = parse_valid("config steering toe 85");
    CHECK(command.toe_tenths_deg == 850);
    // Parsing only enforces representation; the shared settings guard checks actual travel.
    command = parse_valid("config steering toe 3276.7");
    CHECK(command.toe_tenths_deg == INT16_MAX);
    command = parse_valid("config steering toe -3276.8");
    CHECK(command.toe_tenths_deg == INT16_MIN);
    command = parse_valid("config steering toe 0");
    CHECK(command.toe_tenths_deg == 0);
    const char *invalid[] = {"config tank limit -1", "config tank limit 101", "config tank limit 12.5",
        "config tank limit 4294967296", "config tank limit 25 extra", "config steering toe 3276.8",
        "config steering toe -3276.9", "config steering toe nan", "config steering toe inf",
        "config steering toe 0.01", "config steering toe 1 extra", "config steering toe ", "tank left"};
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); i++) expect_invalid(invalid[i]);

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

    command = parse_valid("config arm-latch on");
    CHECK(command.operation == REMOTE_CONFIG_ARM_LATCH);
    CHECK(command.enabled);

    command = parse_valid("config arm-latch off");
    CHECK(command.operation == REMOTE_CONFIG_ARM_LATCH);
    CHECK(!command.enabled);


}

static void test_independent_steering_commands(void)
{
    remote_command_t c = parse_valid("config steering servo right 1000 1490 2000 30 30");
    CHECK(c.operation == REMOTE_CONFIG_STEERING_SERVO);
    CHECK(c.unsigned_values[0] == STEERING_RIGHT && c.servo.center_us == 1490);
    c = parse_valid("config steering trim left +25");
    CHECK(c.operation == REMOTE_CONFIG_STEERING_TRIM && c.unsigned_values[0] == STEERING_LEFT && c.steering_trim_us == 25);
    c = parse_valid("config steering trim right -35");
    CHECK(c.operation == REMOTE_CONFIG_STEERING_TRIM && c.unsigned_values[0] == STEERING_RIGHT && c.steering_trim_us == -35);
    c = parse_valid("config steering trim right 0");
    CHECK(c.steering_trim_us == 0);
    c = parse_valid("config steering linkage 90 68");
    CHECK(c.operation == REMOTE_CONFIG_STEERING_LINKAGE && c.linkage.inside_tenths_deg == 900 && c.linkage.outside_tenths_deg == 680);
    c = parse_valid("config steering linkage 45.5 0");
    CHECK(c.linkage.inside_tenths_deg == 455 && c.linkage.outside_tenths_deg == 0);
    c = parse_valid("config steering linkage 0 90");
    CHECK(c.linkage.inside_tenths_deg == 0 && c.linkage.outside_tenths_deg == 900);
    c = parse_valid("config steering trim left 1900"); CHECK(c.steering_trim_us == 1900);
    c = parse_valid("config steering trim right -1900"); CHECK(c.steering_trim_us == -1900);
    c = parse_valid("config steering servo left 2500 1500 500 30 30");
    CHECK(c.servo.left_us == 2500 && c.servo.center_us == 1500 && c.servo.right_us == 500);
    c = parse_valid("config steering servo right 500 550 2500 30 30");
    CHECK(c.servo.left_us == 500 && c.servo.center_us == 550 && c.servo.right_us == 2500);
    c = parse_valid("config steering ackermann-strength 0:60 10.5:40.2 100:20");
    CHECK(c.operation == REMOTE_CONFIG_ACKERMANN && c.point_count == 3);
    CHECK(c.points[1].speed_tenths_kmh == 105 && c.points[1].strength_tenths_percent == 402);
    c = parse_valid("config steering geometry 445.5 320 107 29.9 30");
    CHECK(c.operation == REMOTE_CONFIG_STEERING_GEOMETRY && c.geometry.wheelbase_tenths_mm == 4455);
    c = parse_valid("config steering input 2050 1518 987 0");
    CHECK(c.operation == REMOTE_CONFIG_STEERING_INPUT && c.steering_input.deadband_us == 0);
    c = parse_valid("config steering servo left 2000 1500 1000 30.1 40.5");
    CHECK(c.servo.left_tenths_deg == 301 && c.servo.right_tenths_deg == 405);
    c = parse_valid("config steering ackermann-strength 0:100 1:80 2:60 3:40 4:20 5:10 6:5 300:0");
    CHECK(c.point_count == 8);
    const char *invalid[] = {
        "config steering linkage 90.1 68", "config steering linkage 90 90.1",
        "config steering linkage -1 68", "config steering linkage 90 -1",
        "config steering linkage 90 68.01", "config steering linkage 90 nan",
        "config steering linkage inf 68", "config steering linkage 90",
        "config steering linkage 90 68 extra", "config steering linkage 65536 68",
        "config steering linkage 90 1e30",
        "config steering trim left 1901", "config steering trim right -1901",
        "config steering servo left 2501 1500 500 30 30",
        "config steering servo right 2500 1500 499 30 30",
        "config steering servo left 2500 1500 500 180 180",
        "config steering input 2500 1500 500 30",
        "config steering trim left 25.5", "config steering trim left nan",
        "config steering trim left inf", "config steering trim left 0x10",
        "config steering trim right 1e2", "config steering trim right +",
        "config steering trim right --1", "config steering trim left ",
        "config steering trim right -", "config steering trim both 10",
        "config steering trim left 10 extra", "config steering trim left 65536",
        "config steering trim left -65536", "config steering trim left 999999999999999999999999999",
        "config steering trim left -999999999999999999999999999",
        "config steering ackermann 0:50 100:0",
        "config steering ackermann-strength 0:-0.1 100:0",
        "config steering geometry 0 320 107 30 30",
        "config steering geometry 445 320 107 0 30",
        "config steering geometry 445 320 107 85.1 30",
        "config steering geometry 445 320 501 30 30",
        "config steering geometry 445 320 107 30 30 extra",
        "config steering geometry 445.01 320 107 30 30",
        "config steering geometry 1e30 320 107 30 30",
        "config steering input 2050 1518 987 151",
        "config steering input 1650 1500 1000 150",
        "config steering input 2050 1518 987 30.5",
        "config steering servo left 2000 1500 1000 0 30",
        "config steering servo left 2000 1500 1000 30 86",
        "config steering servo left 2000 1500 1000 30 30 extra",
        "config steering servo left 4294968296 1500 1000 30 30",
        "config steering servo left 2000.1 1500 1000 30 30",
        "config steering servo other 1000 1500 2000",
        "config steering servo left 1490 1500 2000",
        "config steering servo left 1000 900 2000",
        "config steering servo left 799 1500 2000",
        "config steering servo left 4294968296 1500 2000",
        "config steering servo left 1000 1500 2201",
        "config steering servo left 1000 1500 2000 extra",
        "config steering ackermann-strength 0:50",
        "config steering ackermann-strength 1:50 20:0",
        "config steering ackermann-strength 0:50 0:20",
        "config steering ackermann-strength 0:50 20:0 10:0",
        "config steering ackermann-strength 0:50 301:0",
        "config steering ackermann-strength 0:101 20:0",
        "config steering ackermann-strength 0:-101 20:0",
        "config steering ackermann-strength 0:nan 20:0",
        "config steering ackermann-strength 0:50 inf:0",
        "config steering ackermann-strength 0:50 20:0x",
        "config steering ackermann-strength 0:50 20:0 extra",
        "config steering ackermann-strength 0:50 20:0.01",
        "config steering ackermann-strength 0:50 20.01:0",
        "config steering ackermann-strength 0:0 1:0 2:0 3:0 4:0 5:0 6:0 7:0 8:0",
        "cal tv", "cal imu", "tv enable", "tv disable",
        "config steering speed-limit on", "config steering lateral-g 1", "config steering trim 1",
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) expect_invalid(invalid[i]);
}

static void test_calibration_commands(void)
{
    remote_command_t command = parse_valid("cal receiver");
    CHECK(command.operation == REMOTE_CALIBRATE_THROTTLE);
    command = parse_valid("cal steering");
    CHECK(command.operation == REMOTE_CALIBRATE_STEERING);
    command = parse_valid("cal arm");
    CHECK(command.operation == REMOTE_CALIBRATE_ARM);
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
    expect_invalid("config reverse -0");
    expect_invalid("config steering servo left -4294966296 1500 2000");
    expect_invalid("config steering ackermann-strength 0:40 20:0 50:");
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
    expect_invalid("config logging rate 5");
    expect_invalid("config logging rate 0");
    expect_invalid("config logging rate 51");
    expect_invalid("config reverse 10 extra");
}

int main(void)
{
    test_drivetrain_and_scalar_commands();
    test_failsafe_and_boolean_commands();
    test_independent_steering_commands();
    test_calibration_commands();
    test_rejects_drive_and_malformed_configuration();
    return 0;
}
