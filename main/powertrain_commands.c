#include "powertrain_controller.h"

bool powertrain_apply_command(const remote_command_t *command)
{
    switch (command->operation) {
    case REMOTE_DISARM_FOR_CONFIG:
        return powertrain_disarm_for_configuration();
    case REMOTE_CONFIG_DRIVETRAIN:
        return powertrain_set_drivetrain_mode(command->drivetrain_mode);
    case REMOTE_CONFIG_REVERSE_LIMIT:
        return powertrain_set_reverse_limit((uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_TANK_LIMIT:
        return powertrain_set_tank_limit((uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_DRIVE_SMOOTHING:
        return powertrain_set_drive_smoothing(
            (uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_FAILSAFE:
        return powertrain_set_failsafe((uint16_t)command->unsigned_values[0],
                                       (uint16_t)command->unsigned_values[1]);
    case REMOTE_CONFIG_FAILSAFE_ENABLED:
        return powertrain_set_failsafe_enabled(command->enabled);
    case REMOTE_CONFIG_RPM_POLES:
        return powertrain_set_motor_poles((uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_RPM_PPR:
        return powertrain_set_rpm_pulses_per_revolution(
            (uint16_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_STEERING_SMOOTHING:
        return powertrain_set_steering_smoothing(
            (uint16_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_STEERING_SERVO:
        return powertrain_set_steering_servo((uint8_t)command->unsigned_values[0], &command->servo);
    case REMOTE_CONFIG_WHEEL_CALIBRATION:
        return powertrain_set_wheel_calibration((uint8_t)command->unsigned_values[0], &command->wheel_calibration,
                                                command->geometry.max_left_tenths_deg, command->geometry.max_right_tenths_deg);
    case REMOTE_CONFIG_STEERING_TRIM:
        return powertrain_set_steering_trim((uint8_t)command->unsigned_values[0], command->steering_trim_us);
    case REMOTE_CONFIG_STEERING_LINKAGE:
        return powertrain_set_steering_linkage(&command->linkage);
    case REMOTE_CONFIG_STEERING_TOE:
        return powertrain_set_steering_toe(command->toe_tenths_deg);
    case REMOTE_CONFIG_STEERING_GEOMETRY:
        return powertrain_set_steering_geometry(&command->geometry);
    case REMOTE_CONFIG_STEERING_INPUT:
        return powertrain_set_steering_input(&command->steering_input);
    case REMOTE_CONFIG_ACKERMANN:
        return powertrain_set_ackermann_curve(command->points, command->point_count);
    case REMOTE_CONFIG_STEERING_LIMIT:
        return powertrain_set_steering_limit_curve(command->steering_limit.points,
                                                  command->steering_limit.point_count);
    case REMOTE_CONFIG_ARM_LATCH:
        return powertrain_set_permanent_arm_latch_enabled(command->enabled);
    case REMOTE_CALIBRATE_THROTTLE:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_THROTTLE);
    case REMOTE_CALIBRATE_STEERING:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_STEERING);
    case REMOTE_CALIBRATE_ARM:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_ARM);
    case REMOTE_CALIBRATE_WHEEL_LEFT:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_WHEEL_LEFT);
    case REMOTE_CALIBRATE_WHEEL_RIGHT:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_WHEEL_RIGHT);
    case REMOTE_CALIBRATE_CAPTURE:
        return powertrain_remote_calibration_capture();
    case REMOTE_CALIBRATE_ESC_ARM:
        return powertrain_cal_esc_arm();
    case REMOTE_CALIBRATE_ESC_MAX:
        return powertrain_cal_esc_max();
    case REMOTE_CALIBRATE_ESC_MIN:
        return powertrain_cal_esc_min();
    case REMOTE_CALIBRATE_ESC_MANUAL:
        return powertrain_cal_manual();
    case REMOTE_CALIBRATE_CANCEL:
        return powertrain_cal_cancel();
    default:
        return false;
    }
}
