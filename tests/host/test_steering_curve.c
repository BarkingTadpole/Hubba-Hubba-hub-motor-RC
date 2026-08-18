#include "steering_curve.h"

#define CHECK(condition)            \
    do {                            \
        if (!(condition)) {         \
            __builtin_trap();       \
        }                           \
    } while (0)

static float absolute(float value)
{
    return value < 0.0f ? -value : value;
}

static bool near(float actual, float expected, float tolerance)
{
    return absolute(actual - expected) <= tolerance;
}

static void test_center_preserves_toe_out_and_zero_mean(void)
{
    steering_curve_sample_t sample;
    CHECK(steering_curve_sample(0.0f, &sample));
    CHECK(sample.valid);
    CHECK(!sample.saturated);
    CHECK(sample.servo_command_deg == 0.0f);
    CHECK(near(sample.left_wheel_deg, -1.640f, 0.0001f));
    CHECK(near(sample.right_wheel_deg, 1.640f, 0.0001f));
    CHECK(sample.average_wheel_deg == 0.0f);
    CHECK(sample.normalized_average == 0.0f);
}

static void test_right_endpoint_and_coordinate_conversion(void)
{
    steering_curve_sample_t sample;
    CHECK(steering_curve_sample(45.0f, &sample));
    CHECK(near(sample.left_wheel_deg, 29.906f, 0.0001f));
    CHECK(near(sample.right_wheel_deg, 29.570f, 0.0001f));
    CHECK(near(sample.average_wheel_deg, 29.738f, 0.0001f));
    CHECK(near(sample.normalized_average, 1.0f, 0.0001f));
}

static void test_left_endpoint_and_coordinate_conversion(void)
{
    steering_curve_sample_t sample;
    CHECK(steering_curve_sample(-45.0f, &sample));
    CHECK(near(sample.left_wheel_deg, -30.929f, 0.0001f));
    CHECK(near(sample.right_wheel_deg, -31.229f, 0.0001f));
    CHECK(near(sample.average_wheel_deg, -31.079f, 0.0001f));
    CHECK(near(sample.normalized_average, -1.0f, 0.0001f));
}

static void test_piecewise_linear_interpolation_on_nonuniform_segment(void)
{
    steering_curve_sample_t sample;
    CHECK(steering_curve_sample(41.5f, &sample));
    CHECK(near(sample.left_wheel_deg, 27.926f, 0.0001f));
    CHECK(near(sample.right_wheel_deg, 27.675f, 0.0001f));
    CHECK(near(sample.average_wheel_deg, 27.8005f, 0.0001f));
    CHECK(sample.normalized_average > 0.0f);
    CHECK(sample.normalized_average < 1.0f);
}

static void test_near_center_interpolation_preserves_toe_geometry(void)
{
    steering_curve_sample_t sample;
    CHECK(steering_curve_sample(1.0f, &sample));
    CHECK(near(sample.left_wheel_deg, -0.8200f, 0.0001f));
    CHECK(near(sample.right_wheel_deg, 2.3990f, 0.0001f));
    CHECK(near(sample.average_wheel_deg, 0.7895f, 0.0001f));
    CHECK(sample.normalized_average > 0.0f);
}

static void test_saturation_and_non_finite_rejection(void)
{
    steering_curve_sample_t sample;
    CHECK(steering_curve_sample(100.0f, &sample));
    CHECK(sample.valid);
    CHECK(sample.saturated);
    CHECK(sample.servo_command_deg == 45.0f);
    CHECK(near(sample.normalized_average, 1.0f, 0.0001f));

    CHECK(steering_curve_sample(-100.0f, &sample));
    CHECK(sample.valid);
    CHECK(sample.saturated);
    CHECK(sample.servo_command_deg == -45.0f);
    CHECK(near(sample.normalized_average, -1.0f, 0.0001f));

    CHECK(!steering_curve_sample(__builtin_nanf(""), &sample));
    CHECK(!sample.valid);
    CHECK(sample.normalized_average == 0.0f);
}

int main(void)
{
    test_center_preserves_toe_out_and_zero_mean();
    test_right_endpoint_and_coordinate_conversion();
    test_left_endpoint_and_coordinate_conversion();
    test_piecewise_linear_interpolation_on_nonuniform_segment();
    test_near_center_interpolation_preserves_toe_geometry();
    test_saturation_and_non_finite_rejection();
    return 0;
}
