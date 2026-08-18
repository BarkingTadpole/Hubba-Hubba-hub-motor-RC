#include "steering_center_guard.h"

#define CHECK(condition)            \
    do {                            \
        if (!(condition)) {         \
            __builtin_trap();       \
        }                           \
    } while (0)

#define CENTER_US 1503U
#define DEADBAND_US 30U

static bool accept(steering_center_guard_t *state,
                   uint16_t pulse_us,
                   int64_t frame_at_us)
{
    return steering_center_guard_accept(state,
                                        pulse_us,
                                        CENTER_US,
                                        DEADBAND_US,
                                        frame_at_us);
}

static void test_center_and_observed_single_frame_spikes_are_rejected(void)
{
    steering_center_guard_t state;
    steering_center_guard_init(&state);

    CHECK(!accept(&state, 1514, 20000));
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_LOCKED);

    CHECK(!accept(&state, 1837, 40000));
    CHECK(steering_center_guard_status(&state) ==
          STEERING_CENTER_GUARD_PENDING_HIGH);
    CHECK(!accept(&state, 1515, 60000));
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_LOCKED);
    CHECK(state.rejected_spike_count == 1);

    CHECK(!accept(&state, 1399, 80000));
    CHECK(steering_center_guard_status(&state) ==
          STEERING_CENTER_GUARD_PENDING_LOW);
    CHECK(!accept(&state, 1514, 100000));
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_LOCKED);
    CHECK(state.rejected_spike_count == 2);
}

static void test_duplicate_loop_read_does_not_confirm_a_frame(void)
{
    steering_center_guard_t state;
    steering_center_guard_init(&state);

    CHECK(!accept(&state, 1944, 20000));
    CHECK(!accept(&state, 1944, 20000));
    CHECK(steering_center_guard_status(&state) ==
          STEERING_CENTER_GUARD_PENDING_HIGH);
    CHECK(!accept(&state, 1514, 40000));
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_LOCKED);
    CHECK(state.rejected_spike_count == 1);
}

static void test_two_same_side_frames_release_steering(void)
{
    steering_center_guard_t state;
    steering_center_guard_init(&state);

    CHECK(!accept(&state, 1600, 20000));
    CHECK(accept(&state, 1750, 40000));
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_TRACKING);
    CHECK(accept(&state, 1900, 60000));
    CHECK(state.rejected_spike_count == 0);
}

static void test_opposite_side_frame_starts_new_confirmation(void)
{
    steering_center_guard_t state;
    steering_center_guard_init(&state);

    CHECK(!accept(&state, 1800, 20000));
    CHECK(!accept(&state, 1200, 40000));
    CHECK(steering_center_guard_status(&state) ==
          STEERING_CENTER_GUARD_PENDING_LOW);
    CHECK(state.rejected_spike_count == 1);
    CHECK(accept(&state, 1100, 60000));
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_TRACKING);
}

static void test_center_target_is_immediate_and_relocks_after_two_frames(void)
{
    steering_center_guard_t state;
    steering_center_guard_init(&state);

    CHECK(!accept(&state, 1600, 20000));
    CHECK(accept(&state, 1700, 40000));

    CHECK(!accept(&state, 1514, 60000));
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_TRACKING);
    CHECK(!accept(&state, 1515, 80000));
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_LOCKED);
}

static void test_recenter_preserves_diagnostics_and_requires_fresh_frames(void)
{
    steering_center_guard_t state;
    steering_center_guard_init(&state);

    CHECK(!accept(&state, 1850, 20000));
    CHECK(!accept(&state, 1514, 40000));
    CHECK(state.rejected_spike_count == 1);

    CHECK(!accept(&state, 1200, 60000));
    steering_center_guard_recenter(&state);
    CHECK(steering_center_guard_status(&state) == STEERING_CENTER_GUARD_LOCKED);
    CHECK(state.rejected_spike_count == 1);

    CHECK(!accept(&state, 1200, 60000));
    CHECK(accept(&state, 1200, 80000));
}

int main(void)
{
    test_center_and_observed_single_frame_spikes_are_rejected();
    test_duplicate_loop_read_does_not_confirm_a_frame();
    test_two_same_side_frames_release_steering();
    test_opposite_side_frame_starts_new_confirmation();
    test_center_target_is_immediate_and_relocks_after_two_frames();
    test_recenter_preserves_diagnostics_and_requires_fresh_frames();
    return 0;
}
