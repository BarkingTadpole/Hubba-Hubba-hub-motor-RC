#include "steering_input_filter.h"

#define CHECK(condition)            \
    do {                            \
        if (!(condition)) {         \
            __builtin_trap();       \
        }                           \
    } while (0)

#define OUTLIER_THRESHOLD_US 30U

static bool update(steering_input_filter_t *state,
                   uint16_t pulse_us,
                   int64_t frame_at_us,
                   uint16_t *filtered_us)
{
    return steering_input_filter_update(state,
                                        pulse_us,
                                        OUTLIER_THRESHOLD_US,
                                        frame_at_us,
                                        filtered_us);
}

static void test_requires_three_distinct_frames(void)
{
    steering_input_filter_t state;
    uint16_t filtered_us = 0;
    steering_input_filter_init(&state);

    CHECK(!update(&state, 1500, 20000, &filtered_us));
    CHECK(!update(&state, 1501, 20000, &filtered_us));
    CHECK(!update(&state, 1501, 40000, &filtered_us));
    CHECK(steering_input_filter_status(&state) == STEERING_INPUT_FILTER_WARMUP);
    CHECK(update(&state, 1502, 60000, &filtered_us));
    CHECK(filtered_us == 1501);
    CHECK(steering_input_filter_status(&state) == STEERING_INPUT_FILTER_ACTIVE);
}

static void test_observed_center_spikes_are_rejected(void)
{
    steering_input_filter_t state;
    uint16_t filtered_us = 0;
    steering_input_filter_init(&state);

    CHECK(!update(&state, 1514, 20000, &filtered_us));
    CHECK(!update(&state, 1837, 40000, &filtered_us));
    CHECK(update(&state, 1515, 60000, &filtered_us));
    CHECK(filtered_us == 1515);
    CHECK(state.rejected_spike_count == 1);

    CHECK(update(&state, 1514, 80000, &filtered_us));
    CHECK(filtered_us == 1515);
    CHECK(update(&state, 1399, 100000, &filtered_us));
    CHECK(filtered_us == 1514);
    CHECK(update(&state, 1515, 120000, &filtered_us));
    CHECK(filtered_us == 1514);
    CHECK(state.rejected_spike_count == 2);
}

static void test_isolated_spike_is_rejected_away_from_center(void)
{
    steering_input_filter_t state;
    uint16_t filtered_us = 0;
    steering_input_filter_init(&state);

    CHECK(!update(&state, 1800, 20000, &filtered_us));
    CHECK(!update(&state, 1200, 40000, &filtered_us));
    CHECK(update(&state, 1802, 60000, &filtered_us));
    CHECK(filtered_us == 1800);
    CHECK(state.rejected_spike_count == 1);
}

static void test_active_filter_rejects_spikes_while_held_off_center(void)
{
    steering_input_filter_t state;
    uint16_t filtered_us = 0;
    steering_input_filter_init(&state);

    CHECK(!update(&state, 1800, 20000, &filtered_us));
    CHECK(!update(&state, 1801, 40000, &filtered_us));
    CHECK(update(&state, 1800, 60000, &filtered_us));
    CHECK(filtered_us == 1800);

    CHECK(update(&state, 1200, 80000, &filtered_us));
    CHECK(filtered_us == 1800);
    CHECK(update(&state, 1802, 100000, &filtered_us));
    CHECK(filtered_us == 1800);
    CHECK(state.rejected_spike_count == 1);

    CHECK(update(&state, 1801, 120000, &filtered_us));
    CHECK(filtered_us == 1801);
    CHECK(update(&state, 2150, 140000, &filtered_us));
    CHECK(filtered_us == 1802);
    CHECK(update(&state, 1800, 160000, &filtered_us));
    CHECK(filtered_us == 1801);
    CHECK(state.rejected_spike_count == 2);
}

static void test_real_step_has_one_frame_delay(void)
{
    steering_input_filter_t state;
    uint16_t filtered_us = 0;
    steering_input_filter_init(&state);

    CHECK(!update(&state, 1500, 20000, &filtered_us));
    CHECK(!update(&state, 1500, 40000, &filtered_us));
    CHECK(update(&state, 1500, 60000, &filtered_us));
    CHECK(filtered_us == 1500);

    CHECK(update(&state, 1900, 80000, &filtered_us));
    CHECK(filtered_us == 1500);
    CHECK(update(&state, 1900, 100000, &filtered_us));
    CHECK(filtered_us == 1900);
    CHECK(state.rejected_spike_count == 0);
}

static void test_ramp_is_delayed_by_one_frame_without_stalling(void)
{
    steering_input_filter_t state;
    uint16_t filtered_us = 0;
    steering_input_filter_init(&state);

    CHECK(!update(&state, 1500, 20000, &filtered_us));
    CHECK(!update(&state, 1600, 40000, &filtered_us));
    CHECK(update(&state, 1700, 60000, &filtered_us));
    CHECK(filtered_us == 1600);
    CHECK(update(&state, 1800, 80000, &filtered_us));
    CHECK(filtered_us == 1700);
    CHECK(update(&state, 1900, 100000, &filtered_us));
    CHECK(filtered_us == 1800);
}

static void test_reset_preserves_diagnostics_and_restarts_warmup(void)
{
    steering_input_filter_t state;
    uint16_t filtered_us = 0;
    steering_input_filter_init(&state);

    CHECK(!update(&state, 1700, 20000, &filtered_us));
    CHECK(!update(&state, 1100, 40000, &filtered_us));
    CHECK(update(&state, 1701, 60000, &filtered_us));
    CHECK(state.rejected_spike_count == 1);

    steering_input_filter_reset(&state);
    CHECK(state.rejected_spike_count == 1);
    CHECK(steering_input_filter_status(&state) == STEERING_INPUT_FILTER_WARMUP);
    CHECK(!update(&state, 1800, 60000, &filtered_us));
    CHECK(!update(&state, 1800, 80000, &filtered_us));
    CHECK(update(&state, 1800, 100000, &filtered_us));
    CHECK(filtered_us == 1800);
}

int main(void)
{
    test_requires_three_distinct_frames();
    test_observed_center_spikes_are_rejected();
    test_isolated_spike_is_rejected_away_from_center();
    test_active_filter_rejects_spikes_while_held_off_center();
    test_real_step_has_one_frame_delay();
    test_ramp_is_delayed_by_one_frame_without_stalling();
    test_reset_preserves_diagnostics_and_restarts_warmup();
    return 0;
}
