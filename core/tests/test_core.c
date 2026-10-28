/*
 * Minimal plain-C smoke test for the core. The full behavioral test suite
 * lives in tests/ (pytest, driving this same C code through ctypes).
 * Values here are SYNTHETIC test values, not vehicle thresholds.
 */
#include "aero_core.h"

#include <stdio.h>

static int failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static void synthetic_config(aero_config_t *c)
{
    unsigned i;
    aero_config_init_tbd(c);
    for (i = 0; i < AERO_SIG_COUNT; i++) {
        c->signal_timeout_ms[i] = 50;
        c->signal_min[i] = -1000.0f;
        c->signal_max[i] = 1000.0f;
    }
    c->can_bus_timeout_ms = 100;
    c->inputs_stable_ms = 20;
    c->fault_recovery_stable_ms = 50;
    c->ld_entry_speed_mps = 20.0f;
    c->ld_exit_speed_mps = 18.0f;
    c->ld_entry_throttle = 0.9f;
    c->ld_exit_throttle = 0.8f;
    c->ld_entry_debounce_ms = 30;
    c->hdf_brake_bar = 2.0f;
    c->hdf_steering_rad = 0.1f;
    c->hdf_yaw_rate_rps = 0.2f;
    c->hdf_lat_accel_mps2 = 3.0f;
    c->channel_count = 3;
    for (i = 0; i < 3; i++) {
        c->hdf_position_deg[i] = 0.0f;
        c->ld_position_deg[i] = 30.0f;
    }
    c->position_tolerance_deg = 1.0f;
    c->motion_timeout_ms = 300;
}

static void set_inputs(aero_inputs_t *in, uint32_t t, float speed, float throttle, float brake)
{
    const float v[AERO_SIG_COUNT] = {speed, throttle, brake, 0.0f, 0.0f, 0.0f};
    unsigned i;
    for (i = 0; i < AERO_SIG_COUNT; i++) {
        in->signal[i].value = v[i];
        in->signal[i].timestamp_ms = t;
        in->signal[i].received = true;
        in->signal[i].source_valid = true;
    }
    in->can_last_rx_ms = t;
    in->can_ever_rx = true;
}

int main(void)
{
    aero_config_t cfg;
    aero_supervisor_t sup;
    aero_inputs_t in = {0};
    const aero_output_t *out = NULL;
    uint32_t t;
    unsigned i;

    /* TBD config refuses to leave POWER_ON_INIT. */
    aero_config_init_tbd(&cfg);
    CHECK(!aero_config_is_complete(&cfg));
    aero_init(&sup, &cfg, 0);
    in.actuator_supply_good = true;
    out = aero_tick(&sup, &in, 5);
    CHECK(out->state == AERO_STATE_POWER_ON_INIT);
    CHECK(out->reason == AERO_REASON_CONFIG_INCOMPLETE);

    /* Synthetic config: boot -> wait -> HDF -> LD -> braking -> HDF. */
    synthetic_config(&cfg);
    CHECK(aero_config_is_complete(&cfg));
    aero_init(&sup, &cfg, 0);
    for (i = 0; i < AERO_MAX_CHANNELS; i++) {
        in.channel[i].output_sensor_ok = true;
        in.channel[i].comms_ok = true;
        in.channel[i].measured_output_deg = 0.0f;
    }
    for (t = 5; t <= 100; t += 5) {
        set_inputs(&in, t, 10.0f, 0.2f, 0.0f);
        out = aero_tick(&sup, &in, t);
    }
    CHECK(out->state == AERO_STATE_HIGH_DOWNFORCE);
    CHECK(out->output_confirmed);

    for (; t <= 200; t += 5) {
        set_inputs(&in, t, 30.0f, 1.0f, 0.0f);
        out = aero_tick(&sup, &in, t);
    }
    CHECK(out->state == AERO_STATE_REDUCED_DRAG);
    CHECK(out->target == AERO_TARGET_LD);

    set_inputs(&in, t, 30.0f, 1.0f, 10.0f);
    out = aero_tick(&sup, &in, t);
    CHECK(out->state == AERO_STATE_HIGH_DOWNFORCE);
    CHECK(out->reason == AERO_REASON_STABILITY_BRAKING);

    printf("%s (%d failures)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}
