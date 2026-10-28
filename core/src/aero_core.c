/*
 * aero_core.c - Active aero supervisory state machine (portable C99 core).
 *
 * PRELIMINARY / PLACEHOLDER SOFTWARE. Not validated, not vehicle-ready.
 *
 * Tick order (explicit, so there are no hidden one-cycle races):
 *   1. validate signals (freshness, source flag, range)
 *   2. gather fault conditions (CAN silence, supply, comms, output sensor,
 *      plus the motion-timeout result computed at the end of the PREVIOUS tick)
 *   3. latch faults / honor reset requests
 *   4. evaluate priority rules -> next state + reason
 *   5. derive target and per-channel commands
 *   6. evaluate per-channel position evidence (may raise motion timeout,
 *      which is acted on next tick)
 */
#include "aero_core.h"

#include <math.h>
#include <string.h>

static bool elapsed(uint32_t now, uint32_t since, uint32_t duration)
{
    /* Unsigned subtraction handles uint32 wraparound of the monotonic clock. */
    return (uint32_t)(now - since) >= duration;
}

static bool is_set_f(float v) { return !isnan(v); }
static bool is_set_ms(uint32_t v) { return v != AERO_TBD_MS; }

/* ----------------------------------------------------------------- config */

void aero_config_init_tbd(aero_config_t *cfg)
{
    uint32_t i;
    memset(cfg, 0, sizeof(*cfg));
    for (i = 0; i < AERO_SIG_COUNT; i++) {
        cfg->signal_required[i] = true; /* conservative until the team decides */
        cfg->signal_timeout_ms[i] = AERO_TBD_MS;
        cfg->signal_min[i] = NAN;
        cfg->signal_max[i] = NAN;
    }
    cfg->can_bus_timeout_ms = AERO_TBD_MS;
    cfg->inputs_stable_ms = AERO_TBD_MS;
    cfg->fault_recovery_stable_ms = AERO_TBD_MS;
    cfg->ld_entry_speed_mps = NAN;
    cfg->ld_exit_speed_mps = NAN;
    cfg->ld_entry_throttle = NAN;
    cfg->ld_exit_throttle = NAN;
    cfg->ld_entry_debounce_ms = AERO_TBD_MS;
    cfg->hdf_brake_bar = NAN;
    cfg->hdf_steering_rad = NAN;
    cfg->hdf_yaw_rate_rps = NAN;
    cfg->hdf_lat_accel_mps2 = NAN;
    cfg->channel_count = 0;
    for (i = 0; i < AERO_MAX_CHANNELS; i++) {
        cfg->hdf_position_deg[i] = NAN;
        cfg->ld_position_deg[i] = NAN;
    }
    cfg->position_tolerance_deg = NAN;
    cfg->motion_timeout_ms = AERO_TBD_MS;
    cfg->variable_aero_enabled = false;
}

bool aero_config_is_complete(const aero_config_t *cfg)
{
    uint32_t i;
    for (i = 0; i < AERO_SIG_COUNT; i++) {
        if (!cfg->signal_required[i]) {
            continue;
        }
        if (!is_set_ms(cfg->signal_timeout_ms[i]) || !is_set_f(cfg->signal_min[i]) ||
            !is_set_f(cfg->signal_max[i]) || !(cfg->signal_min[i] < cfg->signal_max[i])) {
            return false;
        }
    }
    if (!is_set_ms(cfg->can_bus_timeout_ms) || !is_set_ms(cfg->inputs_stable_ms) ||
        !is_set_ms(cfg->fault_recovery_stable_ms) || !is_set_ms(cfg->ld_entry_debounce_ms) ||
        !is_set_ms(cfg->motion_timeout_ms)) {
        return false;
    }
    if (!is_set_f(cfg->ld_entry_speed_mps) || !is_set_f(cfg->ld_exit_speed_mps) ||
        !is_set_f(cfg->ld_entry_throttle) || !is_set_f(cfg->ld_exit_throttle) ||
        !is_set_f(cfg->hdf_brake_bar) || !is_set_f(cfg->hdf_steering_rad) ||
        !is_set_f(cfg->hdf_yaw_rate_rps) || !is_set_f(cfg->hdf_lat_accel_mps2) ||
        !is_set_f(cfg->position_tolerance_deg)) {
        return false;
    }
    /* Hysteresis must not be inverted. */
    if (cfg->ld_exit_speed_mps > cfg->ld_entry_speed_mps ||
        cfg->ld_exit_throttle > cfg->ld_entry_throttle) {
        return false;
    }
    if (!(cfg->position_tolerance_deg > 0.0f)) {
        return false;
    }
    if (cfg->channel_count < 3u || cfg->channel_count > AERO_MAX_CHANNELS) {
        return false;
    }
    for (i = 0; i < cfg->channel_count; i++) {
        if (!is_set_f(cfg->hdf_position_deg[i]) || !is_set_f(cfg->ld_position_deg[i])) {
            return false;
        }
    }
    /* No approved variable-aero map exists yet. */
    if (cfg->variable_aero_enabled) {
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ init */

void aero_init(aero_supervisor_t *sup, const aero_config_t *cfg, uint32_t now_ms)
{
    uint32_t i;
    memset(sup, 0, sizeof(*sup));
    sup->cfg = *cfg;
    sup->cfg_valid = aero_config_is_complete(cfg);
    sup->out.timestamp_ms = now_ms;
    sup->out.state = AERO_STATE_POWER_ON_INIT;
    sup->out.previous_state = AERO_STATE_POWER_ON_INIT;
    sup->out.reason = sup->cfg_valid ? AERO_REASON_NONE : AERO_REASON_CONFIG_INCOMPLETE;
    sup->out.target = AERO_TARGET_NONE;
    sup->out.state_entered_ms = now_ms;
    for (i = 0; i < AERO_MAX_CHANNELS; i++) {
        sup->out.channel_command_deg[i] = NAN;
        sup->out.channel_status[i] = AERO_POS_NOT_COMMANDED;
    }
    for (i = 0; i < AERO_SIG_COUNT; i++) {
        sup->out.signal_status[i] = (uint8_t)AERO_SIGSTAT_NEVER_RECEIVED;
    }
    sup->last_target = AERO_TARGET_NONE;
}

/* ------------------------------------------------------------- helpers */

static aero_signal_status_t classify_signal(const aero_config_t *cfg, aero_signal_id_t id,
                                            const aero_signal_sample_t *s, uint32_t now_ms)
{
    if (!s->received) {
        return AERO_SIGSTAT_NEVER_RECEIVED;
    }
    if (is_set_ms(cfg->signal_timeout_ms[id]) &&
        elapsed(now_ms, s->timestamp_ms, cfg->signal_timeout_ms[id] + 1u)) {
        return AERO_SIGSTAT_STALE;
    }
    if (!s->source_valid) {
        return AERO_SIGSTAT_SOURCE_INVALID;
    }
    if (isnan(s->value) || s->value < cfg->signal_min[id] || s->value > cfg->signal_max[id]) {
        return AERO_SIGSTAT_FRESH_IMPLAUSIBLE;
    }
    return AERO_SIGSTAT_FRESH_VALID;
}

static float sig(const aero_inputs_t *in, aero_signal_id_t id) { return in->signal[id].value; }

/* Returns a STABILITY_* reason if HDF is demanded, else AERO_REASON_NONE. */
static aero_reason_t stability_demand(const aero_config_t *cfg, const aero_inputs_t *in)
{
    if (sig(in, AERO_SIG_BRAKE) > cfg->hdf_brake_bar) {
        return AERO_REASON_STABILITY_BRAKING;
    }
    if (fabsf(sig(in, AERO_SIG_STEERING)) > cfg->hdf_steering_rad) {
        return AERO_REASON_STABILITY_STEERING;
    }
    if (fabsf(sig(in, AERO_SIG_YAW_RATE)) > cfg->hdf_yaw_rate_rps) {
        return AERO_REASON_STABILITY_YAW;
    }
    if (fabsf(sig(in, AERO_SIG_LAT_ACCEL)) > cfg->hdf_lat_accel_mps2) {
        return AERO_REASON_STABILITY_LAT_ACCEL;
    }
    return AERO_REASON_NONE;
}

static bool ld_entry_conditions(const aero_config_t *cfg, const aero_inputs_t *in)
{
    return sig(in, AERO_SIG_SPEED) >= cfg->ld_entry_speed_mps &&
           sig(in, AERO_SIG_THROTTLE) >= cfg->ld_entry_throttle;
}

static bool ld_hold_conditions(const aero_config_t *cfg, const aero_inputs_t *in)
{
    return sig(in, AERO_SIG_SPEED) >= cfg->ld_exit_speed_mps &&
           sig(in, AERO_SIG_THROTTLE) >= cfg->ld_exit_throttle;
}

static aero_target_t target_for_state(aero_state_t s)
{
    switch (s) {
    case AERO_STATE_HIGH_DOWNFORCE:
    case AERO_STATE_INHIBITED:
    case AERO_STATE_COMMANDED_FALLBACK:
        return AERO_TARGET_HDF;
    case AERO_STATE_REDUCED_DRAG:
        return AERO_TARGET_LD;
    default:
        return AERO_TARGET_NONE;
    }
}

/* ---------------------------------------------------- running-state rules */

typedef struct {
    aero_state_t state;
    aero_reason_t reason;
} decision_t;

static decision_t decide_running(aero_supervisor_t *sup, const aero_inputs_t *in, uint32_t now_ms,
                                 bool inputs_valid, uint32_t active)
{
    const aero_config_t *cfg = &sup->cfg;
    aero_state_t cur = sup->out.state;
    decision_t d;
    d.state = cur;
    d.reason = AERO_REASON_NONE;

    /* Priority order. The first matching rule wins. */
    if ((active & AERO_FAULT_ACTUATOR_SUPPLY) != 0u) {
        d.state = AERO_STATE_COMMANDED_FALLBACK;
        d.reason = AERO_REASON_ACTUATOR_SUPPLY_LOST;
    } else if (in->manual_inhibit) {
        d.state = AERO_STATE_INHIBITED;
        d.reason = AERO_REASON_MANUAL_INHIBIT;
    } else if (sup->out.latched_faults != 0u) {
        d.state = AERO_STATE_COMMANDED_FALLBACK;
        d.reason = AERO_REASON_LATCHED_FAULT;
    } else if ((active & AERO_FAULT_CAN_BUS_SILENT) != 0u) {
        d.state = AERO_STATE_COMMANDED_FALLBACK;
        d.reason = AERO_REASON_CAN_BUS_SILENT;
    } else if (cur == AERO_STATE_COMMANDED_FALLBACK) {
        /* No faults now: recoverable faults must stay clear for a stable window. */
        if (!sup->faults_clear_timing) {
            sup->faults_clear_timing = true;
            sup->faults_clear_since_ms = now_ms;
        }
        if (elapsed(now_ms, sup->faults_clear_since_ms, cfg->fault_recovery_stable_ms)) {
            d.state = AERO_STATE_WAIT_FOR_VALID_INPUTS;
            d.reason = AERO_REASON_FAULT_RECOVERED;
        } else {
            d.reason = AERO_REASON_FAULT_RECOVERING;
        }
    } else if (!inputs_valid) {
        if (cur == AERO_STATE_WAIT_FOR_VALID_INPUTS) {
            d.reason = AERO_REASON_WAITING_FOR_INPUTS;
        } else {
            d.state = AERO_STATE_INHIBITED;
            d.reason = AERO_REASON_REQUIRED_INPUT_INVALID;
        }
    } else if (cur == AERO_STATE_INHIBITED) {
        d.state = AERO_STATE_WAIT_FOR_VALID_INPUTS;
        d.reason = AERO_REASON_INHIBIT_CLEARED;
    } else if (cur == AERO_STATE_WAIT_FOR_VALID_INPUTS) {
        if (!sup->inputs_valid_timing) {
            sup->inputs_valid_timing = true;
            sup->inputs_valid_since_ms = now_ms;
        }
        if (elapsed(now_ms, sup->inputs_valid_since_ms, cfg->inputs_stable_ms)) {
            d.state = AERO_STATE_HIGH_DOWNFORCE;
            d.reason = AERO_REASON_INPUTS_VALID;
        } else {
            d.reason = AERO_REASON_INPUTS_STABILIZING;
        }
    } else {
        /* HIGH_DOWNFORCE / REDUCED_DRAG (variable aero is never entered). */
        aero_reason_t stab = stability_demand(cfg, in);
        if (stab != AERO_REASON_NONE) {
            /* Safety exit: never delayed by debounce or dwell. */
            d.state = AERO_STATE_HIGH_DOWNFORCE;
            d.reason = stab;
        } else if (cur == AERO_STATE_REDUCED_DRAG) {
            if (ld_hold_conditions(cfg, in)) {
                d.reason = AERO_REASON_LD_ELIGIBLE;
            } else {
                d.state = AERO_STATE_HIGH_DOWNFORCE;
                d.reason = AERO_REASON_LD_CONDITIONS_LOST;
            }
        } else if (ld_entry_conditions(cfg, in)) {
            if (!sup->ld_eligible_timing) {
                sup->ld_eligible_timing = true;
                sup->ld_eligible_since_ms = now_ms;
            }
            if (elapsed(now_ms, sup->ld_eligible_since_ms, cfg->ld_entry_debounce_ms)) {
                d.state = AERO_STATE_REDUCED_DRAG;
                d.reason = AERO_REASON_LD_ELIGIBLE;
            } else {
                d.state = AERO_STATE_HIGH_DOWNFORCE;
                d.reason = AERO_REASON_LD_DEBOUNCING;
            }
        } else {
            d.state = AERO_STATE_HIGH_DOWNFORCE;
            d.reason = AERO_REASON_HDF_DEFAULT;
        }
    }

    /* Timers only run while their exact condition keeps holding. */
    if (d.reason != AERO_REASON_FAULT_RECOVERING && d.reason != AERO_REASON_FAULT_RECOVERED) {
        sup->faults_clear_timing = false;
    }
    if (d.reason != AERO_REASON_INPUTS_STABILIZING) {
        sup->inputs_valid_timing = false;
    }
    if (d.reason != AERO_REASON_LD_DEBOUNCING) {
        sup->ld_eligible_timing = false;
    }
    return d;
}

/* ------------------------------------------------------------------ tick */

const aero_output_t *aero_tick(aero_supervisor_t *sup, const aero_inputs_t *in, uint32_t now_ms)
{
    const aero_config_t *cfg = &sup->cfg;
    aero_output_t *out = &sup->out;
    aero_state_t cur = out->state;
    decision_t d;
    uint32_t i;
    uint32_t active = 0u;
    uint32_t sig_inhibit = 0u;
    bool inputs_valid = true;
    bool running;
    bool motion_fault = false;
    bool all_confirmed;

    out->timestamp_ms = now_ms;

    /* 1. Signal validation. */
    for (i = 0; i < AERO_SIG_COUNT; i++) {
        aero_signal_status_t st = classify_signal(cfg, (aero_signal_id_t)i, &in->signal[i], now_ms);
        out->signal_status[i] = (uint8_t)st;
        if (cfg->signal_required[i] && st != AERO_SIGSTAT_FRESH_VALID) {
            inputs_valid = false;
            sig_inhibit |= AERO_INHIBIT_SIGNAL(i);
        }
    }

    if (!sup->cfg_valid) {
        /* Refuse to leave POWER_ON_INIT without a complete configuration. */
        out->reason = AERO_REASON_CONFIG_INCOMPLETE;
        out->target = AERO_TARGET_NONE;
        out->command_enabled = false;
        out->output_confirmed = false;
        return out;
    }

    /* 2. Fault conditions present now. */
    if (in->can_ever_rx && elapsed(now_ms, in->can_last_rx_ms, cfg->can_bus_timeout_ms + 1u)) {
        active |= AERO_FAULT_CAN_BUS_SILENT;
    }
    if (!in->actuator_supply_good) {
        active |= AERO_FAULT_ACTUATOR_SUPPLY;
    }
    for (i = 0; i < cfg->channel_count; i++) {
        if (!in->channel[i].comms_ok) {
            active |= AERO_FAULT_ACTUATOR_COMMS;
        }
        if (!in->channel[i].output_sensor_ok) {
            active |= AERO_FAULT_OUTPUT_SENSOR;
        }
    }
    active |= out->active_faults & AERO_FAULT_MOTION_TIMEOUT; /* from previous tick */

    running = (cur != AERO_STATE_POWER_ON_INIT && cur != AERO_STATE_SERVICE_CALIBRATION);

    /* 3. Latch, then honor a deliberate reset only for conditions that are gone. */
    if (running) {
        out->latched_faults |= active & AERO_FAULT_LATCHING_MASK;
        if (cur == AERO_STATE_COMMANDED_FALLBACK && in->fault_reset_request) {
            out->latched_faults &= active;
        }
    }
    out->inhibit_flags = (in->manual_inhibit ? AERO_INHIBIT_MANUAL : 0u) | sig_inhibit;

    /* 4. State decision. */
    d.state = cur;
    d.reason = AERO_REASON_NONE;
    switch (cur) {
    case AERO_STATE_POWER_ON_INIT:
        if (in->service_request) {
            d.state = AERO_STATE_SERVICE_CALIBRATION;
            d.reason = AERO_REASON_SERVICE_ENTER;
        } else {
            d.state = AERO_STATE_WAIT_FOR_VALID_INPUTS;
            d.reason = AERO_REASON_BOOT_COMPLETE;
        }
        break;
    case AERO_STATE_SERVICE_CALIBRATION:
        if (in->service_exit) {
            d.state = AERO_STATE_WAIT_FOR_VALID_INPUTS;
            d.reason = AERO_REASON_SERVICE_EXIT;
        } else {
            d.reason = AERO_REASON_SERVICE_ACTIVE;
        }
        break;
    default:
        d = decide_running(sup, in, now_ms, inputs_valid, active);
        break;
    }

    if (d.state != cur) {
        out->previous_state = cur;
        out->transition_count++;
        out->state_entered_ms = now_ms;
    }
    out->state = d.state;
    out->reason = d.reason;

    /* 5. Target and commands. Service-mode motion is not implemented (TBD). */
    out->target = target_for_state(d.state);
    out->command_enabled = (out->target != AERO_TARGET_NONE) && in->actuator_supply_good;
    if (out->target != sup->last_target) {
        sup->last_target = out->target;
        sup->command_change_ms = now_ms;
    }

    /* 6. Position evidence. Only the independent output sensor counts. */
    all_confirmed = (out->target != AERO_TARGET_NONE);
    for (i = 0; i < AERO_MAX_CHANNELS; i++) {
        aero_pos_status_t ps;
        float cmd;
        if (i >= cfg->channel_count || out->target == AERO_TARGET_NONE) {
            out->channel_command_deg[i] = NAN;
            out->channel_status[i] = AERO_POS_NOT_COMMANDED;
            continue;
        }
        cmd = (out->target == AERO_TARGET_HDF) ? cfg->hdf_position_deg[i] : cfg->ld_position_deg[i];
        out->channel_command_deg[i] = cmd;

        if (!in->channel[i].output_sensor_ok) {
            ps = AERO_POS_UNKNOWN;
        } else if (fabsf(in->channel[i].measured_output_deg - cmd) <= cfg->position_tolerance_deg) {
            ps = AERO_POS_CONFIRMED;
        } else if (!out->command_enabled) {
            ps = AERO_POS_UNCONFIRMED; /* cannot move; no timeout fault since nothing was driven */
        } else if (now_ms == sup->command_change_ms) {
            ps = AERO_POS_REQUESTED;
        } else if (elapsed(now_ms, sup->command_change_ms, cfg->motion_timeout_ms)) {
            ps = AERO_POS_UNCONFIRMED;
            motion_fault = true;
        } else {
            ps = AERO_POS_IN_MOTION;
        }
        out->channel_status[i] = ps;
        if (ps != AERO_POS_CONFIRMED) {
            all_confirmed = false;
        }
    }
    out->output_confirmed = all_confirmed;

    if (motion_fault) {
        active |= AERO_FAULT_MOTION_TIMEOUT;
        if (running) {
            out->latched_faults |= AERO_FAULT_MOTION_TIMEOUT; /* acted on next tick */
        }
    } else {
        active &= ~AERO_FAULT_MOTION_TIMEOUT;
    }
    out->active_faults = active;
    return out;
}

/* ----------------------------------------------------------------- names */

const char *aero_state_name(aero_state_t s)
{
    static const char *const names[AERO_STATE_COUNT] = {
        "POWER_ON_INIT",         "SERVICE_CALIBRATION", "WAIT_FOR_VALID_INPUTS",
        "HIGH_DOWNFORCE",        "REDUCED_DRAG",        "INTERMEDIATE_VARIABLE_AERO",
        "INHIBITED",             "COMMANDED_FALLBACK",
    };
    return ((unsigned)s < AERO_STATE_COUNT) ? names[s] : "?";
}

const char *aero_reason_name(aero_reason_t r)
{
    static const char *const names[AERO_REASON_COUNT] = {
        "NONE",
        "CONFIG_INCOMPLETE",
        "BOOT_COMPLETE",
        "SERVICE_ENTER",
        "SERVICE_ACTIVE",
        "SERVICE_EXIT",
        "WAITING_FOR_INPUTS",
        "INPUTS_STABILIZING",
        "INPUTS_VALID",
        "REQUIRED_INPUT_INVALID",
        "MANUAL_INHIBIT",
        "INHIBIT_CLEARED",
        "ACTUATOR_SUPPLY_LOST",
        "CAN_BUS_SILENT",
        "LATCHED_FAULT",
        "FAULT_RECOVERING",
        "FAULT_RECOVERED",
        "STABILITY_BRAKING",
        "STABILITY_STEERING",
        "STABILITY_YAW",
        "STABILITY_LAT_ACCEL",
        "LD_DEBOUNCING",
        "LD_ELIGIBLE",
        "LD_CONDITIONS_LOST",
        "HDF_DEFAULT",
    };
    return ((unsigned)r < AERO_REASON_COUNT) ? names[r] : "?";
}

const char *aero_pos_status_name(aero_pos_status_t p)
{
    static const char *const names[] = {
        "NOT_COMMANDED", "REQUESTED", "IN_MOTION", "CONFIRMED", "UNCONFIRMED", "UNKNOWN",
    };
    return ((unsigned)p < sizeof(names) / sizeof(names[0])) ? names[p] : "?";
}

size_t aero_sizeof_config(void) { return sizeof(aero_config_t); }
size_t aero_sizeof_inputs(void) { return sizeof(aero_inputs_t); }
size_t aero_sizeof_output(void) { return sizeof(aero_output_t); }
size_t aero_sizeof_supervisor(void) { return sizeof(aero_supervisor_t); }
