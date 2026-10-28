/*
 * aero_core.h - Active aero supervisory state machine (portable C99 core).
 *
 * PRELIMINARY / PLACEHOLDER SOFTWARE. Not validated, not vehicle-ready.
 * An HDF *command* is never treated as proof of an HDF *position*.
 *
 * Portability rules for this core (so it can move to ESP32-S3 / ESP-IDF):
 *   - no dynamic allocation, no stdio, no OS / RTOS / wall-clock calls
 *   - all time is injected as monotonic milliseconds (uint32_t)
 *   - all hardware I/O (CAN, actuators, switches) lives outside the core
 *
 * State names mirror the COPoP flowchart dated 2026-10-28.
 */
#ifndef AERO_CORE_H
#define AERO_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AERO_CORE_VERSION 1u
#define AERO_MAX_CHANNELS 4u
#define AERO_TBD_MS UINT32_MAX /* unset timing parameter */
/* Unset float parameters are NaN; see aero_config_init_tbd(). */

/* ---------------------------------------------------------------- states */
typedef enum {
    AERO_STATE_POWER_ON_INIT = 0,
    AERO_STATE_SERVICE_CALIBRATION,
    AERO_STATE_WAIT_FOR_VALID_INPUTS,
    AERO_STATE_HIGH_DOWNFORCE,
    AERO_STATE_REDUCED_DRAG,
    AERO_STATE_INTERMEDIATE_VARIABLE_AERO, /* feature-gated; unreachable in Phase 1 */
    AERO_STATE_INHIBITED,
    AERO_STATE_COMMANDED_FALLBACK,
    AERO_STATE_COUNT
} aero_state_t;

typedef enum {
    AERO_TARGET_NONE = 0, /* no position commanded (outputs not driven) */
    AERO_TARGET_HDF,
    AERO_TARGET_LD
} aero_target_t;

/* Why the supervisor is where it is. Every tick reports exactly one. */
typedef enum {
    AERO_REASON_NONE = 0,
    AERO_REASON_CONFIG_INCOMPLETE,
    AERO_REASON_BOOT_COMPLETE,
    AERO_REASON_SERVICE_ENTER,
    AERO_REASON_SERVICE_ACTIVE,
    AERO_REASON_SERVICE_EXIT,
    AERO_REASON_WAITING_FOR_INPUTS,
    AERO_REASON_INPUTS_STABILIZING,
    AERO_REASON_INPUTS_VALID,
    AERO_REASON_REQUIRED_INPUT_INVALID,
    AERO_REASON_MANUAL_INHIBIT,
    AERO_REASON_INHIBIT_CLEARED,
    AERO_REASON_ACTUATOR_SUPPLY_LOST,
    AERO_REASON_CAN_BUS_SILENT,
    AERO_REASON_LATCHED_FAULT,
    AERO_REASON_FAULT_RECOVERING,
    AERO_REASON_FAULT_RECOVERED,
    AERO_REASON_STABILITY_BRAKING,
    AERO_REASON_STABILITY_STEERING,
    AERO_REASON_STABILITY_YAW,
    AERO_REASON_STABILITY_LAT_ACCEL,
    AERO_REASON_LD_DEBOUNCING,
    AERO_REASON_LD_ELIGIBLE,
    AERO_REASON_LD_CONDITIONS_LOST,
    AERO_REASON_HDF_DEFAULT,
    AERO_REASON_COUNT
} aero_reason_t;

/* ------------------------------------------------------------- signals */
typedef enum {
    AERO_SIG_SPEED = 0,  /* vehicle speed, m/s */
    AERO_SIG_THROTTLE,   /* accelerator pedal, 0..1 */
    AERO_SIG_BRAKE,      /* brake pressure, bar */
    AERO_SIG_STEERING,   /* steering angle, rad */
    AERO_SIG_YAW_RATE,   /* rad/s */
    AERO_SIG_LAT_ACCEL,  /* m/s^2 */
    AERO_SIG_COUNT
} aero_signal_id_t;

typedef enum {
    AERO_SIGSTAT_NEVER_RECEIVED = 0,
    AERO_SIGSTAT_FRESH_VALID,
    AERO_SIGSTAT_FRESH_IMPLAUSIBLE,
    AERO_SIGSTAT_STALE,
    AERO_SIGSTAT_SOURCE_INVALID
} aero_signal_status_t;

typedef struct {
    float value;
    uint32_t timestamp_ms; /* receive time, same monotonic clock as tick */
    bool received;         /* false = never received (NOT the same as 0.0) */
    bool source_valid;     /* sender's own validity flag */
} aero_signal_sample_t;

/* -------------------------------------------------------------- channels */
typedef enum {
    AERO_CH_FRONT_LEFT = 0,
    AERO_CH_FRONT_RIGHT,
    AERO_CH_REAR_MAIN,
    AERO_CH_REAR_AUX /* only when channel_count == 4 */
} aero_channel_id_t;

typedef enum {
    AERO_POS_NOT_COMMANDED = 0,
    AERO_POS_REQUESTED,   /* command issued this tick */
    AERO_POS_IN_MOTION,   /* within motion window, not yet at target */
    AERO_POS_CONFIRMED,   /* independent output sensor within tolerance */
    AERO_POS_UNCONFIRMED, /* window expired, or no trustworthy evidence */
    AERO_POS_UNKNOWN      /* output sensor unavailable */
} aero_pos_status_t;

/* Feedback per channel, supplied by the actuator adapter (or simulator). */
typedef struct {
    float measured_output_deg; /* INDEPENDENT element/output sensor, not motor encoder */
    bool output_sensor_ok;
    bool comms_ok;             /* actuator controller responding */
} aero_channel_feedback_t;

/* ---------------------------------------------------------------- faults */
/* Bit flags. RECOVERABLE faults may auto-clear; LATCHED ones need a reset. */
#define AERO_FAULT_CAN_BUS_SILENT      (1u << 0) /* recoverable */
#define AERO_FAULT_ACTUATOR_SUPPLY     (1u << 1) /* latched */
#define AERO_FAULT_ACTUATOR_COMMS      (1u << 2) /* latched */
#define AERO_FAULT_OUTPUT_SENSOR       (1u << 3) /* latched */
#define AERO_FAULT_MOTION_TIMEOUT      (1u << 4) /* latched */
#define AERO_FAULT_RECOVERABLE_MASK    (AERO_FAULT_CAN_BUS_SILENT)
#define AERO_FAULT_LATCHING_MASK       (AERO_FAULT_ACTUATOR_SUPPLY | AERO_FAULT_ACTUATOR_COMMS | \
                                        AERO_FAULT_OUTPUT_SENSOR | AERO_FAULT_MOTION_TIMEOUT)

/* Inhibit flags: bit 0 = manual switch, bits 1.. = required signal i invalid. */
#define AERO_INHIBIT_MANUAL            (1u << 0)
#define AERO_INHIBIT_SIGNAL(sig)       (1u << (1u + (uint32_t)(sig)))

/* ---------------------------------------------------------------- config */
/*
 * Every field starts as TBD (NaN / AERO_TBD_MS). aero_init() refuses to leave
 * POWER_ON_INIT until every required field is set. Real values must come from
 * approved team decisions; simulation values live in a separate, clearly
 * labeled sim-only config file.
 */
typedef struct {
    /* signal validation */
    bool signal_required[AERO_SIG_COUNT];
    uint32_t signal_timeout_ms[AERO_SIG_COUNT];
    float signal_min[AERO_SIG_COUNT];
    float signal_max[AERO_SIG_COUNT];
    uint32_t can_bus_timeout_ms;      /* no frames at all for this long = bus silent */
    uint32_t inputs_stable_ms;        /* WAIT -> HDF stabilization window */
    uint32_t fault_recovery_stable_ms;/* recoverable fault must stay clear this long */

    /* LD eligibility (entry thresholds stricter than exit = hysteresis) */
    float ld_entry_speed_mps;
    float ld_exit_speed_mps;
    float ld_entry_throttle;
    float ld_exit_throttle;
    uint32_t ld_entry_debounce_ms;

    /* stability demand -> immediate HDF */
    float hdf_brake_bar;
    float hdf_steering_rad;
    float hdf_yaw_rate_rps;
    float hdf_lat_accel_mps2;

    /* actuator channels */
    uint8_t channel_count;            /* 3 or 4 */
    float hdf_position_deg[AERO_MAX_CHANNELS];
    float ld_position_deg[AERO_MAX_CHANNELS];
    float position_tolerance_deg;
    uint32_t motion_timeout_ms;

    bool variable_aero_enabled;       /* must stay false until approved */
} aero_config_t;

/* --------------------------------------------------------------- inputs */
typedef struct {
    aero_signal_sample_t signal[AERO_SIG_COUNT];
    uint32_t can_last_rx_ms;          /* time of last frame of any kind */
    bool can_ever_rx;

    bool manual_inhibit;              /* driver switch, read as a digital input */
    bool actuator_supply_good;        /* actuator branch power present */
    bool service_request;             /* only honored in POWER_ON_INIT */
    bool service_exit;
    bool fault_reset_request;         /* deliberate reset of latched faults */

    aero_channel_feedback_t channel[AERO_MAX_CHANNELS];
} aero_inputs_t;

/* --------------------------------------------------------------- outputs */
typedef struct {
    uint32_t timestamp_ms;
    aero_state_t state;
    aero_state_t previous_state;
    aero_reason_t reason;
    aero_target_t target;
    bool command_enabled;             /* false = adapters must not drive actuators */
    float channel_command_deg[AERO_MAX_CHANNELS];
    aero_pos_status_t channel_status[AERO_MAX_CHANNELS];
    bool output_confirmed;            /* ALL channels CONFIRMED at target */
    uint32_t inhibit_flags;
    uint32_t active_faults;           /* conditions present right now */
    uint32_t latched_faults;          /* stay set until a valid reset */
    uint8_t signal_status[AERO_SIG_COUNT]; /* aero_signal_status_t */
    uint32_t transition_count;
    uint32_t state_entered_ms;
} aero_output_t;

/* ----------------------------------------------------------- supervisor */
typedef struct {
    aero_config_t cfg;
    bool cfg_valid;
    aero_output_t out;

    uint32_t inputs_valid_since_ms;
    bool inputs_valid_timing;
    uint32_t ld_eligible_since_ms;
    bool ld_eligible_timing;
    uint32_t faults_clear_since_ms;
    bool faults_clear_timing;

    aero_target_t last_target;
    uint32_t command_change_ms;
} aero_supervisor_t;

/* ------------------------------------------------------------------ API */
void aero_config_init_tbd(aero_config_t *cfg);
bool aero_config_is_complete(const aero_config_t *cfg);

void aero_init(aero_supervisor_t *sup, const aero_config_t *cfg, uint32_t now_ms);
const aero_output_t *aero_tick(aero_supervisor_t *sup, const aero_inputs_t *in, uint32_t now_ms);

const char *aero_state_name(aero_state_t s);
const char *aero_reason_name(aero_reason_t r);
const char *aero_pos_status_name(aero_pos_status_t p);

/* Struct sizes, so host-side bindings can verify their layout matches. */
size_t aero_sizeof_config(void);
size_t aero_sizeof_inputs(void);
size_t aero_sizeof_output(void);
size_t aero_sizeof_supervisor(void);

#ifdef __cplusplus
}
#endif

#endif /* AERO_CORE_H */
