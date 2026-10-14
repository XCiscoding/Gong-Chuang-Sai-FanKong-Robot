#ifndef CHASSIS_HEADING_H
#define CHASSIS_HEADING_H
#include <stdint.h>
/* Candidates, NOT physically calibrated. Output is mixer rotation RPM. */
#define CHASSIS_HEADING_KP             1.0f
#define CHASSIS_HEADING_KI             0.0f
#define CHASSIS_HEADING_KD             0.0f
#define CHASSIS_HEADING_DEADBAND_DEG   0.55f
/* Current MS901M installation reports clockwise yaw as negative, while the
 * inherited mixer sends positive w as wire direction 0 on all four motors,
 * which is clockwise on this chassis. Invert only at the mixer boundary. */
#define CHASSIS_HEADING_CORRECTION_POLARITY (-1.0f)
/* Staged commissioning build: a short HOLD-controlled forward leg, a full
 * stop/gap, then a relative +30 deg TURN. HOLD parameters above are frozen
 * from the passed dynamic trial. */
#ifndef CHASSIS_HEADING_TURN_TEST
#define CHASSIS_HEADING_TURN_TEST 0
#endif
#if CHASSIS_HEADING_TURN_TEST != 0 && CHASSIS_HEADING_TURN_TEST != 1
#error "Invalid CHASSIS_HEADING_TURN_TEST"
#endif
#define CHASSIS_TURN_KP                    2.0f
#define CHASSIS_TURN_KI                    0.0f
#define CHASSIS_TURN_KD                    0.5f
#define CHASSIS_TURN_DEADBAND_DEG          0.55f
#define CHASSIS_TURN_LIMIT_RPM            30.0f
#define CHASSIS_TURN_SLEW_RPM_PER_UPDATE   2.0f
#define CHASSIS_TURN_SETTLE_TOLERANCE_DEG  2.0f
#define CHASSIS_TURN_SETTLE_MS            500U
#define CHASSIS_TURN_TIMEOUT_MS          4000U
#define CHASSIS_TURN_MAX_DELTA_DEG         30.0f
/* First powered trial: flat-ground short forward leg only. */
#ifndef CHASSIS_HEADING_LOW_SPEED_TEST
#define CHASSIS_HEADING_LOW_SPEED_TEST 0
#endif
#if CHASSIS_HEADING_LOW_SPEED_TEST != 0 && CHASSIS_HEADING_LOW_SPEED_TEST != 1
#error "Invalid CHASSIS_HEADING_LOW_SPEED_TEST"
#endif
#if CHASSIS_HEADING_LOW_SPEED_TEST
#define CHASSIS_HEADING_LIMIT_RPM     15.0f
#define CHASSIS_HEADING_HOLD_ERROR_LIMIT_TENTHS 350
#else
#define CHASSIS_HEADING_LIMIT_RPM     30.0f
#endif
#define CHASSIS_HEADING_PERIOD_MS     10U
#define CHASSIS_HEADING_MAX_GAP_MS    20U
#define CHASSIS_HEADING_ERROR_LIMIT    3U
#define CHASSIS_HEADING_FRESH_MS     100U
enum { HEADING_WAITING = 0U, HEADING_ACTIVE = 1U, HEADING_FAULT = 2U };
enum { HEADING_MODE_HOLD = 0U, HEADING_MODE_TURN = 1U,
       HEADING_MODE_SETTLE = 2U };
enum { HEADING_OK = 0U, HEADING_NOT_READY = 1U, HEADING_STALE = 2U,
       HEADING_RECEIVE_ERRORS = 3U, HEADING_PARAMETER = 4U,
       HEADING_SCHEDULE = 5U, HEADING_EXCESS_ERROR = 6U,
       HEADING_TURN_TIMEOUT = 7U };
/* All fields four bytes; signed angles in tenths of degrees.
 * correction_rpm and P/I/D fields are the values applied to the mixer. */
typedef struct {
    uint32_t diagnostic_only, state, fault, target_locked;
    int32_t target_yaw_tenths, yaw_tenths, error_tenths;
    float correction_rpm;
    uint32_t control_updates, last_control_ms, age_ms, receive_errors;
    float p_out, i_out, d_out;
    uint32_t mode, turn_complete, turn_started_ms, settle_started_ms;
    int32_t commanded_delta_tenths;
} ChassisHeadingDiagnostics;
extern volatile ChassisHeadingDiagnostics chassis_heading_diag;
/* First short trial retains per-group calculations for reading after stop.
 * A calculation record does not itself prove ACK/sync or physical motion. */
#define CHASSIS_HEADING_TRACE_PERIOD_MS 20U
#define CHASSIS_HEADING_TRACE_CAPACITY 256U
typedef struct {
    uint32_t at_ms;
    int32_t yaw_tenths, error_tenths;
    float correction_rpm, p_out, d_out;
    uint32_t age_ms, mode;
} ChassisHeadingTraceEntry;
extern volatile ChassisHeadingTraceEntry chassis_heading_trace[CHASSIS_HEADING_TRACE_CAPACITY];
extern volatile uint32_t chassis_heading_trace_count;
void ChassisHeading_Init(void);
void ChassisHeading_Service(void);
uint8_t ChassisHeading_ArmCurrent(void);
uint8_t ChassisHeading_IsHealthy(void);
uint8_t ChassisHeading_Compute(float *correction_rpm);
uint8_t ChassisHeading_SetTarget(float target_degrees);
uint8_t ChassisHeading_AddTarget(float delta_degrees);
float ChassisHeading_GetErrorDegrees(void);
uint8_t ChassisHeading_StartTurn(float delta_degrees);
uint8_t ChassisHeading_IsTurnComplete(void);
float ChassisHeading_DirectionDeltaDegrees(void);
/* Keep route heading; clear PID history and rotation at a stop. */
void ChassisHeading_EndMotion(void);
#endif
