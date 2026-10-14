#include "chassis_heading.h"
#include "imu_ms901m.h"
#include "pid_position.h"
#include "main.h"
#include <math.h>

volatile ChassisHeadingDiagnostics chassis_heading_diag;
volatile ChassisHeadingTraceEntry chassis_heading_trace[CHASSIS_HEADING_TRACE_CAPACITY];
volatile uint32_t chassis_heading_trace_count;
static PidPosition pid;
static uint32_t checksum_base, length_base, uart_base;
static uint32_t last_trace_ms;
static uint8_t control_started;

static int32_t NormalizeTenths(int32_t angle)
{
    while (angle > 1800) angle -= 3600;
    while (angle < -1800) angle += 3600;
    return angle;
}

static void ConfigureHoldPid(void)
{
    const PidPosition config = {
        .kp = CHASSIS_HEADING_KP, .ki = CHASSIS_HEADING_KI,
        .kd = CHASSIS_HEADING_KD, .deadband = CHASSIS_HEADING_DEADBAND_DEG,
        .out_limit = CHASSIS_HEADING_LIMIT_RPM, .integral_limit = 10.0f
    };
    pid = config;
}

static void ConfigureTurnPid(void)
{
    const PidPosition config = {
        .kp = CHASSIS_TURN_KP, .ki = CHASSIS_TURN_KI,
        .kd = CHASSIS_TURN_KD, .deadband = CHASSIS_TURN_DEADBAND_DEG,
        .out_limit = CHASSIS_TURN_LIMIT_RPM, .integral_limit = 0.0f
    };
    pid = config;
}

static void RecordTrace(uint32_t now)
{
#if (CHASSIS_HEADING_LOW_SPEED_TEST || CHASSIS_HEADING_TURN_TEST) && !IMU_ONLY_TEST_ACTIVE
    if (chassis_heading_trace_count < CHASSIS_HEADING_TRACE_CAPACITY &&
        (chassis_heading_trace_count == 0U ||
         now - last_trace_ms >= CHASSIS_HEADING_TRACE_PERIOD_MS)) {
        const ChassisHeadingTraceEntry entry = {
            .at_ms = now,
            .yaw_tenths = chassis_heading_diag.yaw_tenths,
            .error_tenths = chassis_heading_diag.error_tenths,
            .correction_rpm = chassis_heading_diag.correction_rpm,
            .p_out = chassis_heading_diag.p_out,
            .d_out = chassis_heading_diag.d_out,
            .age_ms = chassis_heading_diag.age_ms,
            .mode = chassis_heading_diag.mode
        };
        chassis_heading_trace[chassis_heading_trace_count] = entry;
        chassis_heading_trace_count++;
        last_trace_ms = now;
    }
#else
    (void)now;
#endif
}

void ChassisHeading_EndMotion(void)
{
    PidPosition_Reset(&pid);
    control_started = 0U;
    chassis_heading_diag.correction_rpm = 0.0f;
    chassis_heading_diag.p_out = 0.0f;
    chassis_heading_diag.i_out = 0.0f;
    chassis_heading_diag.d_out = 0.0f;
}
static void Fault(uint32_t reason)
{
    if (chassis_heading_diag.state != HEADING_FAULT) {
        chassis_heading_diag.fault = reason;
        chassis_heading_diag.state = HEADING_FAULT;
    }
    ChassisHeading_EndMotion();
}
void ChassisHeading_Init(void)
{
    const ChassisHeadingDiagnostics initial = {0};
    chassis_heading_diag = initial;
    chassis_heading_diag.diagnostic_only = IMU_ONLY_TEST_ACTIVE ? 1U : 0U;
    chassis_heading_diag.fault = HEADING_NOT_READY;
    chassis_heading_diag.age_ms = UINT32_MAX;
    chassis_heading_diag.mode = HEADING_MODE_HOLD;
    ConfigureHoldPid();
    chassis_heading_trace_count = 0U;
    last_trace_ms = 0U;
    checksum_base = length_base = uart_base = 0U;
    ChassisHeading_EndMotion();
}
uint8_t ChassisHeading_ArmCurrent(void)
{
    ImuMs901mSnapshot data = imu_ms901m_diag.data;
    if (chassis_heading_diag.state == HEADING_FAULT) return 0U;
    if (chassis_heading_diag.target_locked) return ChassisHeading_IsHealthy();
    if (!imu_ms901m_diag.zero_ready || !ImuMs901m_IsFresh(&data, HAL_GetTick()) ||
        HAL_GetTick() - data.last_angle_ms > CHASSIS_HEADING_FRESH_MS ||
        data.yaw_tenths < -1800 || data.yaw_tenths > 1800) return 0U;
    chassis_heading_diag.target_yaw_tenths = data.yaw_tenths;
    chassis_heading_diag.yaw_tenths = data.yaw_tenths;
    chassis_heading_diag.error_tenths = 0;
    chassis_heading_diag.age_ms = HAL_GetTick() - data.last_angle_ms;
    checksum_base = data.checksum_errors;
    length_base = data.length_errors;
    uart_base = imu_ms901m_diag.uart_errors;
    chassis_heading_diag.receive_errors = 0U;
    chassis_heading_diag.target_locked = 1U;
    chassis_heading_diag.state = HEADING_ACTIVE;
    chassis_heading_diag.fault = HEADING_OK;
    chassis_heading_diag.mode = HEADING_MODE_HOLD;
    chassis_heading_diag.turn_complete = 0U;
    chassis_heading_diag.turn_started_ms = 0U;
    chassis_heading_diag.settle_started_ms = 0U;
    chassis_heading_diag.commanded_delta_tenths = 0;
    ConfigureHoldPid();
    ChassisHeading_EndMotion();
    return 1U;
}
uint8_t ChassisHeading_IsHealthy(void)
{
    const uint32_t now = HAL_GetTick();
    ImuMs901mSnapshot data = imu_ms901m_diag.data;
    uint32_t a, b, c;
    chassis_heading_diag.yaw_tenths = data.yaw_tenths;
    chassis_heading_diag.age_ms = data.angle_frames ? now - data.last_angle_ms : UINT32_MAX;
    if (chassis_heading_diag.state != HEADING_ACTIVE) return 0U;
    if (!ImuMs901m_IsFresh(&data, now) || now - data.last_angle_ms > CHASSIS_HEADING_FRESH_MS) {
        Fault(HEADING_STALE); return 0U;
    }
    if (data.yaw_tenths < -1800 || data.yaw_tenths > 1800) {
        Fault(HEADING_PARAMETER); return 0U;
    }
    a = data.checksum_errors - checksum_base;
    b = data.length_errors - length_base;
    c = imu_ms901m_diag.uart_errors - uart_base;
    chassis_heading_diag.receive_errors =
        (a >= CHASSIS_HEADING_ERROR_LIMIT || b >= CHASSIS_HEADING_ERROR_LIMIT ||
         c >= CHASSIS_HEADING_ERROR_LIMIT) ? CHASSIS_HEADING_ERROR_LIMIT : a + b + c;
    if (chassis_heading_diag.receive_errors >= CHASSIS_HEADING_ERROR_LIMIT) {
        Fault(HEADING_RECEIVE_ERRORS); return 0U;
    }
    chassis_heading_diag.error_tenths = ImuMs901m_AngleDifference(
        chassis_heading_diag.target_yaw_tenths, data.yaw_tenths);
#if CHASSIS_HEADING_LOW_SPEED_TEST && !IMU_ONLY_TEST_ACTIVE
    /* Dynamic HOLD was accepted by physical observation up to roughly 30 deg;
     * the estimate was not instrumented. Keep a 35 deg emergency guard only
     * in HOLD, and never reinterpret it as a measured performance limit. */
    if (chassis_heading_diag.mode == HEADING_MODE_HOLD &&
        (chassis_heading_diag.error_tenths > CHASSIS_HEADING_HOLD_ERROR_LIMIT_TENTHS ||
         chassis_heading_diag.error_tenths < -CHASSIS_HEADING_HOLD_ERROR_LIMIT_TENTHS)) {
        Fault(HEADING_EXCESS_ERROR); return 0U;
    }
#endif
    if (control_started && now - chassis_heading_diag.last_control_ms > CHASSIS_HEADING_MAX_GAP_MS) {
        Fault(HEADING_SCHEDULE); return 0U;
    }
    return 1U;
}

uint8_t ChassisHeading_SetTarget(float target_degrees)
{
    int32_t target;
    if (chassis_heading_diag.state != HEADING_ACTIVE ||
        chassis_heading_diag.mode != HEADING_MODE_HOLD || !isfinite(target_degrees) ||
        target_degrees < -180.0f || target_degrees > 180.0f) return 0U;
    target = (int32_t)lroundf(target_degrees * 10.0f);
    chassis_heading_diag.target_yaw_tenths = NormalizeTenths(target);
    chassis_heading_diag.error_tenths = ImuMs901m_AngleDifference(
        chassis_heading_diag.target_yaw_tenths, imu_ms901m_diag.data.yaw_tenths);
    return 1U;
}

uint8_t ChassisHeading_AddTarget(float delta_degrees)
{
    int32_t delta;
    if (chassis_heading_diag.state != HEADING_ACTIVE ||
        chassis_heading_diag.mode != HEADING_MODE_HOLD || !isfinite(delta_degrees) ||
        delta_degrees < -180.0f || delta_degrees > 180.0f) return 0U;
    delta = (int32_t)lroundf(delta_degrees * 10.0f);
    chassis_heading_diag.target_yaw_tenths = NormalizeTenths(
        chassis_heading_diag.target_yaw_tenths + delta);
    chassis_heading_diag.error_tenths = ImuMs901m_AngleDifference(
        chassis_heading_diag.target_yaw_tenths, imu_ms901m_diag.data.yaw_tenths);
    return 1U;
}

float ChassisHeading_GetErrorDegrees(void)
{
    return (float)chassis_heading_diag.error_tenths / 10.0f;
}

uint8_t ChassisHeading_StartTurn(float delta_degrees)
{
    const uint32_t now = HAL_GetTick();
    if (!isfinite(delta_degrees) || fabsf(delta_degrees) > CHASSIS_TURN_MAX_DELTA_DEG ||
        fabsf(delta_degrees) <= CHASSIS_TURN_DEADBAND_DEG ||
        chassis_heading_diag.mode != HEADING_MODE_HOLD ||
        !ChassisHeading_IsHealthy() || !ChassisHeading_AddTarget(delta_degrees))
        return 0U;
    ConfigureTurnPid();
    PidPosition_Reset(&pid);
    control_started = 0U;
    chassis_heading_diag.mode = HEADING_MODE_TURN;
    chassis_heading_diag.turn_complete = 0U;
    chassis_heading_diag.turn_started_ms = now;
    chassis_heading_diag.settle_started_ms = 0U;
    chassis_heading_diag.commanded_delta_tenths =
        (int32_t)lroundf(delta_degrees * 10.0f);
    chassis_heading_diag.correction_rpm = 0.0f;
    chassis_heading_diag.p_out = 0.0f;
    chassis_heading_diag.i_out = 0.0f;
    chassis_heading_diag.d_out = 0.0f;
    return 1U;
}

uint8_t ChassisHeading_IsTurnComplete(void)
{
    return (uint8_t)(chassis_heading_diag.state == HEADING_ACTIVE &&
                     chassis_heading_diag.turn_complete != 0U);
}

uint8_t ChassisHeading_Compute(float *correction)
{
    uint32_t now = HAL_GetTick();
    if (!correction) { Fault(HEADING_PARAMETER); return 0U; }
    *correction = 0.0f;
    if (!ChassisHeading_IsHealthy()) return 0U;
    if ((chassis_heading_diag.mode == HEADING_MODE_TURN ||
         chassis_heading_diag.mode == HEADING_MODE_SETTLE) &&
        now - chassis_heading_diag.turn_started_ms >= CHASSIS_TURN_TIMEOUT_MS) {
        Fault(HEADING_TURN_TIMEOUT);
        return 0U;
    }
#if IMU_ONLY_TEST_ACTIVE
    if (!control_started || now - chassis_heading_diag.last_control_ms >= CHASSIS_HEADING_PERIOD_MS)
#endif
    {
        float requested, change;
        float error = ChassisHeading_GetErrorDegrees();
        uint8_t turning = (uint8_t)(chassis_heading_diag.mode == HEADING_MODE_TURN ||
                                    chassis_heading_diag.mode == HEADING_MODE_SETTLE);
        if (turning) {
            if (fabsf(error) <= CHASSIS_TURN_SETTLE_TOLERANCE_DEG) {
                if (chassis_heading_diag.mode == HEADING_MODE_TURN) {
                    chassis_heading_diag.mode = HEADING_MODE_SETTLE;
                    chassis_heading_diag.settle_started_ms = now;
                } else if (now - chassis_heading_diag.settle_started_ms >=
                           CHASSIS_TURN_SETTLE_MS) {
                    chassis_heading_diag.mode = HEADING_MODE_HOLD;
                    chassis_heading_diag.turn_complete = 1U;
                    chassis_heading_diag.correction_rpm = 0.0f;
                    chassis_heading_diag.p_out = 0.0f;
                    chassis_heading_diag.i_out = 0.0f;
                    chassis_heading_diag.d_out = 0.0f;
                    chassis_heading_diag.last_control_ms = now;
                    chassis_heading_diag.control_updates++;
                    ConfigureHoldPid();
                    PidPosition_Reset(&pid);
                    control_started = 0U;
                    RecordTrace(now);
                    return 1U;
                }
            } else if (chassis_heading_diag.mode == HEADING_MODE_SETTLE) {
                chassis_heading_diag.mode = HEADING_MODE_TURN;
                chassis_heading_diag.settle_started_ms = 0U;
            }
        }
        /* Angle wrap must not inject a 360-degree derivative impulse. */
        if (pid.primed) {
            if (error - pid.error > 180.0f) pid.error += 360.0f;
            else if (error - pid.error < -180.0f) pid.error -= 360.0f;
        }
        if (!PidPosition_Calculate(&pid, error)) {
            Fault(HEADING_PARAMETER); return 0U;
        }
        /* Preserve target-current PID semantics, then adapt its output once
         * to the verified IMU/mixer polarity of this physical chassis. */
        requested = CHASSIS_HEADING_CORRECTION_POLARITY * pid.out;
        if (turning) {
            change = requested - chassis_heading_diag.correction_rpm;
            if (change > CHASSIS_TURN_SLEW_RPM_PER_UPDATE)
                change = CHASSIS_TURN_SLEW_RPM_PER_UPDATE;
            else if (change < -CHASSIS_TURN_SLEW_RPM_PER_UPDATE)
                change = -CHASSIS_TURN_SLEW_RPM_PER_UPDATE;
            chassis_heading_diag.correction_rpm += change;
        } else chassis_heading_diag.correction_rpm = requested;
        chassis_heading_diag.p_out = CHASSIS_HEADING_CORRECTION_POLARITY * pid.p_out;
        chassis_heading_diag.i_out = CHASSIS_HEADING_CORRECTION_POLARITY * pid.i_out;
        chassis_heading_diag.d_out = CHASSIS_HEADING_CORRECTION_POLARITY * pid.d_out;
        chassis_heading_diag.last_control_ms = now;
        chassis_heading_diag.control_updates++;
        control_started = 1U;
        RecordTrace(now);
    }
    *correction = chassis_heading_diag.correction_rpm;
    return 1U;
}
float ChassisHeading_DirectionDeltaDegrees(void)
{
    return (float)ImuMs901m_AngleDifference(imu_ms901m_diag.data.yaw_tenths,
                  chassis_heading_diag.target_yaw_tenths) / 10.0f;
}
void ChassisHeading_Service(void)
{
#if IMU_ONLY_TEST_ACTIVE
    float unused;
    if (chassis_heading_diag.state == HEADING_WAITING) {
        (void)ChassisHeading_IsHealthy(); /* Keep waiting-state observation truthful. */
        (void)ChassisHeading_ArmCurrent();
    }
    if (chassis_heading_diag.state == HEADING_ACTIVE) (void)ChassisHeading_Compute(&unused);
#else
    if (chassis_heading_diag.state == HEADING_ACTIVE) (void)ChassisHeading_IsHealthy();
#endif
}
