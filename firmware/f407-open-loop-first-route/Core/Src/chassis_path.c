#include "chassis_path.h"
#include "chassis.h"
#include "step_motor.h"
#include "chassis_heading.h"
#include <math.h>

/* Junior baseline a513d96: HomeToQR order and Chassis_SINAccel profile.
 * Adapt speed/hold/ramp for the current car after the 2026-10-03 feedback.
 * Apply the measured X polarity in Chassis. Point distances need calibration. */
static const Chassis_PathPoint first_points[] = {
    {0.0f, CHASSIS_FIRST_ROUTE_SPEED_RPM, 0.0f,
        CHASSIS_FIRST_ROUTE_Y_HOLD_COUNT, CHASSIS_FIRST_ROUTE_RAMP_COUNT},
    {-CHASSIS_FIRST_ROUTE_SPEED_RPM, 0.0f, 0.0f,
        CHASSIS_FIRST_ROUTE_X_HOLD_COUNT, CHASSIS_FIRST_ROUTE_RAMP_COUNT},
    {0.0f, CHASSIS_FIRST_ROUTE_BUMP_SPEED_RPM, 0.0f,
        CHASSIS_FIRST_ROUTE_BUMP_HOLD_COUNT, CHASSIS_FIRST_ROUTE_BUMP_RAMP_COUNT}
};
const Chassis_Path Chassis_Path_FirstRoute = {CHASSIS_FIRST_ROUTE_BUMP_RAMP_COUNT, 3U, first_points};

/* Start the car physically at segment 3. No approach legs, no repeat. */
static const Chassis_PathPoint third_only_points[] = {
    {0.0f, CHASSIS_THIRD_ONLY_SPEED_RPM, 0.0f,
        CHASSIS_THIRD_ONLY_HOLD_COUNT, CHASSIS_THIRD_ONLY_RAMP_COUNT}
};
const Chassis_Path Chassis_Path_ThirdOnly = {
    CHASSIS_THIRD_ONLY_RAMP_COUNT, 1U, third_only_points
};

/* Separate trial path; all historic route fields remain unchanged.
 * 300 ms rise + 2400 ms hold + 300 ms fall = 3000 ms. */
static const Chassis_PathPoint heading_test_points[] = {
    {0.0f, 100.0f, 0.0f, 1200.0f, 150.0f}
};
const Chassis_Path Chassis_Path_HeadingTest = {150.0f, 1U, heading_test_points};

/* Staged active-turn trial:
 * 1) short forward leg under the frozen HOLD controller;
 * 2) MotorTest performs zero-speed, sync, STOP, disable and a 2 s gap;
 * 3) relative +30 deg TURN with vx/vy exactly zero.
 * The turn profile fields only provide a 5 s outer safety window; TURN owns
 * its own 4 s timeout. */
static const Chassis_PathPoint turn_test_points[] = {
    {0.0f, CHASSIS_TURN_TEST_APPROACH_SPEED_RPM, 0.0f,
        CHASSIS_TURN_TEST_APPROACH_HOLD_COUNT,
        CHASSIS_TURN_TEST_APPROACH_RAMP_COUNT},
    {0.0f, 0.0f, 30.0f, 2000.0f, 250.0f}
};
const Chassis_Path Chassis_Path_TurnTest = {250.0f, 2U, turn_test_points};

#define CHASSIS_PATH_NO_ACTIVE_TURN 0xFFU
static uint8_t active_turn_index = CHASSIS_PATH_NO_ACTIVE_TURN;

static uint8_t Valid(const Chassis_Path *path, uint8_t index)
{
    const Chassis_PathPoint *p;
    float end;
    uint8_t translation, turn;
    if (!path || !path->Path || !path->Path_Size || index >= path->Path_Size)
        return 0U;
    p = &path->Path[index];
    end = (index + 1U == path->Path_Size) ? path->End_K : p->K;
    translation = (uint8_t)(p->Delta_Angle == 0.0f);
    turn = (uint8_t)(isfinite(p->Delta_Angle) &&
                     fabsf(p->vx) < 0.001f && fabsf(p->vy) < 0.001f &&
                     fabsf(p->Delta_Angle) > CHASSIS_TURN_DEADBAND_DEG &&
                     fabsf(p->Delta_Angle) <= CHASSIS_TURN_MAX_DELTA_DEG);
    return isfinite(p->vx) && isfinite(p->vy) && (translation || turn) &&
           fabsf(p->vx) + fabsf(p->vy) <= 3000.0f &&
           isfinite(p->t) && p->t >= 0.0f && p->t <= 10000.0f &&
           floorf(p->t) == p->t &&
           isfinite(p->K) && p->K > 0.0f && p->K <= 10000.0f &&
           floorf(p->K) == p->K &&
           isfinite(end) && end > 0.0f && end <= 10000.0f && floorf(end) == end;
}

uint32_t Chassis_PathPointDurationMs(const Chassis_Path *path, uint8_t index)
{
    const Chassis_PathPoint *p;
    if (!Valid(path, index)) return 0U;
    p = &path->Path[index];
    return 2U * (uint32_t)(p->K + p->t +
           ((index + 1U == path->Path_Size) ? path->End_K : p->K));
}

uint8_t Chassis_PathSamplePoint(const Chassis_Path *path, uint8_t index,
                               uint32_t elapsed_ms, float *vx, float *vy)
{
    const Chassis_PathPoint *p;
    uint32_t step, ramp, hold, end;
    float gain;
    if (!vx || !vy) return 0U;
    *vx = 0.0f; *vy = 0.0f;
    if (!Valid(path, index)) return 0U;
    p = &path->Path[index];
    step = elapsed_ms / 2U;
    ramp = (uint32_t)p->K;
    hold = (uint32_t)p->t;
    end = (uint32_t)((index + 1U == path->Path_Size) ? path->End_K : p->K);
    /* Same 0.5 - 0.5*cos(PI*i/K) as junior Chassis_SINAccel.
     * ACK serialization samples the profile at elapsed wall time; it does
     * not promise the reference's nominal 2 ms UART update cadence. */
    if (step < ramp)
        gain = 0.5f - 0.5f * cosf(3.141592654f * (float)step / (float)ramp);
    else if (step < ramp + hold)
        gain = 1.0f;
    else if (step < ramp + hold + end)
        gain = 0.5f + 0.5f * cosf(3.141592654f *
                               (float)(step - ramp - hold) / (float)end);
    else gain = 0.0f;
    *vx = p->vx * gain; *vy = p->vy * gain;
    return 1U;
}

/* Open-loop route adapter. It deliberately accepts translation points only,
 * so a future route turn cannot silently run without heading feedback. */
uint8_t Chassis_PathPreparePointOpenLoop(uint8_t index, uint32_t elapsed_ms)
{
    const Chassis_PathPoint *point;
    float vx, vy;

    if (!Chassis_PathSamplePoint(&CHASSIS_PATH_ACTIVE_ROUTE, index,
                                 elapsed_ms, &vx, &vy)) {
        StepMotor_ClearTargets();
        return 0U;
    }
    point = &CHASSIS_PATH_ACTIVE_ROUTE.Path[index];
    if (point->Delta_Angle != 0.0f) {
        StepMotor_ClearTargets();
        return 0U;
    }
    return Chassis_InverseMotionControl(vx, vy, 0.0f);
}

uint8_t Chassis_PathPreparePoint(uint8_t index, uint32_t elapsed_ms)
{
    const Chassis_PathPoint *p;
    float vx, vy, w;
    if (!Chassis_PathSamplePoint(&CHASSIS_PATH_ACTIVE_ROUTE, index, elapsed_ms, &vx, &vy)) {
        StepMotor_ClearTargets();
        return 0U;
    }
    p = &CHASSIS_PATH_ACTIVE_ROUTE.Path[index];
    if (p->Delta_Angle != 0.0f) {
        if (active_turn_index == CHASSIS_PATH_NO_ACTIVE_TURN) {
            if (!ChassisHeading_StartTurn(p->Delta_Angle)) {
                StepMotor_ClearTargets();
                return 0U;
            }
            active_turn_index = index;
        } else if (active_turn_index != index) {
            StepMotor_ClearTargets();
            return 0U;
        }
        if (!ChassisHeading_Compute(&w)) {
            StepMotor_ClearTargets();
            return 0U;
        }
        return Chassis_InverseMotionControl(0.0f, 0.0f, w);
    }
    return Chassis_SetSpeed(vx, vy, ChassisHeading_DirectionDeltaDegrees());
}

uint8_t Chassis_PathPointIsTurn(uint8_t index)
{
    return Valid(&CHASSIS_PATH_ACTIVE_ROUTE, index) &&
           CHASSIS_PATH_ACTIVE_ROUTE.Path[index].Delta_Angle != 0.0f;
}

uint8_t Chassis_PathPointComplete(uint8_t index)
{
    return Chassis_PathPointIsTurn(index) && ChassisHeading_IsTurnComplete();
}

void Chassis_PathResetRuntime(void)
{
    active_turn_index = CHASSIS_PATH_NO_ACTIVE_TURN;
}
