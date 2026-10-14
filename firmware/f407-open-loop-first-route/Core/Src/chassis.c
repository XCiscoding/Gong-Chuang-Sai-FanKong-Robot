#include "chassis.h"
#include "chassis_motor_map.h"
#include "step_motor.h"
#include "chassis_heading.h"
#include <math.h>

/* Port of RC控制代码/CarBody/Chassis.c: Chassis_InverseMotionControl.
 * Preserve its X-layout mixer, wheel ordering and rounded signed RPM.
 * Adapt body X polarity at the mixer input using the measured wheel setup.
 * Load compensation remains off, as in the reference's initial settings. */
void Chassis_Init(void) { StepMotor_ClearTargets(); }

uint8_t Chassis_InverseMotionControl(float v_x, float v_y, float w)
{
    const float SignX = CHASSIS_WHEEL_PATTERN_X ? -1.0f : 1.0f;
    const float reference_vx = CHASSIS_X_INPUT_POLARITY * v_x;
    const float values[4] = {
        roundf(SignX * reference_vx + v_y - SignX * w), /* left front */
        roundf(SignX * reference_vx - v_y - SignX * w), /* right front */
        roundf(-SignX * reference_vx + v_y - SignX * w), /* left rear */
        roundf(-SignX * reference_vx - v_y - SignX * w) /* right rear */
    };
    const uint8_t ids[4] = {
        CHASSIS_MOTOR_LEFT_FRONT_ID, CHASSIS_MOTOR_RIGHT_FRONT_ID,
        CHASSIS_MOTOR_LEFT_REAR_ID, CHASSIS_MOTOR_RIGHT_REAR_ID
    };
    uint8_t i;
    for (i = 0U; i < 4U; i++) {
        if (!isfinite(values[i]) || values[i] < -3000.0f || values[i] > 3000.0f) {
            StepMotor_ClearTargets();
            return 0U;
        }
    }
    for (i = 0U; i < 4U; i++)
        if (!StepMotor_SetTargetSpeed(ids[i], (int32_t)values[i])) return 0U;
    return 1U;
}
/* Junior Chassis_SetSpeed: route-frame translation -> body frame,
 * then position-form yaw correction -> existing inverse mixer. */
uint8_t Chassis_SetSpeed(float vx, float vy, float theta)
{
    float w, vx_body, vy_body, sine, cosine;
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(theta) ||
        !ChassisHeading_Compute(&w)) {
        StepMotor_ClearTargets(); return 0U;
    }
    theta *= 0.01745329252f;
    cosine = cosf(theta);
    sine = sinf(theta);
    vx_body = vx * cosine + vy * sine;
    vy_body = -vx * sine + vy * cosine;
    return Chassis_InverseMotionControl(vx_body, vy_body, w);
}
uint8_t Chassis_PrepareMotion(ChassisMotion motion, uint16_t speed_rpm)
{
    const float speed = (float)speed_rpm;
    switch (motion) {
    case CHASSIS_FORWARD: return Chassis_InverseMotionControl(0.0f, speed, 0.0f);
    case CHASSIS_BACKWARD: return Chassis_InverseMotionControl(0.0f, -speed, 0.0f);
    case CHASSIS_LEFT: return Chassis_InverseMotionControl(-speed, 0.0f, 0.0f);
    case CHASSIS_RIGHT: return Chassis_InverseMotionControl(speed, 0.0f, 0.0f);
    case CHASSIS_ROTATE_LEFT: return Chassis_InverseMotionControl(0.0f, 0.0f, speed);
    case CHASSIS_ROTATE_RIGHT: return Chassis_InverseMotionControl(0.0f, 0.0f, -speed);
    default: StepMotor_ClearTargets(); return 0U;
    }
}
