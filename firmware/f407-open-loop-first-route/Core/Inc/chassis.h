#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>
/* Same X layout and equal-weight initial settings as the junior's Chassis.c. */
#define CHASSIS_WHEEL_PATTERN_X 1U
/* Body +X points right and +Y points forward. On 2026-10-03, after the
 * wheel installation was corrected, reference vx=-50 produced physical +X.
 * Calibrate only X at the reference-mixer boundary; do not couple W to it. */
#define CHASSIS_X_INPUT_POLARITY (-1.0f)
typedef enum {
    CHASSIS_FORWARD, CHASSIS_BACKWARD, CHASSIS_LEFT, CHASSIS_RIGHT,
    CHASSIS_ROTATE_LEFT, CHASSIS_ROTATE_RIGHT
} ChassisMotion;
/* Target updates only; the ACK-gated executor owns sending and enabling. */
void Chassis_Init(void);
uint8_t Chassis_InverseMotionControl(float v_x, float v_y, float w);
uint8_t Chassis_SetSpeed(float vx, float vy, float theta_degrees);
uint8_t Chassis_PrepareMotion(ChassisMotion motion, uint16_t speed_rpm);
#endif
