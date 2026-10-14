#ifndef CHASSIS_PATH_H
#define CHASSIS_PATH_H
#include <stdint.h>
#include "chassis_heading.h"

/* Current-car commissioning values. Counts retain junior's 2 ms unit.
 * +Y is calibrated from the measured 105 cm run. The -X hold is a
 * conservative first compensation for the observed short lateral run and
 * remains independent until the next measured distance is available. */
#define CHASSIS_FIRST_ROUTE_SPEED_RPM          100.0f
#define CHASSIS_FIRST_ROUTE_Y_HOLD_COUNT       525.0f
#define CHASSIS_FIRST_ROUTE_X_HOLD_COUNT       575.0f
#define CHASSIS_FIRST_ROUTE_BUMP_SPEED_RPM     210.0f
#define CHASSIS_FIRST_ROUTE_BUMP_HOLD_COUNT    460.0f
#define CHASSIS_FIRST_ROUTE_RAMP_COUNT         250.0f
#define CHASSIS_FIRST_ROUTE_BUMP_RAMP_COUNT     80.0f

/* This build selects the complete three-segment first route. */
#ifndef CHASSIS_ROUTE_THIRD_ONLY
#define CHASSIS_ROUTE_THIRD_ONLY 0
#endif
#if CHASSIS_ROUTE_THIRD_ONLY != 0 && CHASSIS_ROUTE_THIRD_ONLY != 1
#error "Invalid CHASSIS_ROUTE_THIRD_ONLY"
#endif
#define CHASSIS_THIRD_ONLY_SPEED_RPM  180.0f
#define CHASSIS_THIRD_ONLY_HOLD_COUNT 450.0f
#define CHASSIS_THIRD_ONLY_RAMP_COUNT  80.0f

/* TURN commissioning approach: 200 ms rise + 600 ms hold + 200 ms fall.
 * MotorTest then performs zero-speed/STOP/disable and waits 2 s before TURN. */
#define CHASSIS_TURN_TEST_APPROACH_SPEED_RPM   100.0f
#define CHASSIS_TURN_TEST_APPROACH_HOLD_COUNT  300.0f
#define CHASSIS_TURN_TEST_APPROACH_RAMP_COUNT  100.0f

/* Junior Chassis_Path.h fields and 2 ms count units. Body +X is right,
 * +Y is forward. This commissioning route stops between its points. */
typedef struct {
    float vx, vy, Delta_Angle, t, K;
} Chassis_PathPoint;
typedef struct {
    float End_K;
    uint8_t Path_Size;
    const Chassis_PathPoint *Path;
} Chassis_Path;

extern const Chassis_Path Chassis_Path_FirstRoute;
extern const Chassis_Path Chassis_Path_ThirdOnly;
extern const Chassis_Path Chassis_Path_HeadingTest;
extern const Chassis_Path Chassis_Path_TurnTest;
#if CHASSIS_HEADING_TURN_TEST
#define CHASSIS_PATH_ACTIVE_ROUTE Chassis_Path_TurnTest
#elif CHASSIS_HEADING_LOW_SPEED_TEST
#define CHASSIS_PATH_ACTIVE_ROUTE Chassis_Path_HeadingTest
#elif CHASSIS_ROUTE_THIRD_ONLY
#define CHASSIS_PATH_ACTIVE_ROUTE Chassis_Path_ThirdOnly
#else
#define CHASSIS_PATH_ACTIVE_ROUTE Chassis_Path_FirstRoute
#endif
uint32_t Chassis_PathPointDurationMs(const Chassis_Path *path, uint8_t index);
uint8_t Chassis_PathSamplePoint(const Chassis_Path *path, uint8_t index,
                               uint32_t elapsed_ms, float *vx, float *vy);
uint8_t Chassis_PathPreparePoint(uint8_t index, uint32_t elapsed_ms);
uint8_t Chassis_PathPreparePointOpenLoop(uint8_t index, uint32_t elapsed_ms);
uint8_t Chassis_PathPointIsTurn(uint8_t index);
uint8_t Chassis_PathPointComplete(uint8_t index);
void Chassis_PathResetRuntime(void);
#endif
