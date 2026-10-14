#ifndef CHASSIS_MOTOR_MAP_H
#define CHASSIS_MOTOR_MAP_H

/* Verified physical mapping, viewed from above with the vehicle nose forward.
 * The logical wheel position is deliberately independent from the bus ID. */
#define CHASSIS_MOTOR_LEFT_FRONT_ID   1U
#define CHASSIS_MOTOR_RIGHT_FRONT_ID  4U
#define CHASSIS_MOTOR_RIGHT_REAR_ID   3U
#define CHASSIS_MOTOR_LEFT_REAR_ID    2U

/* Handoff polarity; four-wheel physical forward is still to be observed. */
#define CHASSIS_MOTOR_LEFT_FRONT_FORWARD_DIR  0U
#define CHASSIS_MOTOR_LEFT_REAR_FORWARD_DIR   0U
#define CHASSIS_MOTOR_RIGHT_REAR_FORWARD_DIR  1U
#define CHASSIS_MOTOR_RIGHT_FRONT_FORWARD_DIR 1U

#if (CHASSIS_MOTOR_LEFT_FRONT_ID == CHASSIS_MOTOR_RIGHT_FRONT_ID) || \
    (CHASSIS_MOTOR_LEFT_FRONT_ID == CHASSIS_MOTOR_RIGHT_REAR_ID) || \
    (CHASSIS_MOTOR_LEFT_FRONT_ID == CHASSIS_MOTOR_LEFT_REAR_ID) || \
    (CHASSIS_MOTOR_RIGHT_FRONT_ID == CHASSIS_MOTOR_RIGHT_REAR_ID) || \
    (CHASSIS_MOTOR_RIGHT_FRONT_ID == CHASSIS_MOTOR_LEFT_REAR_ID) || \
    (CHASSIS_MOTOR_RIGHT_REAR_ID == CHASSIS_MOTOR_LEFT_REAR_ID)
#error "Each chassis wheel must have a unique X42S ID"
#endif

#endif /* CHASSIS_MOTOR_MAP_H */
