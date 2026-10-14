#ifndef PID_POSITION_H
#define PID_POSITION_H
#include <stdint.h>
/* Junior position-form semantics: Ki/Kd are per update, not per second. */
typedef struct {
    float kp, ki, kd, deadband, out_limit, integral_limit;
    float error, sum_error, p_out, i_out, d_out, out;
    uint8_t primed;
} PidPosition;
void PidPosition_Reset(PidPosition *pid);
uint8_t PidPosition_Calculate(PidPosition *pid, float wrapped_error);
#endif
