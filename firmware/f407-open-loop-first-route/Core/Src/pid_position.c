#include "pid_position.h"
#include <math.h>
static float Clamp(float value, float limit)
{
    return value < -limit ? -limit : value > limit ? limit : value;
}
void PidPosition_Reset(PidPosition *pid)
{
    if (!pid) return;
    pid->error = pid->sum_error = 0.0f;
    pid->p_out = pid->i_out = pid->d_out = pid->out = 0.0f;
    pid->primed = 0U;
}
uint8_t PidPosition_Calculate(PidPosition *pid, float error)
{
    float sum, i_out, candidate;
    if (!pid || !isfinite(error) || !isfinite(pid->kp) || pid->kp < 0.0f ||
        !isfinite(pid->ki) || pid->ki < 0.0f || !isfinite(pid->kd) || pid->kd < 0.0f ||
        !isfinite(pid->deadband) || pid->deadband < 0.0f ||
        !isfinite(pid->out_limit) || pid->out_limit <= 0.0f ||
        !isfinite(pid->integral_limit) || pid->integral_limit < 0.0f) {
        PidPosition_Reset(pid); return 0U;
    }
    if (fabsf(error) < pid->deadband) error = 0.0f;
    pid->p_out = pid->kp * error;
    pid->d_out = pid->primed ? pid->kd * (error - pid->error) : 0.0f;
    sum = pid->ki > 0.0f ? pid->sum_error + error : 0.0f;
    i_out = pid->ki > 0.0f ? Clamp(pid->ki * sum, pid->integral_limit) : 0.0f;
    candidate = pid->p_out + i_out + pid->d_out;
    /* Clamp I BEFORE summing and freeze when it pushes output saturation.
     * Corrects reference post-sum clamp; retains P/I/D parameter semantics. */
    if ((candidate > pid->out_limit && error > 0.0f) ||
        (candidate < -pid->out_limit && error < 0.0f)) i_out = pid->i_out;
    if (!isfinite(candidate) || !isfinite(pid->p_out) || !isfinite(pid->d_out)) {
        PidPosition_Reset(pid); return 0U;
    }
    pid->sum_error = pid->ki > 0.0f ? i_out / pid->ki : 0.0f;
    pid->i_out = i_out;
    pid->out = Clamp(pid->p_out + pid->i_out + pid->d_out, pid->out_limit);
    pid->error = error;
    pid->primed = 1U;
    return 1U;
}
