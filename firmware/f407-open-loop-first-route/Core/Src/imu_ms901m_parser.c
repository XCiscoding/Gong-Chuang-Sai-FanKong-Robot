#include "imu_ms901m.h"
#include <string.h>

/* Port the verified Test1 MS901M stream, NOT the HWT101 55/53 protocol:
 * 55 55 type payload_length payload... sum8. Angle type=01, length=06. */
void ImuMs901m_ParserInit(ImuMs901mParser *parser)
{
    if (parser) memset(parser, 0, sizeof(*parser));
}
static void DropFirst(ImuMs901mParser *p)
{
    p->length--;
    memmove(p->frame, p->frame + 1U, p->length);
}
static int32_t ReadAngle(const uint8_t *b)
{
    int16_t raw = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    return ((int32_t)raw * 1800) / 32768;
}
void ImuMs901m_Feed(ImuMs901mParser *p, uint8_t byte, uint32_t now_ms)
{
    uint32_t expected;
    uint8_t sum, i;
    if (!p) return;
    p->data.raw_bytes++;
    if (p->length == IMU_MS901M_FRAME_MAX) DropFirst(p);
    p->frame[p->length++] = byte;
    for (;;) {
        if (!p->length) return;
        if (p->frame[0] != 0x55U) { DropFirst(p); continue; }
        if (p->length < 2U) return;
        if (p->frame[1] != 0x55U) { DropFirst(p); continue; }
        if (p->length < 4U) return;
        expected = 5U + (uint32_t)p->frame[3];
        if (expected < 6U || expected > IMU_MS901M_FRAME_MAX) {
            p->data.length_errors++; DropFirst(p); continue;
        }
        if (p->length < expected) return;
        sum = 0U;
        for (i = 0U; i < expected - 1U; i++) sum = (uint8_t)(sum + p->frame[i]);
        if (sum != p->frame[expected - 1U]) {
            p->data.checksum_errors++; DropFirst(p); continue;
        }
        p->data.valid_frames++;
        if (p->frame[2] == 0x01U && p->frame[3] == 0x06U) {
            p->data.roll_tenths = ReadAngle(p->frame + 4U);
            p->data.pitch_tenths = ReadAngle(p->frame + 6U);
            p->data.yaw_tenths = ReadAngle(p->frame + 8U);
            p->data.angle_frames++;
            p->data.last_angle_ms = now_ms;
        }
        p->length = (uint8_t)(p->length - expected);
        memmove(p->frame, p->frame + expected, p->length);
    }
}
uint8_t ImuMs901m_IsFresh(const ImuMs901mSnapshot *data, uint32_t now_ms)
{
    return data && data->angle_frames != 0U &&
           (uint32_t)(now_ms - data->last_angle_ms) <= IMU_MS901M_FRESH_MS;
}
int32_t ImuMs901m_AngleDifference(int32_t angle, int32_t reference)
{
    int32_t difference = ((angle % 3600) - (reference % 3600)) % 3600;
    if (difference >= 1800) difference -= 3600;
    if (difference < -1800) difference += 3600;
    return difference;
}
