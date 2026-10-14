#ifndef IMU_MS901M_H
#define IMU_MS901M_H
#include <stdint.h>

/* This diagnostic does not initialise the motor bus or PB8. */
#ifndef IMU_ONLY_TEST_ACTIVE
#define IMU_ONLY_TEST_ACTIVE 0
#endif
/* Heading observation uses SWD RAM; disable blocking diagnostic UART. */
#ifndef IMU_DIAGNOSTIC_LOG_ENABLED
#define IMU_DIAGNOSTIC_LOG_ENABLED 0
#endif
#define IMU_MS901M_BAUDRATE 115200U
#define IMU_MS901M_FRESH_MS 500U
#define IMU_MS901M_FRAME_MAX 32U

typedef struct {
    uint32_t raw_bytes, valid_frames, angle_frames;
    uint32_t checksum_errors, length_errors, last_angle_ms;
    int32_t roll_tenths, pitch_tenths, yaw_tenths;
} ImuMs901mSnapshot;
typedef struct {
    uint8_t frame[IMU_MS901M_FRAME_MAX];
    uint8_t length;
    ImuMs901mSnapshot data;
} ImuMs901mParser;
typedef struct {
    ImuMs901mSnapshot data;
    uint32_t uart_errors, age_ms, state, zero_ready;
    int32_t zero_yaw_tenths, relative_yaw_tenths;
} ImuMs901mDiagnostics;
/* state: 0 waiting, 1 fresh, 2 stale. Angles are tenths of degrees. */
extern volatile ImuMs901mDiagnostics imu_ms901m_diag;
void ImuMs901m_ParserInit(ImuMs901mParser *parser);
void ImuMs901m_Feed(ImuMs901mParser *parser, uint8_t byte, uint32_t now_ms);
uint8_t ImuMs901m_IsFresh(const ImuMs901mSnapshot *data, uint32_t now_ms);
int32_t ImuMs901m_AngleDifference(int32_t angle, int32_t reference);
void ImuMs901m_Init(void);
void ImuMs901m_Service(void);
#endif
