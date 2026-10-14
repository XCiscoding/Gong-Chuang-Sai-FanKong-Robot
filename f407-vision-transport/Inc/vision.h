#ifndef VISION_H
#define VISION_H

#include <stdint.h>
#include "vision_uart.h"

#define VISION_TEXT_LINE_MAX 80U
#define VISION_FRESH_MS 200U
#define VISION_MODE_RETRY_MS 500U

/* This image is a passive UART/vision proof. Keep every motor path suppressed
 * until text protocol, sign, LOST and timeout behaviour pass on real hardware. */
#ifndef VISION_LINK_TEST_ACTIVE
#define VISION_LINK_TEST_ACTIVE 1U
#endif
#if VISION_LINK_TEST_ACTIVE != 0U && VISION_LINK_TEST_ACTIVE != 1U
#error "Invalid VISION_LINK_TEST_ACTIVE"
#endif

typedef enum {
    VISION_MODE_IDLE = 0,
    VISION_MODE_TRACK,
    VISION_MODE_RECOG
} VisionMode;

#ifndef VISION_STARTUP_MODE
#define VISION_STARTUP_MODE VISION_MODE_TRACK
#endif

typedef enum {
    VISION_TARGET_NONE = 0,
    VISION_TARGET_QR,
    VISION_TARGET_BALL,
    VISION_TARGET_TARGET
} VisionTarget;

typedef struct {
    VisionMode mode;
    VisionTarget target;
    uint8_t valid;
    uint16_t sequence;
    int16_t dx;
    int16_t dy;
    int16_t angle_tenths;
    uint8_t quality;
    uint8_t color;
    uint16_t task_code;
    uint32_t received_ms;
} VisionObservation;

typedef struct {
    uint32_t complete_lines;
    uint32_t malformed_lines;
    uint32_t line_overflow;
    uint32_t acknowledgements;
    uint32_t last_ack_ms;
    VisionMode acknowledged_mode;
    uint32_t stale_sequences;
    uint32_t unexpected_frames;
    uint32_t session_resets;
    uint32_t link_timeouts;
    uint32_t lost_frames;
    uint32_t unstable_frames;
    uint8_t link_active;
    uint16_t last_sequence;
    VisionUartDiagnostics uart;
} VisionDiagnostics;

extern volatile VisionObservation vision_latest;
extern volatile VisionDiagnostics vision_diag;
extern volatile char vision_debug_last_line[VISION_TEXT_LINE_MAX];
extern volatile uint8_t vision_debug_last_line_length;
extern volatile uint32_t vision_debug_mode_request;
extern volatile uint32_t vision_debug_mode_sent;

void Vision_Init(void);
void Vision_Service(void);
uint8_t Vision_RequestMode(VisionMode mode);
uint8_t Vision_GetLatest(VisionObservation *observation);
uint8_t Vision_IsFresh(uint32_t now_ms);
void Vision_GetDiagnostics(VisionDiagnostics *diagnostics);

#endif /* VISION_H */
