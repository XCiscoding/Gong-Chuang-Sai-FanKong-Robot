#include "vision.h"
#include "main.h"
#include <string.h>

static char received_line[VISION_TEXT_LINE_MAX];
static uint8_t received_length;
static uint8_t discard_until_newline;
static uint8_t session_active;
static uint8_t sequence_initialized;
static uint16_t last_sequence;
static uint32_t last_progress_ms;
static uint32_t last_request_ms;

volatile VisionObservation vision_latest;
volatile VisionDiagnostics vision_diag;
volatile char vision_debug_last_line[VISION_TEXT_LINE_MAX];
volatile uint8_t vision_debug_last_line_length;
volatile uint32_t vision_debug_mode_request;
volatile uint32_t vision_debug_mode_sent;

static void SaveDebugLastLine(const char *line)
{
    uint8_t i = 0U;
    if (line == 0) return;
    while (i < VISION_TEXT_LINE_MAX - 1U && line[i] != '\0') {
        vision_debug_last_line[i] = line[i];
        i++;
    }
    vision_debug_last_line[i] = '\0';
    vision_debug_last_line_length = i;
}

static uint8_t ParseUnsigned(const char *text, uint32_t maximum, uint32_t *value)
{
    uint32_t result = 0U;
    if (text == 0 || *text == '\0') return 0U;
    while (*text) {
        uint32_t digit;
        if (*text < '0' || *text > '9') return 0U;
        digit = (uint32_t)(*text++ - '0');
        if (result > (maximum - digit) / 10U) return 0U;
        result = result * 10U + digit;
    }
    *value = result;
    return 1U;
}

static uint8_t ParseSigned(const char *text, int32_t minimum, int32_t maximum,
                           int32_t *value)
{
    uint8_t negative = 0U;
    uint32_t magnitude;
    if (text == 0 || *text == '\0') return 0U;
    if (*text == '-') { negative = 1U; text++; }
    if (!ParseUnsigned(text, (uint32_t)(negative ? -minimum : maximum),
                       &magnitude)) return 0U;
    *value = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    return *value >= minimum && *value <= maximum;
}

static uint8_t Split(char *line, char **fields, uint8_t capacity)
{
    uint8_t count = 0U;
    if (line == 0 || *line == '\0') return 0U;
    fields[count++] = line;
    while (*line) {
        if (*line == ',') {
            *line = '\0';
            if (count == capacity || line[1] == '\0') return 0U;
            fields[count++] = line + 1;
        }
        line++;
    }
    return count;
}

static VisionMode ParseMode(const char *text)
{
    if (strcmp(text, "TRACK") == 0) return VISION_MODE_TRACK;
    if (strcmp(text, "RECOG") == 0) return VISION_MODE_RECOG;
    return VISION_MODE_IDLE;
}

static uint8_t ParseQuality(const char *text, uint8_t *quality)
{
    uint32_t value;
    if (!ParseUnsigned(text, 100U, &value)) return 0U;
    *quality = (uint8_t)value;
    return 1U;
}

static uint8_t ParseTrack(char **field, uint8_t count,
                          VisionObservation *observation)
{
    uint32_t sequence;
    int32_t dx, angle;
    if (!ParseUnsigned(field[1], 65535U, &sequence)) return 0U;
    observation->mode = VISION_MODE_TRACK;
    observation->target = VISION_TARGET_NONE;
    observation->sequence = (uint16_t)sequence;
    observation->received_ms = HAL_GetTick();
    observation->valid = 0U;
    if (count == 3U && strcmp(field[2], "LOST") == 0) return 1U;
    if (count != 5U || !ParseSigned(field[2], -32768, 32767, &dx) ||
        !ParseSigned(field[3], -1800, 1800, &angle) ||
        !ParseQuality(field[4], &observation->quality)) return 0U;
    observation->dx = (int16_t)dx;
    observation->angle_tenths = (int16_t)angle;
    observation->valid = 1U;
    return 1U;
}

static uint8_t ParseRecognition(char **field, uint8_t count,
                                VisionObservation *observation)
{
    uint32_t sequence, value;
    int32_t dx, dy;
    if (!ParseUnsigned(field[1], 65535U, &sequence)) return 0U;
    observation->mode = VISION_MODE_RECOG;
    observation->target = VISION_TARGET_NONE;
    observation->sequence = (uint16_t)sequence;
    observation->received_ms = HAL_GetTick();
    observation->valid = 0U;
    if (count == 3U && strcmp(field[2], "NONE") == 0) return 1U;
    if (count < 5U || strcmp(field[3], "VALID") != 0) return 0U;
    if (strcmp(field[2], "QR") == 0) {
        if (count != 6U || !ParseUnsigned(field[4], 999U, &value) ||
            !ParseQuality(field[5], &observation->quality)) return 0U;
        observation->target = VISION_TARGET_QR;
        observation->task_code = (uint16_t)value;
    } else if (strcmp(field[2], "BALL") == 0 ||
               strcmp(field[2], "TARGET") == 0) {
        if (count != 8U || strlen(field[4]) != 1U ||
            (field[4][0] != 'R' && field[4][0] != 'G' && field[4][0] != 'B') ||
            !ParseSigned(field[5], -32768, 32767, &dx) ||
            !ParseSigned(field[6], -32768, 32767, &dy) ||
            !ParseQuality(field[7], &observation->quality)) return 0U;
        observation->target = (strcmp(field[2], "BALL") == 0) ?
                              VISION_TARGET_BALL : VISION_TARGET_TARGET;
        observation->color = (uint8_t)field[4][0];
        observation->dx = (int16_t)dx;
        observation->dy = (int16_t)dy;
    } else return 0U;
    observation->valid = 1U;
    return 1U;
}

static void InvalidateLatest(void)
{
    vision_latest.valid = 0U;
    vision_latest.quality = 0U;
    vision_latest.dx = 0;
    vision_latest.dy = 0;
    vision_latest.angle_tenths = 0;
}

static void BeginSession(VisionMode mode, uint32_t now_ms)
{
    session_active = 1U;
    sequence_initialized = 0U;
    last_progress_ms = now_ms;
    InvalidateLatest();
    vision_diag.session_resets++;
    vision_diag.link_active = 1U;
    vision_diag.acknowledged_mode = mode;
    vision_diag.last_ack_ms = now_ms;
}

static void EndSession(void)
{
    session_active = 0U;
    sequence_initialized = 0U;
    InvalidateLatest();
    vision_diag.link_active = 0U;
    vision_debug_mode_sent = (uint32_t)VISION_MODE_IDLE;
}

static void AcceptObservation(const VisionObservation *candidate)
{
    uint16_t delta;
    uint32_t requested_mode = vision_debug_mode_request;
    if (!session_active || candidate->mode != (VisionMode)requested_mode) {
        vision_diag.unexpected_frames++;
        return;
    }
    if (sequence_initialized) {
        delta = (uint16_t)(candidate->sequence - last_sequence);
        if (delta == 0U || delta > 0x7FFFU) {
            vision_diag.stale_sequences++;
            return;
        }
    }
    sequence_initialized = 1U;
    last_sequence = candidate->sequence;
    last_progress_ms = candidate->received_ms;
    vision_diag.last_sequence = candidate->sequence;
    vision_latest = *candidate;
    if (!candidate->valid) vision_diag.lost_frames++;
    else if (candidate->quality < 100U) vision_diag.unstable_frames++;
}

static void ParseLine(char *line)
{
    char *field[8];
    uint8_t count = Split(line, field, sizeof(field) / sizeof(field[0]));
    VisionObservation candidate = {0};
    VisionMode acknowledged;
    uint32_t now_ms = HAL_GetTick();

    if (count == 2U && strcmp(field[0], "ACK") == 0 &&
        (acknowledged = ParseMode(field[1])) != VISION_MODE_IDLE) {
        vision_diag.acknowledgements++;
        if ((uint32_t)acknowledged == vision_debug_mode_request) {
            BeginSession(acknowledged, now_ms);
        }
        return;
    }
    if (count >= 3U && strcmp(field[0], "TRACK") == 0 &&
        ParseTrack(field, count, &candidate)) {
        AcceptObservation(&candidate);
        return;
    }
    if (count >= 3U && strcmp(field[0], "RECOG") == 0 &&
        ParseRecognition(field, count, &candidate)) {
        AcceptObservation(&candidate);
        return;
    }
    vision_diag.malformed_lines++;
}

void Vision_Init(void)
{
    const VisionDiagnostics initial = {0};
    const VisionObservation no_observation = {0};
    vision_diag = initial;
    vision_latest = no_observation;
    vision_debug_last_line[0] = '\0';
    vision_debug_last_line_length = 0U;
    vision_debug_mode_request = (uint32_t)VISION_STARTUP_MODE;
    vision_debug_mode_sent = (uint32_t)VISION_MODE_IDLE;
    received_length = discard_until_newline = 0U;
    session_active = sequence_initialized = 0U;
    last_sequence = 0U;
    last_progress_ms = last_request_ms = 0U;
    VisionUart_Init();
}

void Vision_Service(void)
{
    uint8_t byte;
    uint32_t now_ms;
    uint32_t requested_mode;
    VisionUart_Service();

    while (VisionUart_ReadByte(&byte)) {
        if (byte == '\r') continue;
        if (byte == '\n') {
            if (discard_until_newline) {
                discard_until_newline = 0U;
            } else if (received_length != 0U) {
                received_line[received_length] = '\0';
                vision_diag.complete_lines++;
                SaveDebugLastLine(received_line);
                ParseLine(received_line);
            }
            received_length = 0U;
        } else if (discard_until_newline == 0U) {
            if (byte < 0x20U || byte > 0x7EU ||
                received_length >= VISION_TEXT_LINE_MAX - 1U) {
                discard_until_newline = 1U;
                received_length = 0U;
                vision_diag.line_overflow++;
            } else {
                received_line[received_length++] = (char)byte;
            }
        }
    }

    now_ms = HAL_GetTick();
    requested_mode = vision_debug_mode_request;
    if (session_active && (uint32_t)(now_ms - last_progress_ms) > VISION_FRESH_MS) {
        vision_diag.link_timeouts++;
        EndSession();
    }
    if (requested_mode != (uint32_t)VISION_MODE_TRACK &&
        requested_mode != (uint32_t)VISION_MODE_RECOG) {
        EndSession();
        return;
    }
    if (vision_debug_mode_sent != requested_mode) {
        EndSession();
    }
    if (!session_active &&
        (vision_debug_mode_sent != requested_mode ||
         (uint32_t)(now_ms - last_request_ms) >= VISION_MODE_RETRY_MS) &&
        Vision_RequestMode((VisionMode)requested_mode)) {
        vision_debug_mode_sent = requested_mode;
        last_request_ms = now_ms;
    }
}

uint8_t Vision_RequestMode(VisionMode mode)
{
    if (mode == VISION_MODE_TRACK) return VisionUart_Send("MODE,TRACK\n");
    if (mode == VISION_MODE_RECOG) return VisionUart_Send("MODE,RECOG\n");
    return 0U;
}

uint8_t Vision_GetLatest(VisionObservation *observation)
{
    uint32_t primask;
    uint8_t available;
    if (observation == 0) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    *observation = vision_latest;
    available = vision_latest.received_ms != 0U;
    if (primask == 0U) __enable_irq();
    return available;
}

uint8_t Vision_IsFresh(uint32_t now_ms)
{
    return session_active && vision_latest.valid &&
           vision_latest.quality == 100U &&
           vision_latest.received_ms != 0U &&
           (uint32_t)(now_ms - vision_latest.received_ms) <= VISION_FRESH_MS;
}

void Vision_GetDiagnostics(VisionDiagnostics *diagnostics)
{
    if (diagnostics == 0) return;
    *diagnostics = vision_diag;
    VisionUart_GetDiagnostics(&diagnostics->uart);
}
