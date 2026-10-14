#include "x42s_safety.h"
#include "board_bringup.h"
#include "main.h"

/* This diagnostic firmware never sends a motion, enable, speed or position command. */
#define X42S_FIRST_ID                         1U
#define X42S_LAST_ID                          4U
#define X42S_CONTROL_ID                       5U
#define X42S_PROBE_ID                         3U
#define X42S_CONFIG_FRAME_LENGTH              33U
#define X42S_PROBE_BOOT_MARKER_MS             1500U
#define X42S_PROBE_REPLY_TIMEOUT_MS           500U
#define X42S_RESULT_PERIOD_MS                 2000U
#define X42S_RESULT_SILENT_PULSE_MS           400U
#define X42S_RESULT_GARBLED_PULSE_MS          150U
#define X42S_RESULT_GARBLED_GAP_MS            200U
#define X42S_INTER_COMMAND_GAP_MS              10U
#define X42S_ID3_DIAG_REPLY_WINDOW_MS          500U
#define X42S_ID3_DIAG_STEP_COUNT                 6U
#define X42S_ID3_DIAG_ID                         3U

/* This is deliberately a compile-time fuse. Setting it non-zero must fail the build. */
#define X42S_TEST_ARMED                        0U

#if (X42S_TEST_ARMED != 0U)
#error "X42S motor tests are forbidden in the X42S diagnostic firmware"
#endif

typedef enum
{
    X42S_PROBE_IDLE = 0,
    X42S_PROBE_WAIT_BOOT,
    X42S_PROBE_WAIT_CONTROL_REPLY,
    X42S_PROBE_WAIT_TARGET_REPLY,
    X42S_PROBE_DONE
} X42S_ProbeState;

typedef enum
{
    X42S_PROBE_RESULT_PENDING = 0,
    X42S_PROBE_RESULT_VALID,
    X42S_PROBE_RESULT_SILENT,
    X42S_PROBE_RESULT_TARGET_PARTIAL,
    X42S_PROBE_RESULT_TARGET_QUERY_ECHO,
    X42S_PROBE_RESULT_TARGET_MALFORMED,
    X42S_PROBE_RESULT_TARGET_UART_ERROR,
    X42S_PROBE_RESULT_CONTROL_NOISE
} X42S_ProbeResult;

static volatile uint8_t x42s_rx_frame[X42S_CONFIG_FRAME_LENGTH];
static volatile uint8_t x42s_rx_count;
static volatile uint8_t x42s_rx_ready;
static volatile uint8_t x42s_rx_any_byte;
static volatile uint8_t x42s_rx_overflow;
static volatile uint8_t x42s_rx_uart_error;
static X42S_ProbeState x42s_probe_state;
static X42S_ProbeResult x42s_probe_result;
static uint8_t x42s_control_had_bytes;
static uint32_t x42s_probe_started_ms;
static uint32_t x42s_result_started_ms;
typedef enum
{
    X42S_TX_OK = 0,
    X42S_TX_TXE_TIMEOUT,
    X42S_TX_TC_TIMEOUT
} X42S_TxResult;

/* Each command has its own 500 ms receive window. All are stop/disable/read-only. */
static const uint8_t x42s_id3_commands[X42S_ID3_DIAG_STEP_COUNT][6] =
{
    {3U, 0xFEU, 0x98U, 0U, 0x6BU, 0U},
    {3U, 0xF3U, 0xABU, 0U, 0U, 0x6BU},
    {3U, 0x1FU, 0x6BU, 0U, 0U, 0U},
    {3U, 0x42U, 0x6CU, 0x6BU, 0U, 0U},
    {3U, 0x3AU, 0x6BU, 0U, 0U, 0U},
    {3U, 0x35U, 0x6BU, 0U, 0U, 0U}
};
static const uint8_t x42s_id3_tx_lengths[] = {5U, 6U, 3U, 4U, 3U, 3U};
static const uint8_t x42s_id3_rx_lengths[] = {4U, 4U, 7U, 33U, 4U, 6U};
static uint8_t x42s_id3_step;
static uint8_t x42s_id3_good_mask;
static uint8_t x42s_id3_bad_reply;
static uint8_t x42s_id3_response_mode;
static uint8_t x42s_id3_flags;
static uint16_t x42s_id3_speed_raw;
static uint16_t x42s_id3_firmware_version;
static uint16_t x42s_id3_hardware_version;
static X42S_TxResult x42s_id3_tx_fault;
static uint32_t x42s_id3_sent_ms;
static uint32_t x42s_id3_marker_started_ms;
/* 0 pending, 1 silent, 2 TX timeout, 3 invalid/rejected, 4 unsafe snapshot,
 * 5 queries valid but control ACK disabled by config, 6 all checks passed. */
static uint8_t x42s_id3_result;

static void X42S_UartInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t pclk1_hz;

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* Keep a disconnected RX at UART idle high during the power-off test. */
    gpio.Pin = GPIO_PIN_9;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &gpio);

    USART3->CR1 = 0U;
    USART3->CR2 = 0U;
    USART3->CR3 = 0U;
    pclk1_hz = HAL_RCC_GetPCLK1Freq();
    USART3->BRR = pclk1_hz / 115200U;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    HAL_NVIC_SetPriority(USART3_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

static X42S_TxResult X42S_Send(const uint8_t *frame, uint16_t length)
{
    uint16_t index;
    uint32_t started_ms;

    for (index = 0U; index < length; index++)
    {
        started_ms = HAL_GetTick();
        while ((USART3->SR & USART_SR_TXE) == 0U)
        {
            if ((HAL_GetTick() - started_ms) > 100U)
            {
                HAL_Delay(X42S_INTER_COMMAND_GAP_MS);
                return X42S_TX_TXE_TIMEOUT;
            }
        }
        USART3->DR = frame[index];
    }

    started_ms = HAL_GetTick();
    while ((USART3->SR & USART_SR_TC) == 0U)
    {
        if ((HAL_GetTick() - started_ms) > 100U)
        {
            HAL_Delay(X42S_INTER_COMMAND_GAP_MS);
            return X42S_TX_TC_TIMEOUT;
        }
    }
    /* RM0090 TC means the last stop bit left USART. It does not prove reception.
     * ZDT manual p37 requires a few ms between commands; use at least 10 ms. */
    HAL_Delay(X42S_INTER_COMMAND_GAP_MS);
    return X42S_TX_OK;
}

static X42S_TxResult X42S_SendStop(uint8_t motor_id)
{
    const uint8_t frame[] = {motor_id, 0xFEU, 0x98U, 0x00U, 0x6BU};
    return X42S_Send(frame, sizeof(frame));
}

static X42S_TxResult X42S_SendDisable(uint8_t motor_id)
{
    const uint8_t frame[] = {motor_id, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    return X42S_Send(frame, sizeof(frame));
}

static void X42S_SendStopAndDisableToAll(void)
{
    uint8_t motor_id;

    for (motor_id = X42S_FIRST_ID; motor_id <= X42S_LAST_ID; motor_id++)
    {
        X42S_SendStop(motor_id);
        X42S_SendDisable(motor_id);
    }
}

static void X42S_ResetReplyCapture(uint8_t motor_id)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    x42s_rx_count = 0U;
    x42s_rx_ready = 0U;
    x42s_rx_any_byte = 0U;
    x42s_rx_overflow = 0U;
    x42s_rx_uart_error = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void X42S_SendConfigRead(uint8_t motor_id)
{
    const uint8_t frame[] = {motor_id, 0x42U, 0x6CU, 0x6BU};

    X42S_ResetReplyCapture(motor_id);
    X42S_Send(frame, sizeof(frame));
}

static uint8_t X42S_HasExpectedConfigReply(uint8_t motor_id)
{
    uint8_t received_count;
    uint8_t received_ready;
    uint8_t first_byte;
    uint8_t second_byte;
    uint8_t last_byte;
    uint8_t overflow;
    uint8_t uart_error;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    received_count = x42s_rx_count;
    received_ready = x42s_rx_ready;
    first_byte = x42s_rx_frame[0U];
    second_byte = x42s_rx_frame[1U];
    last_byte = x42s_rx_frame[X42S_CONFIG_FRAME_LENGTH - 1U];
    overflow = x42s_rx_overflow;
    uart_error = x42s_rx_uart_error;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return (uint8_t)((received_ready != 0U) &&
                     (received_count == X42S_CONFIG_FRAME_LENGTH) &&
                     (overflow == 0U) &&
                     (uart_error == 0U) &&
                     (first_byte == motor_id) &&
                     (second_byte == 0x42U) &&
                     (last_byte == 0x6BU));
}

static X42S_ProbeResult X42S_ClassifyTargetReply(void)
{
    const uint8_t query[] = {X42S_PROBE_ID, 0x42U, 0x6CU, 0x6BU};
    uint8_t received_count;
    uint8_t overflow;
    uint8_t uart_error;
    uint8_t query_echo = 1U;
    uint8_t index;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    received_count = x42s_rx_count;
    overflow = x42s_rx_overflow;
    uart_error = x42s_rx_uart_error;
    if (received_count == sizeof(query))
    {
        for (index = 0U; index < sizeof(query); index++)
        {
            if (x42s_rx_frame[index] != query[index])
            {
                query_echo = 0U;
                break;
            }
        }
    }
    else
    {
        query_echo = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (uart_error != 0U)
    {
        return X42S_PROBE_RESULT_TARGET_UART_ERROR;
    }

    if ((overflow == 0U) && (received_count > 0U) && (received_count <= 3U))
    {
        return X42S_PROBE_RESULT_TARGET_PARTIAL;
    }

    if ((overflow == 0U) && (query_echo != 0U))
    {
        return X42S_PROBE_RESULT_TARGET_QUERY_ECHO;
    }

    return X42S_PROBE_RESULT_TARGET_MALFORMED;
}

static void X42S_FinishProbe(X42S_ProbeResult result)
{
    x42s_probe_result = result;
    x42s_probe_state = X42S_PROBE_DONE;
    x42s_result_started_ms = HAL_GetTick();
}

static void X42S_UpdateProbeMarker(void)
{
    uint32_t phase;
    uint8_t pulse_count;
    uint8_t pulse_index;

    if (x42s_probe_state == X42S_PROBE_WAIT_BOOT)
    {
        BoardBringup_SetMarker(1U);
        return;
    }

    if ((x42s_probe_state == X42S_PROBE_WAIT_CONTROL_REPLY) ||
        (x42s_probe_state == X42S_PROBE_WAIT_TARGET_REPLY))
    {
        BoardBringup_SetMarker(0U);
        return;
    }

    if (x42s_probe_state != X42S_PROBE_DONE)
    {
        BoardBringup_SetMarker(0U);
        return;
    }

    if (x42s_probe_result == X42S_PROBE_RESULT_VALID)
    {
        BoardBringup_SetMarker(1U);
        return;
    }

    phase = (HAL_GetTick() - x42s_result_started_ms) % X42S_RESULT_PERIOD_MS;
    if (x42s_probe_result == X42S_PROBE_RESULT_SILENT)
    {
        BoardBringup_SetMarker((uint8_t)(phase < X42S_RESULT_SILENT_PULSE_MS));
        return;
    }

    if (x42s_probe_result == X42S_PROBE_RESULT_TARGET_PARTIAL)
    {
        pulse_count = 2U;
    }
    else if (x42s_probe_result == X42S_PROBE_RESULT_CONTROL_NOISE)
    {
        pulse_count = 3U;
    }
    else if (x42s_probe_result == X42S_PROBE_RESULT_TARGET_QUERY_ECHO)
    {
        pulse_count = 4U;
    }
    else if (x42s_probe_result == X42S_PROBE_RESULT_TARGET_MALFORMED)
    {
        pulse_count = 5U;
    }
    else if (x42s_probe_result == X42S_PROBE_RESULT_TARGET_UART_ERROR)
    {
        pulse_count = 6U;
    }
    else
    {
        BoardBringup_SetMarker(0U);
        return;
    }

    for (pulse_index = 0U; pulse_index < pulse_count; pulse_index++)
    {
        uint32_t pulse_start = pulse_index * (X42S_RESULT_GARBLED_PULSE_MS + X42S_RESULT_GARBLED_GAP_MS);
        if ((phase >= pulse_start) && (phase < (pulse_start + X42S_RESULT_GARBLED_PULSE_MS)))
        {
            BoardBringup_SetMarker(1U);
            return;
        }
    }

    BoardBringup_SetMarker(0U);
}

void X42S_ReadOnlyProbeInit(void)
{
    /* PB8 lights immediately: this proves the diagnostic firmware reached its entry point. */
    BoardBringup_TestSignalLow();
    X42S_UartInit();
    X42S_SendStopAndDisableToAll();
    X42S_ResetReplyCapture(0U);

    x42s_probe_state = X42S_PROBE_WAIT_BOOT;
    x42s_probe_result = X42S_PROBE_RESULT_PENDING;
    x42s_control_had_bytes = 0U;
    x42s_probe_started_ms = HAL_GetTick();
    x42s_result_started_ms = 0U;
}

void X42S_ReadOnlyProbeService(void)
{
    uint32_t elapsed_ms;

    elapsed_ms = HAL_GetTick() - x42s_probe_started_ms;
    if (x42s_probe_state == X42S_PROBE_WAIT_BOOT)
    {
        if (elapsed_ms >= X42S_PROBE_BOOT_MARKER_MS)
        {
            /* ID5 is deliberately unused: it is a no-motion control for line echo/noise. */
            X42S_SendConfigRead(X42S_CONTROL_ID);
            x42s_probe_state = X42S_PROBE_WAIT_CONTROL_REPLY;
            x42s_probe_started_ms = HAL_GetTick();
        }
    }
    else if (x42s_probe_state == X42S_PROBE_WAIT_CONTROL_REPLY)
    {
        if (elapsed_ms >= X42S_PROBE_REPLY_TIMEOUT_MS)
        {
            x42s_control_had_bytes = x42s_rx_any_byte;
            X42S_SendConfigRead(X42S_PROBE_ID);
            x42s_probe_state = X42S_PROBE_WAIT_TARGET_REPLY;
            x42s_probe_started_ms = HAL_GetTick();
        }
    }
    else if (x42s_probe_state == X42S_PROBE_WAIT_TARGET_REPLY)
    {
        if (elapsed_ms >= X42S_PROBE_REPLY_TIMEOUT_MS)
        {
            if (x42s_control_had_bytes != 0U)
            {
                X42S_FinishProbe(X42S_PROBE_RESULT_CONTROL_NOISE);
            }
            else if (X42S_HasExpectedConfigReply(X42S_PROBE_ID) != 0U)
            {
                X42S_FinishProbe(X42S_PROBE_RESULT_VALID);
            }
            else if (x42s_rx_any_byte != 0U)
            {
                X42S_FinishProbe(X42S_ClassifyTargetReply());
            }
            else
            {
                X42S_FinishProbe(X42S_PROBE_RESULT_SILENT);
            }
        }
    }

    X42S_UpdateProbeMarker();
}

static void X42S_Id3SetResult(uint8_t result)
{
    if (result != x42s_id3_result)
    {
        x42s_id3_result = result;
        x42s_id3_marker_started_ms = HAL_GetTick();
    }
}

static void X42S_Id3StartTransaction(void)
{
    X42S_TxResult result;
    X42S_ResetReplyCapture(X42S_ID3_DIAG_ID);
    result = X42S_Send(x42s_id3_commands[x42s_id3_step],
                       x42s_id3_tx_lengths[x42s_id3_step]);
    if ((result != X42S_TX_OK) && (x42s_id3_tx_fault == X42S_TX_OK))
    {
        x42s_id3_tx_fault = result; /* Keep the first local fault until reset. */
        X42S_Id3SetResult(2U);
    }
    x42s_id3_sent_ms = HAL_GetTick();
}

static void X42S_Id3ClassifyTransaction(void)
{
    uint8_t frame[X42S_CONFIG_FRAME_LENGTH];
    uint8_t count, fault, index;
    uint8_t step = x42s_id3_step;
    uint8_t code = x42s_id3_commands[step][1U];
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    count = x42s_rx_count;
    fault = x42s_rx_overflow | x42s_rx_uart_error;
    for (index = 0U; index < count; index++) frame[index] = x42s_rx_frame[index];
    if (primask == 0U) __enable_irq();

    if (fault != 0U) { x42s_id3_bad_reply = 1U; return; }
    if (count == 0U) return; /* Missing ACK is evaluated against Response below. */

    if (step <= 1U)
    {
        /* Both mode may append a completion frame. Match each complete frame. */
        if ((count != 4U) && (count != 8U)) { x42s_id3_bad_reply = 1U; return; }
        for (index = 0U; index < count; index += 4U)
        {
            if ((frame[index] != X42S_ID3_DIAG_ID) ||
                (frame[index + 1U] != code) || (frame[index + 3U] != 0x6BU))
            { x42s_id3_bad_reply = 1U; return; }
            if (frame[index + 2U] == 0x02U) x42s_id3_good_mask |= (uint8_t)(1U << step);
            else if (frame[index + 2U] != 0x9FU) x42s_id3_bad_reply = 1U;
        }
        return;
    }

    if ((count != x42s_id3_rx_lengths[step]) || (frame[0U] != X42S_ID3_DIAG_ID) ||
        (frame[1U] != code) || (frame[count - 1U] != 0x6BU))
    { x42s_id3_bad_reply = 1U; return; }

    if (step == 2U)
    {
        x42s_id3_firmware_version = ((uint16_t)frame[2U] << 8) | frame[3U];
        x42s_id3_hardware_version = ((uint16_t)frame[4U] << 8) | frame[5U];
    }
    else if (step == 3U)
    {
        if ((frame[2U] != 0x21U) || (frame[3U] != 0x15U) ||
            (frame[6U] != 2U) || (frame[18U] != 5U) ||
            (frame[20U] != X42S_ID3_DIAG_ID) || (frame[21U] != 0U) || (frame[22U] > 4U))
        { x42s_id3_bad_reply = 1U; return; }
        x42s_id3_response_mode = frame[22U];
    }
    else if (step == 4U) x42s_id3_flags = frame[2U];
    else
    {
        if (frame[2U] > 1U) { x42s_id3_bad_reply = 1U; return; }
        x42s_id3_speed_raw = ((uint16_t)frame[3U] << 8) | frame[4U];
    }
    x42s_id3_good_mask |= (uint8_t)(1U << step);
}

static uint8_t X42S_Id3EvaluateCycle(void)
{
    if (x42s_id3_tx_fault != X42S_TX_OK) return 2U;
    if (x42s_id3_bad_reply != 0U) return 3U;
    /* Judge the two available state snapshots even if another query is silent. */
    if (((x42s_id3_good_mask & 0x10U) != 0U) && ((x42s_id3_flags & 1U) != 0U)) return 4U;
    if (((x42s_id3_good_mask & 0x20U) != 0U) && (x42s_id3_speed_raw != 0U)) return 4U;
    if ((x42s_id3_good_mask & 0x3CU) != 0x3CU) return 1U;
    if ((x42s_id3_good_mask & 3U) == 3U) return 6U;
    if ((x42s_id3_response_mode == 0U) || (x42s_id3_response_mode == 2U)) return 5U;
    return 1U;
}

static void X42S_Id3UpdateMarker(void)
{
    uint32_t phase = (HAL_GetTick() - x42s_id3_marker_started_ms) % 2500U;
    uint8_t asserted = 0U;
    if (x42s_id3_result == 6U) asserted = 1U;
    else if (x42s_id3_result == 1U) asserted = (phase < 400U) ? 1U : 0U;
    else if (x42s_id3_result >= 2U)
    {
        asserted = ((phase < ((uint32_t)x42s_id3_result * 350U)) &&
                    ((phase % 350U) < 150U)) ? 1U : 0U;
    }
    /* Steady means valid ACKs plus disabled/zero-speed snapshots, never a
     * demonstration that this firmware can stop a motor already in motion. */
    BoardBringup_SetMarker(asserted);
}

void X42S_Id3StopRecoveryInit(void)
{
    BoardBringup_TestSignalLow();
    X42S_UartInit();
    x42s_id3_step = 0U;
    x42s_id3_good_mask = 0U;
    x42s_id3_bad_reply = 0U;
    x42s_id3_tx_fault = X42S_TX_OK;
    x42s_id3_result = 0U;
    x42s_id3_marker_started_ms = HAL_GetTick();
    BoardBringup_SetMarker(0U);
    X42S_Id3StartTransaction(); /* Stop is always the first command after reset. */
}

void X42S_Id3StopRecoveryService(void)
{
    if ((HAL_GetTick() - x42s_id3_sent_ms) >= X42S_ID3_DIAG_REPLY_WINDOW_MS)
    {
        X42S_Id3ClassifyTransaction();
        x42s_id3_step++;
        if (x42s_id3_step == X42S_ID3_DIAG_STEP_COUNT)
        {
            X42S_Id3SetResult(X42S_Id3EvaluateCycle());
            x42s_id3_step = 0U;
            x42s_id3_good_mask = 0U;
            x42s_id3_bad_reply = 0U;
        }
        X42S_Id3StartTransaction();
    }
    X42S_Id3UpdateMarker();
}


/* Keep the retired diagnostic buildable when CubeIDE rescans Core/Src.
 * The active step_motor driver provides the strong USART3 handler. If that
 * driver is absent, this weak handler still supports the legacy diagnostic. */
__attribute__((weak)) void USART3_IRQHandler(void)
{
    uint32_t status = USART3->SR;
    uint8_t byte_received;

    if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
        x42s_rx_uart_error = 1U;
    }

    if ((status & USART_SR_RXNE) != 0U)
    {
        byte_received = (uint8_t)USART3->DR;
        x42s_rx_any_byte = 1U;

        if (x42s_rx_count < X42S_CONFIG_FRAME_LENGTH)
        {
            x42s_rx_frame[x42s_rx_count] = byte_received;
            x42s_rx_count++;
        }
        else
        {
            x42s_rx_overflow = 1U;
        }

        if (x42s_rx_count == X42S_CONFIG_FRAME_LENGTH)
        {
            x42s_rx_ready = 1U;
        }
    }
    else if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
        (void)USART3->DR;
    }
}
