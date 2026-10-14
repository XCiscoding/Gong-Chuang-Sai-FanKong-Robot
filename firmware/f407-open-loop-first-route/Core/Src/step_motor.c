#include "step_motor.h"
#include "main.h"

#define STEP_MOTOR_INTER_COMMAND_GAP_MS 10U
#define STEP_MOTOR_TX_TIMEOUT_MS       100U

static volatile uint8_t step_motor_rx[STEP_MOTOR_MAX_REPLY_BYTES];
static volatile uint8_t step_motor_rx_count;
static volatile uint8_t step_motor_rx_overflow;
static volatile uint8_t step_motor_rx_uart_error;
static int32_t step_motor_targets[4];

void StepMotor_ClearTargets(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++) step_motor_targets[i] = 0;
}
uint8_t StepMotor_SetTargetSpeed(uint8_t id, int32_t speed_rpm)
{
    if (id < 1U || id > 4U || speed_rpm < -3000 || speed_rpm > 3000) return 0U;
    step_motor_targets[id - 1U] = speed_rpm;
    return 1U;
}
uint8_t StepMotor_GetTargetSpeed(uint8_t id, uint8_t *direction, uint16_t *speed_rpm)
{
    int32_t speed;
    if (id < 1U || id > 4U || direction == 0 || speed_rpm == 0) return 0U;
    speed = step_motor_targets[id - 1U];
    *direction = (speed < 0) ? 1U : 0U;
    *speed_rpm = (uint16_t)((speed < 0) ? -speed : speed);
    return 1U;
}

static void StepMotor_ResetReply(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    step_motor_rx_count = 0U;
    step_motor_rx_overflow = 0U;
    step_motor_rx_uart_error = 0U;
    if (primask == 0U) __enable_irq();
}

static StepMotorTxResult StepMotor_WriteFrame(const uint8_t *frame, uint8_t length,
                                              uint8_t reset_reply, uint8_t add_gap)
{
    uint8_t index;
    uint32_t started_ms;

    if (reset_reply != 0U) StepMotor_ResetReply();
    started_ms = HAL_GetTick();
    for (index = 0U; index < length; index++)
    {
        while ((USART3->SR & USART_SR_TXE) == 0U)
        {
            if ((HAL_GetTick() - started_ms) > STEP_MOTOR_TX_TIMEOUT_MS)
            {
                if (add_gap != 0U) HAL_Delay(STEP_MOTOR_INTER_COMMAND_GAP_MS);
                return STEP_MOTOR_TX_TXE_TIMEOUT;
            }
        }
        USART3->DR = frame[index];
    }

    while ((USART3->SR & USART_SR_TC) == 0U)
    {
        if ((HAL_GetTick() - started_ms) > STEP_MOTOR_TX_TIMEOUT_MS)
        {
            if (add_gap != 0U) HAL_Delay(STEP_MOTOR_INTER_COMMAND_GAP_MS);
            return STEP_MOTOR_TX_TC_TIMEOUT;
        }
    }

    if (add_gap != 0U) HAL_Delay(STEP_MOTOR_INTER_COMMAND_GAP_MS);
    return STEP_MOTOR_TX_OK;
}

static StepMotorTxResult StepMotor_SendFrame(const uint8_t *frame, uint8_t length)
{
    /* Commissioning/configuration path: one command, one reply window. */
    return StepMotor_WriteFrame(frame, length, 1U, 1U);
}

StepMotorTxResult StepMotor_SendTargetFast(uint8_t id)
{
    uint8_t direction;
    uint16_t speed_rpm;
    uint8_t frame[8];
    if (!StepMotor_GetTargetSpeed(id, &direction, &speed_rpm))
        return STEP_MOTOR_TX_PARAMETER;
    frame[0] = id;
    frame[1] = 0xF6U;
    frame[2] = direction;
    frame[3] = (uint8_t)(speed_rpm >> 8);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = 0U;
    frame[6] = 1U;
    frame[7] = 0x6BU;
    return StepMotor_WriteFrame(frame, sizeof(frame), 1U, 0U);
}

StepMotorTxResult StepMotor_SyncMoveFast(void)
{
    const uint8_t frame[] = {0U, 0xFFU, 0x66U, 0x6BU};
    return StepMotor_WriteFrame(frame, sizeof(frame), 1U, 0U);
}

void StepMotor_Init(void)
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
    StepMotor_ResetReply();
}

StepMotorTxResult StepMotor_Enable(uint8_t id, uint8_t enable)
{
    const uint8_t frame[] = {id, 0xF3U, 0xABU,
                             (enable != 0U) ? 1U : 0U, 0x00U, 0x6BU};
    return StepMotor_SendFrame(frame, sizeof(frame));
}

StepMotorTxResult StepMotor_SetSpeed(uint8_t id, uint8_t direction,
                                     uint16_t speed_rpm, uint8_t acceleration)
{
    return StepMotor_SetSpeedSync(id, direction, speed_rpm, acceleration, 0U);
}

StepMotorTxResult StepMotor_SetSpeedSync(uint8_t id, uint8_t direction,
                                         uint16_t speed_rpm,
                                         uint8_t acceleration,
                                         uint8_t sync)
{
    /* Byte-for-byte frame from the senior/junior StepMotor_ZDTSetSpeed(). */
    const uint8_t frame[] = {id, 0xF6U, (direction != 0U) ? 1U : 0U,
                             (uint8_t)(speed_rpm >> 8), (uint8_t)speed_rpm,
                             acceleration, (sync != 0U) ? 1U : 0U, 0x6BU};
    return StepMotor_SendFrame(frame, sizeof(frame));
}

StepMotorTxResult StepMotor_SyncMove(uint8_t id)
{
    /* Byte-for-byte frame from the senior/junior StepMotor_ZDTSyncMove(). */
    const uint8_t frame[] = {id, 0xFFU, 0x66U, 0x6BU};
    return StepMotor_SendFrame(frame, sizeof(frame));
}

StepMotorTxResult StepMotor_Stop(uint8_t id)
{
    const uint8_t frame[] = {id, 0xFEU, 0x98U, 0x00U, 0x6BU};
    return StepMotor_SendFrame(frame, sizeof(frame));
}

StepMotorTxResult StepMotor_ReadBusVoltage(uint8_t id)
{
    const uint8_t frame[] = {id, 0x24U, 0x6BU};
    return StepMotor_SendFrame(frame, sizeof(frame));
}

StepMotorTxResult StepMotor_ReadStatus(uint8_t id)
{
    const uint8_t frame[] = {id, 0x3AU, 0x6BU};
    return StepMotor_SendFrame(frame, sizeof(frame));
}

StepMotorTxResult StepMotor_ReadSpeed(uint8_t id)
{
    const uint8_t frame[] = {id, 0x35U, 0x6BU};
    return StepMotor_SendFrame(frame, sizeof(frame));
}

StepMotorTxResult StepMotor_ReadConfig(uint8_t id)
{
    /* Read-only configuration query retained from the supplied X42S code. */
    const uint8_t frame[] = {id, 0x42U, 0x6CU, 0x6BU};
    return StepMotor_SendFrame(frame, sizeof(frame));
}

void StepMotor_GetReply(StepMotorReply *reply)
{
    uint8_t index;
    uint8_t count;
    uint32_t primask;

    if (reply == 0) return;
    primask = __get_PRIMASK();
    __disable_irq();
    count = step_motor_rx_count;
    if (count > STEP_MOTOR_MAX_REPLY_BYTES) count = STEP_MOTOR_MAX_REPLY_BYTES;
    for (index = 0U; index < count; index++) reply->bytes[index] = step_motor_rx[index];
    reply->length = count;
    reply->overflow = step_motor_rx_overflow;
    reply->uart_error = step_motor_rx_uart_error;
    if (primask == 0U) __enable_irq();
}

uint8_t StepMotor_ReplyIsAck(const StepMotorReply *reply, uint8_t id,
                             uint8_t command, uint8_t *return_code)
{
    uint8_t index;
    uint8_t selected_code;

    if ((reply == 0) || ((reply->length != 4U) && (reply->length != 8U)) ||
        (reply->overflow != 0U) || (reply->uart_error != 0U)) return 0U;

    selected_code = 0x9FU;
    for (index = 0U; index < reply->length; index += 4U)
    {
        if ((reply->bytes[index] != id) ||
            (reply->bytes[index + 1U] != command) ||
            (reply->bytes[index + 3U] != 0x6BU)) return 0U;
        if ((reply->bytes[index + 2U] != 0x02U) &&
            (reply->bytes[index + 2U] != 0x9FU))
            selected_code = reply->bytes[index + 2U];
        else if ((reply->bytes[index + 2U] == 0x02U) &&
                 ((selected_code == 0x9FU) || (selected_code == 0x02U)))
            selected_code = 0x02U;
    }
    if (return_code != 0) *return_code = selected_code;
    return 1U;
}

uint8_t StepMotor_ParseBusVoltage(const StepMotorReply *reply, uint8_t id,
                                  uint16_t *millivolts)
{
    if ((reply == 0) || (millivolts == 0) || (reply->length != 5U) ||
        (reply->overflow != 0U) || (reply->uart_error != 0U) ||
        (reply->bytes[0] != id) || (reply->bytes[1] != 0x24U) ||
        (reply->bytes[4] != 0x6BU)) return 0U;
    *millivolts = ((uint16_t)reply->bytes[2] << 8) | reply->bytes[3];
    return 1U;
}

uint8_t StepMotor_ParseStatus(const StepMotorReply *reply, uint8_t id,
                              uint8_t *status_flags)
{
    if ((reply == 0) || (status_flags == 0) || (reply->length != 4U) ||
        (reply->overflow != 0U) || (reply->uart_error != 0U) ||
        (reply->bytes[0] != id) || (reply->bytes[1] != 0x3AU) ||
        (reply->bytes[3] != 0x6BU)) return 0U;
    *status_flags = reply->bytes[2];
    return 1U;
}

uint8_t StepMotor_ParseSpeed(const StepMotorReply *reply, uint8_t id,
                             uint8_t *negative, uint16_t *speed_raw)
{
    if ((reply == 0) || (negative == 0) || (speed_raw == 0) ||
        (reply->length != 6U) || (reply->overflow != 0U) ||
        (reply->uart_error != 0U) || (reply->bytes[0] != id) ||
        (reply->bytes[1] != 0x35U) || (reply->bytes[2] > 1U) ||
        (reply->bytes[5] != 0x6BU)) return 0U;
    *negative = reply->bytes[2];
    *speed_raw = ((uint16_t)reply->bytes[3] << 8) | reply->bytes[4];
    return 1U;
}

uint8_t StepMotor_ParseConfigIdentity(const StepMotorReply *reply, uint8_t id)
{
    if ((reply == 0) || (reply->length != 33U) ||
        (reply->overflow != 0U) || (reply->uart_error != 0U) ||
        (reply->bytes[0] != id) || (reply->bytes[1] != 0x42U) ||
        (reply->bytes[32] != 0x6BU)) return 0U;
    return 1U;
}

void USART3_IRQHandler(void)
{
    uint32_t status = USART3->SR;
    uint8_t byte_received;

    if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
        step_motor_rx_uart_error = 1U;

    if ((status & USART_SR_RXNE) != 0U)
    {
        byte_received = (uint8_t)USART3->DR;
        if (step_motor_rx_count < STEP_MOTOR_MAX_REPLY_BYTES)
            step_motor_rx[step_motor_rx_count++] = byte_received;
        else
            step_motor_rx_overflow = 1U;
    }
    else if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
        (void)USART3->DR;
    }
}
