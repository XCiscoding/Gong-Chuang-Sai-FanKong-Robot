#include "imu_ms901m.h"
#include "main.h"
#include <stdio.h>

volatile ImuMs901mDiagnostics imu_ms901m_diag;
static ImuMs901mParser parser;
static volatile uint32_t uart_errors;
static uint32_t started_ms, last_log_ms;

#if IMU_ONLY_TEST_ACTIVE && IMU_DIAGNOSTIC_LOG_ENABLED
static void LogLine(const char *line)
{
    uint32_t started = HAL_GetTick();
    while (*line) {
        while (!(USART1->SR & USART_SR_TXE)) {
            if ((uint32_t)(HAL_GetTick() - started) > 25U) return;
        }
        USART1->DR = (uint8_t)*line++;
    }
}
#endif
void ImuMs901m_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    const ImuMs901mDiagnostics initial = {0};
    imu_ms901m_diag = initial;
    ImuMs901m_ParserInit(&parser);
    uart_errors = 0U;
    started_ms = last_log_ms = HAL_GetTick();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &gpio);
    /* Receive only: PC6 and USART6 transmitter remain unconfigured. */
    USART6->CR1 = USART6->CR2 = USART6->CR3 = 0U;
    USART6->BRR = (HAL_RCC_GetPCLK2Freq() + IMU_MS901M_BAUDRATE / 2U) / IMU_MS901M_BAUDRATE;
    USART6->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    HAL_NVIC_SetPriority(USART6_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

#if IMU_ONLY_TEST_ACTIVE && IMU_DIAGNOSTIC_LOG_ENABLED
    /* Separate diagnostic TX. Vision reserves USART2/PD5/PD6. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9; gpio.Pull = GPIO_NOPULL;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);
    USART1->CR1 = USART1->CR2 = USART1->CR3 = 0U;
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + 57600U) / 115200U;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
    LogLine("F407 MS901M receive-only; MOTOR OFF; PC7 RX; PA9 log; keep still 2s\r\n");
#endif
}
void USART6_IRQHandler(void)
{
    uint32_t status = USART6->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)USART6->DR;
        if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
            uart_errors++;
            parser.length = 0U;
        } else if (status & USART_SR_RXNE) {
            ImuMs901m_Feed(&parser, byte, HAL_GetTick());
        }
    }
}
void ImuMs901m_Service(void)
{
    uint32_t now, primask = __get_PRIMASK();
    ImuMs901mSnapshot data;
    __disable_irq();
    data = parser.data;
    imu_ms901m_diag.uart_errors = uart_errors;
    if (primask == 0U) __enable_irq();
    now = HAL_GetTick();
    imu_ms901m_diag.data = data;
    imu_ms901m_diag.age_ms = data.angle_frames ? (uint32_t)(now - data.last_angle_ms) : UINT32_MAX;
    imu_ms901m_diag.state = !data.angle_frames ? 0U : ImuMs901m_IsFresh(&data, now) ? 1U : 2U;
    if (!imu_ms901m_diag.zero_ready && (uint32_t)(now - started_ms) >= 2000U &&
        data.angle_frames >= 5U && imu_ms901m_diag.state == 1U) {
        imu_ms901m_diag.zero_yaw_tenths = data.yaw_tenths;
        imu_ms901m_diag.zero_ready = 1U;
    }
    if (imu_ms901m_diag.zero_ready) {
        imu_ms901m_diag.relative_yaw_tenths = ImuMs901m_AngleDifference(data.yaw_tenths,
                                                   imu_ms901m_diag.zero_yaw_tenths);
    }
#if IMU_ONLY_TEST_ACTIVE && IMU_DIAGNOSTIC_LOG_ENABLED
    if ((uint32_t)(now - last_log_ms) >= 250U) {
        char line[160];
        last_log_ms = now;
        (void)snprintf(line, sizeof(line),
            "IMU state=%lu bytes=%lu frames=%lu age=%lu yaw10=%ld delta10=%ld zero=%lu err=%lu/%lu\r\n",
            (unsigned long)imu_ms901m_diag.state, (unsigned long)data.raw_bytes,
            (unsigned long)data.angle_frames, (unsigned long)imu_ms901m_diag.age_ms,
            (long)data.yaw_tenths, (long)imu_ms901m_diag.relative_yaw_tenths,
            (unsigned long)imu_ms901m_diag.zero_ready,
            (unsigned long)data.checksum_errors, (unsigned long)imu_ms901m_diag.uart_errors);
        LogLine(line);
    }
#endif
}
