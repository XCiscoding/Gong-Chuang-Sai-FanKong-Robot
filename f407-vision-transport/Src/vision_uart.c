#include "vision_uart.h"
#include "main.h"

#define VISION_UART_RX_BUFFER_SIZE 128U
#define VISION_UART_TX_BUFFER_SIZE 48U

static volatile uint8_t rx_buffer[VISION_UART_RX_BUFFER_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint32_t rx_bytes;
static volatile uint32_t rx_overflow;
static volatile uint32_t uart_errors;
static volatile uint8_t tx_buffer[VISION_UART_TX_BUFFER_SIZE];
static volatile uint8_t tx_length;
static volatile uint8_t tx_position;
static volatile uint32_t tx_busy;

static void PushReceivedByte(uint8_t byte)
{
    uint8_t next = (uint8_t)((rx_head + 1U) % VISION_UART_RX_BUFFER_SIZE);
    if (next == rx_tail) {
        rx_overflow++;
        return;
    }
    rx_buffer[rx_head] = byte;
    rx_head = next;
    rx_bytes++;
}

void VisionUart_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    rx_head = rx_tail = 0U;
    rx_bytes = rx_overflow = uart_errors = tx_busy = 0U;
    tx_length = tx_position = 0U;

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &gpio);

    USART2->CR1 = USART2->CR2 = USART2->CR3 = 0U;
    USART2->BRR = (HAL_RCC_GetPCLK1Freq() + VISION_UART_BAUDRATE / 2U) /
                  VISION_UART_BAUDRATE;
    USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
    HAL_NVIC_SetPriority(USART2_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void VisionUart_IRQHandler(void)
{
    uint32_t status = USART2->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)USART2->DR;
        if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
            uart_errors++;
        else
            PushReceivedByte(byte);
    }
    if ((status & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
        if (tx_position < tx_length)
            USART2->DR = tx_buffer[tx_position++];
        else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
            tx_length = tx_position = 0U;
        }
    }
}

/* The startup file dispatches the STM32 USART2 vector by this exact name.
 * Keep the hardware interrupt entry here; the public driver API remains
 * VisionUart_IRQHandler() for the vision module. */
void USART2_IRQHandler(void)
{
    VisionUart_IRQHandler();
}

void VisionUart_Service(void)
{
    /* TX and RX are interrupt driven; this keeps the driver API symmetric. */
}

uint8_t VisionUart_ReadByte(uint8_t *byte)
{
    uint8_t result = 0U;
    uint32_t primask;
    if (byte == 0) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    if (rx_tail != rx_head) {
        *byte = rx_buffer[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1U) % VISION_UART_RX_BUFFER_SIZE);
        result = 1U;
    }
    if (primask == 0U) __enable_irq();
    return result;
}

uint8_t VisionUart_Send(const char *text)
{
    uint8_t length = 0U;
    uint32_t primask;
    if (text == 0) return 0U;
    while (text[length] != '\0') {
        if (length >= (VISION_UART_TX_BUFFER_SIZE - 1U)) return 0U;
        length++;
    }
    if (length == 0U) return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if (tx_length != 0U) {
        tx_busy++;
        if (primask == 0U) __enable_irq();
        return 0U;
    }
    for (uint8_t i = 0U; i < length; i++) tx_buffer[i] = (uint8_t)text[i];
    tx_length = length;
    tx_position = 0U;
    USART2->CR1 |= USART_CR1_TXEIE;
    if (primask == 0U) __enable_irq();
    return 1U;
}

void VisionUart_GetDiagnostics(VisionUartDiagnostics *diagnostics)
{
    uint32_t primask;
    if (diagnostics == 0) return;
    primask = __get_PRIMASK();
    __disable_irq();
    diagnostics->rx_bytes = rx_bytes;
    diagnostics->rx_overflow = rx_overflow;
    diagnostics->uart_errors = uart_errors;
    diagnostics->tx_busy = tx_busy;
    if (primask == 0U) __enable_irq();
}
