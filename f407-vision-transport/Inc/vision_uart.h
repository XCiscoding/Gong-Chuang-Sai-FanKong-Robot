#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdint.h>

/* F407 <-> K230D: USART2, PD5 TX / PD6 RX, 115200 8N1, 3.3 V logic. */
#define VISION_UART_BAUDRATE 115200U

typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_overflow;
    uint32_t uart_errors;
    uint32_t tx_busy;
} VisionUartDiagnostics;

void VisionUart_Init(void);
void VisionUart_IRQHandler(void);
void VisionUart_Service(void);
uint8_t VisionUart_ReadByte(uint8_t *byte);
uint8_t VisionUart_Send(const char *text);
void VisionUart_GetDiagnostics(VisionUartDiagnostics *diagnostics);

#endif /* VISION_UART_H */
