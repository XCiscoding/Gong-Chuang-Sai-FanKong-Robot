#include "board_bringup.h"
#include "main.h"

#define BOARD_MARKER_PORT GPIOB
#define BOARD_MARKER_PIN  GPIO_PIN_8

void BoardBringup_TestSignalLow(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(BOARD_MARKER_PORT, BOARD_MARKER_PIN, GPIO_PIN_RESET);

  gpio_init.Pin = BOARD_MARKER_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_MARKER_PORT, &gpio_init);
  HAL_GPIO_WritePin(BOARD_MARKER_PORT, BOARD_MARKER_PIN, GPIO_PIN_RESET);
}

void BoardBringup_SetMarker(uint8_t asserted)
{
  HAL_GPIO_WritePin(BOARD_MARKER_PORT, BOARD_MARKER_PIN,
                    (asserted != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
