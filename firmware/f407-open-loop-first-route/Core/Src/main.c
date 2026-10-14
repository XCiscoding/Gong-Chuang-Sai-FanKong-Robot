/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : ACK-gated chassis test runner
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include "imu_ms901m.h"
#include "step_motor.h"
#include "motor_test.h"
#include "chassis.h"
#include "chassis_heading.h"
#include "vision.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  ImuMs901m_Init();
  ChassisHeading_Init();
#if VISION_LINK_TEST_ACTIVE
  Vision_Init();
#endif
  /* Exclusive application modes: the passive vision-link test suppresses
   * motors; route builds retain the 2 ms ACK-gated motor-slot schedule. */
#if !IMU_ONLY_TEST_ACTIVE && !VISION_LINK_TEST_ACTIVE
  StepMotor_Init();
  Chassis_Init();
  MotorTest_Init();
#endif

  while (1)
  {
    ImuMs901m_Service();
    ChassisHeading_Service();
#if VISION_LINK_TEST_ACTIVE
    Vision_Service();
#endif
#if !IMU_ONLY_TEST_ACTIVE && !VISION_LINK_TEST_ACTIVE
    MotorTest_Service();
#endif
    /* HAL_Delay(1) waits two SysTick increments. During the 2 ms
     * motor slots, poll continuously instead of consuming the reply budget. */
#if !IMU_ONLY_TEST_ACTIVE && !VISION_LINK_TEST_ACTIVE
    if (!MotorTest_NeedsFastService()) HAL_Delay(1U);
#else
    HAL_Delay(1U);
#endif
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef oscillator_config = {0};
  RCC_ClkInitTypeDef clock_config = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  oscillator_config.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  oscillator_config.HSIState = RCC_HSI_ON;
  oscillator_config.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscillator_config.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&oscillator_config) != HAL_OK) Error_Handler();

  clock_config.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clock_config.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clock_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clock_config.APB1CLKDivider = RCC_HCLK_DIV1;
  clock_config.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clock_config, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Keep the external PD14-controlled circuit inactive after startup. */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  gpio.Pin = GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &gpio);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
