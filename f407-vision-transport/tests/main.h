#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

uint32_t HAL_GetTick(void);
uint32_t test_get_primask(void);
void test_disable_irq(void);
void test_enable_irq(void);

#define __get_PRIMASK() test_get_primask()
#define __disable_irq() test_disable_irq()
#define __enable_irq() test_enable_irq()

#endif
