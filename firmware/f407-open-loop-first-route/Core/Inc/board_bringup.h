#ifndef BOARD_BRINGUP_H
#define BOARD_BRINGUP_H

#include <stdint.h>

/* PB8 is an external active-low test signal, not an assumed board LED. */
void BoardBringup_TestSignalLow(void);
void BoardBringup_SetMarker(uint8_t asserted);

#endif /* BOARD_BRINGUP_H */
