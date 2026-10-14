#ifndef X42S_SAFETY_H
#define X42S_SAFETY_H

#include <stdint.h>

/* Probes one configured X42S ID, after stopping and disabling IDs 1..4.
 * It never writes an ID or commands motion. */
void X42S_ReadOnlyProbeInit(void);

/* Advances the probe and reports a simple result through the PB8 marker. */
void X42S_ReadOnlyProbeService(void);

/* ID3-only stop/disable and read-only version/config/state/speed diagnostics.
 * One command per 500 ms receive window; no enable or motion frame. */
void X42S_Id3StopRecoveryInit(void);
void X42S_Id3StopRecoveryService(void);


#endif /* X42S_SAFETY_H */
