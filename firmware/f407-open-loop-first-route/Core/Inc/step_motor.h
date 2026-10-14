#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdint.h>

#define STEP_MOTOR_MAX_REPLY_BYTES 40U

typedef enum
{
    STEP_MOTOR_TX_OK = 0,
    STEP_MOTOR_TX_TXE_TIMEOUT,
    STEP_MOTOR_TX_TC_TIMEOUT,
    STEP_MOTOR_TX_PARAMETER
} StepMotorTxResult;

typedef struct
{
    uint8_t bytes[STEP_MOTOR_MAX_REPLY_BYTES];
    uint8_t length;
    uint8_t overflow;
    uint8_t uart_error;
} StepMotorReply;

/* Adapted from the senior/junior StepMotor driver. The protocol-facing API is
 * retained, while the transport is the verified USART3 PD8/PD9 link. */
void StepMotor_Init(void);
/* Signed targets from the reference, separate from synchronous UART TX. */
void StepMotor_ClearTargets(void);
uint8_t StepMotor_SetTargetSpeed(uint8_t id, int32_t speed_rpm);
uint8_t StepMotor_GetTargetSpeed(uint8_t id, uint8_t *direction, uint16_t *speed_rpm);
/* Junior-style runtime slots. The caller schedules ID1..4 then FF66; these
 * functions omit the commissioning-only 10 ms gap. */
StepMotorTxResult StepMotor_SendTargetFast(uint8_t id);
StepMotorTxResult StepMotor_SyncMoveFast(void);
StepMotorTxResult StepMotor_Enable(uint8_t id, uint8_t enable);
StepMotorTxResult StepMotor_SetSpeed(uint8_t id, uint8_t direction,
                                     uint16_t speed_rpm, uint8_t acceleration);
StepMotorTxResult StepMotor_SetSpeedSync(uint8_t id, uint8_t direction,
                                         uint16_t speed_rpm,
                                         uint8_t acceleration,
                                         uint8_t sync);
StepMotorTxResult StepMotor_SyncMove(uint8_t id);
StepMotorTxResult StepMotor_Stop(uint8_t id);
StepMotorTxResult StepMotor_ReadBusVoltage(uint8_t id);
StepMotorTxResult StepMotor_ReadStatus(uint8_t id);
StepMotorTxResult StepMotor_ReadSpeed(uint8_t id);
StepMotorTxResult StepMotor_ReadConfig(uint8_t id);

void StepMotor_GetReply(StepMotorReply *reply);
uint8_t StepMotor_ReplyIsAck(const StepMotorReply *reply, uint8_t id,
                             uint8_t command, uint8_t *return_code);
uint8_t StepMotor_ParseBusVoltage(const StepMotorReply *reply, uint8_t id,
                                  uint16_t *millivolts);
uint8_t StepMotor_ParseStatus(const StepMotorReply *reply, uint8_t id,
                              uint8_t *status_flags);
uint8_t StepMotor_ParseSpeed(const StepMotorReply *reply, uint8_t id,
                             uint8_t *negative, uint16_t *speed_raw);
uint8_t StepMotor_ParseConfigIdentity(const StepMotorReply *reply, uint8_t id);

#endif /* STEP_MOTOR_H */
