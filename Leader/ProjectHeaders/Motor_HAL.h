// Motor_HAL.h
#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>

void MotorHAL_Init(void);
void MotorHAL_SetSpeedCmdRPM(uint8_t id, uint16_t rpm, _Bool reverse);
uint16_t MotorHAL_GetSpeedMeasRPM(uint8_t id);
int32_t MotorHAL_GetEncoderCount(uint8_t id);
uint8_t MotorHAL_GetDutyOut(uint8_t id);
void MotorHAL_DriveEncoderCount(uint8_t id, uint16_t EncoderCounts);
int32_t MotorHAL_GetStartCount(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_HAL_H