#ifndef SERVO_HAL_H
#define SERVO_HAL_H

#include <stdint.h>
#include <stdbool.h>

void Servo_Init();
void Servo_SetAngle(uint8_t id, uint8_t angle);
void Servo_SetPalseWidth(uint8_t id, uint16_t pw);
void Servo_Angle_Step(void);
void Servo_SetAngle_Step(uint8_t id, uint8_t angle);
void Servo_SyncCurrentToOutput(void);
#endif