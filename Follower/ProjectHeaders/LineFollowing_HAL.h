// LineFollowing_HAL.h
#ifndef LINEFOLLOWING_HAL_H
#define LINEFOLLOWING_HAL_H

#include <stdint.h>

bool Init_LineFollowing(void);
void ReadIRSensors(void);

uint32_t LineFollowing_GetFrontLeft(void);
uint32_t LineFollowing_GetFrontCenter(void);
uint32_t LineFollowing_GetFrontRight(void);
uint32_t LineFollowing_GetBackLeft(void);
uint32_t LineFollowing_GetBackCenter(void);
uint32_t LineFollowing_GetBackRight(void);

void LineFollowing_ResetPD(void);
float LineFollowing_ComputeFrontTurn(void);
float LineFollowing_ComputeBackTurn(void);

bool is_branch_F(void);
bool is_T_F(void);
bool is_branch_B(void);
bool is_T_B(void);
#endif // LineFollowing