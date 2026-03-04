// Flywheel_HAL.h
#ifndef FLYWHEEL_HAL_H
#define FLYWHEEL_HAL_H

#include <stdint.h>

void Flywheel_Init(void);
void Flywheel_SetDuty(uint8_t duty_percent);
void Flywheel_SetDuty_Counter(uint8_t duty_percent);
#endif // FLYWHEEL_HAL