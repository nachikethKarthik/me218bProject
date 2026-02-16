#ifndef SPI1_COMMS_HAL_H
#define SPI1_COMMS_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "PIC32_SPI_HAL.h"

bool SPI1Leader_Init(void);
bool SPI1Follower_Init(void);

// Leader -> Follower
void SPI1Leader_SendCmd16(uint16_t cmd);

// Follower -> Leader
uint16_t SPI1Leader_PullData16(void);

// Leader <-> Follower
uint16_t SPI1Leader_RequestResponse16(uint16_t req);

bool SPI1Follower_TryRead16(uint16_t *out_word);
void SPI1Follower_LoadTx16(uint16_t tx_word);

#endif