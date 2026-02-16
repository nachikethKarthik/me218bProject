#ifndef SPI1_COMMHALL_H
#define SPI1_COMMHALL_H

#include <stdint.h>
#include <stdbool.h>
#include "PIC32_SPI_HAL.h"

// init
bool SPI1Leader_Init(void);
bool SPI1Follower_Init(void);

void SPI1Leader_SendCmd16(uint16_t cmd);
uint16_t SPI1Leader_PullData16(void);
uint16_t SPI1Leader_RequestResponse16(uint16_t req);

bool SPI1Follower_TryRead16(uint16_t *out_word);
void SPI1Follower_LoadTx16(uint16_t tx_word);

#endif