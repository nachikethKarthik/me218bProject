#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "SPI1_CommHAL.h"

#define SPI_CLK_PERIOD_NS  (800000u)

#ifndef SPISTAT_RBF_MASK
#define SPISTAT_RBF_MASK   (1u << 0)
#endif
#ifndef SPISTAT_ROV_MASK
#define SPISTAT_ROV_MASK   (1u << 6)
#endif

// -------------------- Leader init --------------------
bool SPI1Leader_Init(void)
{
    SPISetup_BasicConfig(SPI_SPI1);
    SPISetup_SetLeader(SPI_SPI1, SPI_SMP_END);
    SPISetup_SetBitTime(SPI_SPI1, SPI_CLK_PERIOD_NS);
    SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_LO);
    SPISetup_SetActiveEdge(SPI_SPI1, SPI_FIRST_EDGE);
    SPISetup_SetXferWidth(SPI_SPI1, SPI_16BIT);
    SPISetEnhancedBuffer(SPI_SPI1, false);
    SPISetup_MapSSOutput(SPI_SPI1, SPI_RPB4);
    SPISetup_MapSDOutput(SPI_SPI1, SPI_RPB13);
    SPISetup_MapSDInput (SPI_SPI1, SPI_RPB11);

    ANSELBCLR = (1u << 14);
    TRISBCLR  = (1u << 14);

    SPISetup_EnableSPI(SPI_SPI1);

    //if (SPI1STAT & SPISTAT_ROV_MASK) SPI1STATCLR = SPISTAT_ROV_MASK;
    //while (SPI1STAT & SPISTAT_RBF_MASK) (void)SPI1BUF;
    return true;
}

// -------------------- Follower init --------------------
bool SPI1Follower_Init(void)
{
    SPISetup_BasicConfig(SPI_SPI1);
    SPISetup_SetFollower(SPI_SPI1);
    SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_LO);
    SPISetup_SetActiveEdge(SPI_SPI1, SPI_FIRST_EDGE);
    SPISetup_SetXferWidth(SPI_SPI1, SPI_16BIT);
    SPISetEnhancedBuffer(SPI_SPI1, false);

    SPISetup_MapSSInput(SPI_SPI1, SPI_RPB4);
    SPISetup_MapSDOutput(SPI_SPI1, SPI_RPA4);
    SPISetup_MapSDInput (SPI_SPI1, SPI_RPB11);

    ANSELBCLR = (1u << 14);
    TRISBSET  = (1u << 14);

    SPI1BUF = 0xAAAA;

    if (!SPISetup_EnableSPI(SPI_SPI1)) return false;

    // clear overflow / stale rx
    //if (SPI1STAT & SPISTAT_ROV_MASK) SPI1STATCLR = SPISTAT_ROV_MASK;
    //while (SPI1STAT & SPISTAT_RBF_MASK) (void)SPI1BUF;
    return true;
}

// Leader APIs
void SPI1Leader_SendCmd16(uint16_t cmd)
{
    SPIOperate_SPI1_Send16Wait(cmd);
    (void)SPI1BUF;
}

uint16_t SPI1Leader_PullData16(void)
{
    SPIOperate_SPI1_Send16Wait(0x0000u);
    return (uint16_t)SPI1BUF;
}

uint16_t SPI1Leader_RequestResponse16(uint16_t req)
{
    SPIOperate_SPI1_Send16Wait(req);
    (void)SPI1BUF;
    SPIOperate_SPI1_Send16Wait(0x0000u);
    return (uint16_t)SPI1BUF;
}

// Follower APIs
bool SPI1Follower_TryRead16(uint16_t *out_word)
{
    if (SPI1STAT & SPISTAT_ROV_MASK) {
        SPI1STATCLR = SPISTAT_ROV_MASK;
    }
    if ((SPI1STAT & SPISTAT_RBF_MASK) == 0u) return false;

    *out_word = (uint16_t)SPI1BUF;
    return true;
}

void SPI1Follower_LoadTx16(uint16_t tx_word)
{
    SPI1BUF = (uint32_t)tx_word;
}