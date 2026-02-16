#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "SPI1_CommHAL.h"

#define SPI_CLK_PERIOD_NS  1000u

#ifndef SPISTAT_RBF_MASK
#define SPISTAT_RBF_MASK   (1u << 0)
#endif
#ifndef SPISTAT_TBF_MASK
#define SPISTAT_TBF_MASK   (1u << 1)
#endif
#ifndef SPISTAT_ROV_MASK
#define SPISTAT_ROV_MASK   (1u << 6)
#endif

static uint16_t SPI1_Xfer16(uint16_t tx)
{
    // clear overflow
    if (SPI1STAT & SPISTAT_ROV_MASK) {
        SPI1STATCLR = SPISTAT_ROV_MASK;
    }

    while (SPI1STAT & SPISTAT_RBF_MASK) {
        (void)SPI1BUF;
    }

    while (SPI1STAT & SPISTAT_TBF_MASK) {
        ;
    }

    SPI1BUF = tx;

    // wait RX full
    while ((SPI1STAT & SPISTAT_RBF_MASK) == 0u) {
        ;
    }

    return (uint16_t)SPI1BUF;
}


// Leader pins:   SS=RB4,  SDO=RB13, SDI=RB11
// Follower pins: SS=RB4,  SDO=RA4,  SDI=RB11
bool SPI1Leader_Init(void)
{
    if (!SPISetup_BasicConfig(SPI_SPI1)) return false;
    if (!SPISetup_SetLeader(SPI_SPI1, SPI_SMP_END)) return false;
    if (!SPISetup_SetBitTime(SPI_SPI1, SPI_CLK_PERIOD_NS)) return false;
    if (!SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_LO)) return false;
    if (!SPISetup_SetActiveEdge(SPI_SPI1, SPI_FIRST_EDGE)) return false;
    if (!SPISetup_SetXferWidth(SPI_SPI1, SPI_16BIT)) return false;
    if (!SPISetEnhancedBuffer(SPI_SPI1, true)) return false;

    // PPS mapping
    if (!SPISetup_MapSSOutput(SPI_SPI1, SPI_RPB4))  return false;
    if (!SPISetup_MapSDOutput(SPI_SPI1, SPI_RPB13)) return false;
    if (!SPISetup_MapSDInput (SPI_SPI1, SPI_RPB11)) return false;

    if (!SPISetup_EnableSPI(SPI_SPI1)) return false;

    return true;
}

bool SPI1Follower_Init(void)
{
    if (!SPISetup_BasicConfig(SPI_SPI1)) return false;
    if (!SPISetup_SetFollower(SPI_SPI1)) return false;

    if (!SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_LO)) return false;
    if (!SPISetup_SetActiveEdge(SPI_SPI1, SPI_FIRST_EDGE)) return false;

    if (!SPISetup_SetXferWidth(SPI_SPI1, SPI_16BIT)) return false;
    if (!SPISetEnhancedBuffer(SPI_SPI1, true)) return false;

    // Mapping
    if (!SPISetup_MapSSInput(SPI_SPI1, SPI_RPB4))  return false;
    if (!SPISetup_MapSDOutput(SPI_SPI1, SPI_RPA4)) return false;
    if (!SPISetup_MapSDInput (SPI_SPI1, SPI_RPB11)) return false;

    SPI1BUF = 0xAAAA;

    if (!SPISetup_EnableSPI(SPI_SPI1)) return false;
    return true;
}

// Leader APIs
void SPI1Leader_SendCmd16(uint16_t cmd)
{
    (void)SPI1_Xfer16(cmd);
}

uint16_t SPI1Leader_PullData16(void)
{
    return SPI1_Xfer16(0x0000u);
}

uint16_t SPI1Leader_RequestResponse16(uint16_t req)
{
    (void)SPI1_Xfer16(req);
    return SPI1_Xfer16(0x0000u);
}

// Follower APIs

bool SPI1Follower_TryRead16(uint16_t *out_word)
{
    if ((SPI1STAT & SPISTAT_RBF_MASK) == 0u) return false;
    *out_word = (uint16_t)SPI1BUF;
    return true;
}

void SPI1Follower_LoadTx16(uint16_t tx_word)
{
    SPI1BUF = (uint32_t)tx_word;
}
