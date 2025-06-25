/*
 * stm32G070RB_spi_driver.h
 *
 *  Created on: Feb 27, 2025
 *      Author: Ashish
 */

#ifndef INC_STMG070RB_SPI_DRIVER_H_
#define INC_STMG070RB_SPI_DRIVER_H_

#include "stmG070RB.h"

/*
 * Configuration structure for SPIx
 */
typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DS;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t;

/*
 *Handle sturcture for SPIx peripheral
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPIConfig;
} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER   1
#define SPI_DEVICE_MODE_SLAVE    0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FULL_DUPLEX      1 
#define SPI_BUS_CONFIG_HALF_DUPLEX      2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2            0
#define SPI_SCLK_SPEED_DIV4            1
#define SPI_SCLK_SPEED_DIV8            2
#define SPI_SCLK_SPEED_DIV16           3
#define SPI_SCLK_SPEED_DIV32           4
#define SPI_SCLK_SPEED_DIV64           5
#define SPI_SCLK_SPEED_DIV128          6
#define SPI_SCLK_SPEED_DIV256          7

/*
 * @SPI_DS (Data Size)
 * Defines the number of bits in a SPI frame
 */
#define SPI_DS_4BIT      0x3  // 4-bit data size
#define SPI_DS_5BIT      0x4  // 5-bit data size
#define SPI_DS_6BIT      0x5  // 6-bit data size
#define SPI_DS_7BIT      0x6  // 7-bit data size
#define SPI_DS_8BIT      0x7  // 8-bit data size
#define SPI_DS_9BIT      0x8  // 9-bit data size
#define SPI_DS_10BIT     0x9  // 10-bit data size
#define SPI_DS_11BIT     0xA  // 11-bit data size
#define SPI_DS_12BIT     0xB  // 12-bit data size
#define SPI_DS_13BIT     0xC  // 13-bit data size
#define SPI_DS_14BIT     0xD  // 14-bit data size
#define SPI_DS_15BIT     0xE  // 15-bit data size
#define SPI_DS_16BIT     0xF  // 16-bit data size


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW         0
#define SPI_CPOL_HIGH        1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW         0
#define SPI_CPHA_HIGH        1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI           0
#define SPI_SSM_EN           1

/*
 * SPI related status flags definitions
 */

#define SPI_TXE_FLAG     (1 << SPI_SR_TXE)     // Transmit buffer empty
#define SPI_RXNE_FLAG    (1 << SPI_SR_RXNE)    // Receive buffer not empty
#define SPI_BSY_FLAG     (1 << SPI_SR_BSY)     // SPI busy flag
#define SPI_OVR_FLAG     (1 << SPI_SR_OVR)     // Overrun error flag
#define SPI_MODF_FLAG    (1 << SPI_SR_MODF)    // Mode fault flag
#define SPI_CRCERR_FLAG  (1 << SPI_SR_CRCERR)  // CRC error flag
#define SPI_FRE_FLAG     (1 << SPI_SR_FRE)     // Frame format error flag
#define SPI_UDR_FLAG     (1 << SPI_SR_UDR)     // Underrun error flag (only in slave mode)


/**************************************************************
 *  APIs supported by this driver
 **************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data read and write
 */
void SPI_SendData(SPI_Handle_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIX, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIX, uint8_t EnOrDi);

#endif /* INC_STMG070RB_SPI_DRIVER_H_ */
