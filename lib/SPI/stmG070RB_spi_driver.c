/*
 * stm32G070RB_spi_driver.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Ashish
 */

#include "stmG070RB_spi_driver.h"

/*******************************************************
 * @fn               - SPI_PeriClockControl
 *
 * @brief            - Enables or disables the peripheral clock for the specified SPI module.
 *
 * @param[in]        - pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3).
 * @param[in]        - EnorDi: Specifies whether to enable or disable the clock.
 *                     - ENABLE: Enables the SPI peripheral clock.
 *                     - DISABLE: Disables the SPI peripheral clock.
 *
 * @return           - None
 *
 * @note             - Ensure the correct SPI peripheral is passed before enabling/disabling the clock.
 *                   - This function is essential before configuring or using the SPI peripheral.
 *******************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
    }
}

/*******************************************************
 * @fn               - SPI_Init
 *
 * @brief            - Initializes the SPI peripheral with the specified configuration.
 *
 * @param[in]        - pSPIHandle: Pointer to the SPI handle structure containing the SPI configuration settings.
 *
 * @return           - None
 *
 * @note             - This function configures various SPI settings such as device mode, bus configuration,
 *                     clock speed, clock polarity and phase, and data size.
 *                   - Ensure that the SPI peripheral clock is enabled before calling this function.
 *                   - The function writes to the SPI_CR1 and SPI_CR2 registers based on the provided configuration.
 *******************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    // Peripheral clock control
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    uint32_t tempreg = 0;

    // 1. Configure the Device Mode (Master/Slave)
    tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

    // 2. Configure the Bus Config
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
    {
        // Clear BIDI Mode
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
    {
        // Set BIDI Mode
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // Clear BIDI Mode
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        // Set RXONLY bit
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Configure SPI Serial Clock Speed (Baud Rate)
    tempreg &= ~(7 << SPI_CR1_BR); // Clear BR[2:0]
    tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    // 4. Configure CPOL and CPHA
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
    
    // 5. Configure SSM (Software Slave Management)
    tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

    // Write the updated value to SPI_CR1 register
    pSPIHandle->pSPIx->CR1 = tempreg;

    // 6. Configure Data Size (DS[3:0]) in SPI_CR2
    pSPIHandle->pSPIx->CR2 &= ~(0xF << SPI_CR2_DS); // Clear DS bits
    pSPIHandle->pSPIx->CR2 |= (pSPIHandle->SPIConfig.SPI_DS << SPI_CR2_DS);
    
    // 7. Set FRXTH (FIFO RX Threshold) for 8-bit mode
    // FRXTH=1 for 8-bit, FRXTH=0 for 16-bit
    if ((pSPIHandle->SPIConfig.SPI_DS == SPI_DS_8BIT) || 
        (pSPIHandle->SPIConfig.SPI_DS < SPI_DS_9BIT))
    {
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_FRXTH);
    }
    else
    {
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_FRXTH);
    }

    // 8. Enable SPI if needed (optional)
    // pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_SPE);
}

/*******************************************************
 * @fn               -
 *
 * @brief            -
 *
 * @param[in]        -
 *
 * @return           -
 *
 * @note             -
 *******************************************************/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if (pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*******************************************************
 * @fn               - SPI_SendData
 *
 * @brief            - Sends data over SPI using blocking mode.
 *
 * @param[in]        - pSPIx: Pointer to the SPI handle structure.
 * @param[in]        - pTxBuffer: Pointer to the data buffer to be transmitted.
 * @param[in]        - Len: Length of data in bytes.
 *
 * @return           - None
 *
 * @note             - This function waits until the TXE (Transmit Buffer Empty) flag is set before sending data.
 *                   - It supports both 8-bit and 16-bit data frame formats.
 *                   - The function waits until the SPI is no longer busy before returning.
 *                   - Ensure SPI is properly initialized before calling this function.
 *******************************************************/

void SPI_SendData(SPI_Handle_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. Wait until TXE (Transmit Buffer Empty) is set
        while (SPI_GetFlagStatus(pSPIx->pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
            ;

        // 2. Check Data Frame Size (DS[3:0])
        if ((pSPIx->pSPIx->CR2 & (0xF << SPI_CR2_DS)) == (SPI_DS_16BIT << SPI_CR2_DS))
        {
            // 16-bit mode: Load two bytes at a time
            *((__vo uint16_t *)&pSPIx->pSPIx->DR) = *((uint16_t *)pTxBuffer);
            Len -= 2;
            pTxBuffer += 2;
        }
        else
        {
            // 8-bit mode: Load one byte at a time
            *((__vo uint8_t *)&pSPIx->pSPIx->DR) = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }

    // 3. Wait until SPI is no longer busy
    while (pSPIx->pSPIx->SR & SPI_BSY_FLAG)
        ;
}

/*******************************************************
 * @fn               - SPI_PeripheralControl
 *
 * @brief            - Enables or disables the SPI peripheral.
 *
 * @param[in]        - pSPIx: Pointer to the SPI peripheral base address.
 * @param[in]        - EnOrDi: ENABLE to enable SPI, DISABLE to disable SPI.
 *
 * @return           - None
 *
 * @note             - This function controls the SPE (SPI Enable) bit in the SPI_CR1 register.
 *                   - SPI must be enabled before communication can take place.
 *******************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/*******************************************************
 * @fn               - SPI_SSIConfig
 *
 * @brief            - Configures the SSI (Software Slave Management) bit in SPI.
 *
 * @param[in]        - pSPIx: Pointer to the SPI peripheral base address.
 * @param[in]        - EnOrDi: ENABLE to set SSI, DISABLE to clear SSI.
 *
 * @return           - None
 *
 * @note             - This function is used in software slave management mode.
 *                   - Setting the SSI bit prevents the MODF (Mode Fault) error when SPI is in master mode.
 *******************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}
/*******************************************************
 * @fn               - SPI_ReceiveData
 *
 * @brief            - Receives data over SPI using blocking mode.
 *
 * @param[in]        - pSPIx: Pointer to the SPI peripheral base address.
 * @param[in]        - pRxBuffer: Pointer to the buffer to store received data.
 * @param[in]        - Len: Length of data to receive in bytes.
 *
 * @return           - None
 *
 * @note             - This function waits until the RXNE (Receive Buffer Not Empty) flag is set before reading data.
 *******************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. Wait until RXNE is set
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
            ;

        // 2. Check Data Frame Size (DS[3:0])
        if ((pSPIx->CR2 & (0xF << SPI_CR2_DS)) == (SPI_DS_16BIT << SPI_CR2_DS))
        {
            // 16-bit mode
            *((uint16_t *)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            pRxBuffer += 2;
        }
        else
        {
            // 8-bit mode
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}
