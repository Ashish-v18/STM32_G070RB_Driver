#include "stmG070RB.h"
#include <string.h>

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCK
 * PB12 --> SPI2_NSS
 */

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPubPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
    SPI_Handle_t SPI2handle;

    SPI2handle.pSPIx = SPI2;
    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256; // SLOWEST: ~62.5kHz to 250kHz depending on clock
    SPI2handle.SPIConfig.SPI_DS = SPI_DS_8BIT;
    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // Software Slave Management

    SPI_Init(&SPI2handle);
}

int main(void)
{
    char user_data[] = "Hello World";

    // Initialize SPI GPIO pins
    SPI2_GPIOInits();

    // Initialize PB12 manually as a GPIO output for NSS toggling
    GPIO_Handle_t NSSPin;
    NSSPin.pGPIOx = GPIOB;
    NSSPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    NSSPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    NSSPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    NSSPin.GPIO_PinConfig.GPIO_PinPubPdControl = GPIO_NO_PUPD;
    NSSPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&NSSPin);
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_HIGH); // Keep High by default

    // Initialize SPI2 peripheral
    SPI2_Inits();

    // Setting SSI to high avoids MODF error in Master mode
    SPI_SSIConfig(SPI2, ENABLE);

    // Enable the SPI2 peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

    SPI_Handle_t SPI2Handle;
    SPI2Handle.pSPIx = SPI2;

    while (1)
    {
        // Drop NSS to LOW to start transmission
        GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_LOW);

        // Send data
        SPI_SendData(&SPI2Handle, (uint8_t *)user_data, strlen(user_data));

        // Pull NSS to HIGH to end transmission
        GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_HIGH);

        // Longer delay between transmissions for clear visuals (approx 1-2 seconds)
        for (volatile uint32_t i = 0; i < 2000000; i++);
    }
}
