/*
 * stmG070RB.h
 *
 *  Created on: Sep 27, 2024
 *      Author: AshishKumar
 */

#ifndef INC_STM32G0XX_H_
#define INC_STM32G0XX_H_

#include <stdint.h>

#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER ((__vo uint32_t *)0xE000E100)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER ((__vo uint32_t *)0xE000E180)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR ((__vo uint32_t *)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED 4

/*
    Base address of SRAM and Flash
*/
#define FLASH_BASEADDR            0x08000000U
#define SRAM_BASEADDR             0x20000000U
#define ROM_BASEADDR              0x1FFF0000U
#define SRAM                      SRAM1_BASEADDR
#define IOPORT_BASE               0x50000000U

/*
    AHB and APB Bus peripheral Base Address
*/
#define PERIPH_BASEADDR          0x40000000U
#define APBPERIPH_BASEADDR       PERIPH_BASEADDR
#define AHBPERIPH_BASEADDR       (PERIPH_BASE + 0x00020000U)

/*
    Base Addresses of peripherals which are hanging on APB bus
*/
#define I2C1_BASEADDR            0x40005400U
#define I2C2_BASEADDR            0x40005800U
#define USART2_BASEADDR          0x40004400U
#define USART3_BASEADDR          0x40004800U
#define SPI2_BASEADDR            0x40003800U
#define SPI3_BASEADDR            0x40003C00U
#define TIM3_BASEADDR            0x40000400U
#define TIM6_BASEADDR            0x40001000U
#define TIM7_BASEADDR            0x40001400U
#define RTC_BASEADDR             0x40002800U
#define IWDG_BASEADDR            0x40003000U
#define WWDG_BASEADDR            0x40002C00U

/*
    Base Addresses of peripherals which are hanging on APB2 bus
*/
#define SPI1_BASEADDR            0x40013000U
#define ADC1_BASEADDR            0x40012400U
#define TIM1_BASEADDR            0x40012C00U
#define TIM15_BASEADDR           0x40014000U
#define TIM16_BASEADDR           0x40014400U
#define TIM17_BASEADDR           0x40014800U
#define USART1_BASEADDR          0x40013800U
#define EXTI_BASEADDR            0x40021800U
#define SYSCFG_BASEADDR          0x40010000U

/*
    Base Addresses of peripherals which are hanging on AHB bus
*/
#define GPIOA_BASEADDR           0x50000000U
#define GPIOB_BASEADDR           0x50000400U
#define GPIOC_BASEADDR           0x50000800U
#define GPIOD_BASEADDR           0x50000C00U
#define GPIOE_BASEADDR           0x50001000U
#define GPIOF_BASEADDR           0x50001400U
#define RCC_BASEADDR             0x40021000U
#define CRC_BASEADDR             0x40023000U
#define DMA1_BASEADDR            0x40020000U
#define DMA2_BASEADDR            0x40020400U

/****************  Peripheral register definition structures  ****************/

/*
 *   Peripheral register definition structure for GPIO
 */
typedef struct
{
    __vo uint32_t MODER;   /*| GPIO port mode register                       Address offset:0x00 |*/
    __vo uint32_t OTYPER;  /*| GPIO port output type register                Address offset:0x04 |*/
    __vo uint32_t OSPEEDR; /*| GPIO port output speed register               Address offset:0x08 |*/
    __vo uint32_t PUPDR;   /*| GPIO port pull-up/pull-down registor          Address offset:0x0C |*/
    __vo uint32_t IDR;     /*| GPIO port input data register                 Address offset:0x10 |*/
    __vo uint32_t ODR;     /*| GPIO port output data register                Address offset:0x14 |*/
    __vo uint32_t BSSR;    /*| GPIO bit set/reset register                   Address offset:0x18 |*/
    __vo uint32_t LCKR;    /*| GPIO port configuration lock register         Address offset:0x1C |*/
    __vo uint32_t AFR[2];  /*| AFR[0] GPIO alternate function low register,    AFR[1] GPIO alternate function high register       Address offset:0x20-0x24 |*/
    __vo uint32_t BRR;     /*| GPIO port bit reset register                  Address offset:0x28 |*/

} GPIO_RegDef_t;

/*
 *   Peripheral register definition structure for RCC
 */
typedef struct
{
    __vo uint32_t CR;        /*| Clock control register                                     Address offset:0x00  |*/
    __vo uint32_t ICSCR;     /*| Internal clock source calibration register                 Address offset:0x04  |*/
    __vo uint32_t CFGR;      /*| Clock configuration register                               Address offset:0x08  |*/
    __vo uint32_t PLLCFGR;   /*| PLL configuration register                                 Address offset:0x0C  |*/
    uint32_t RESERVED1;      /*|  |*/
    uint32_t RESERVED2;      /*|  |*/
    __vo uint32_t CIER;      /*| Clodk interrupt enable register                            Address offset:0x18  |*/
    __vo uint32_t CIFR;      /*| Clock interrupt flag register                              Address offset:0x1C  |*/
    __vo uint32_t CICR;      /*| Clock interrupt clear register                             Address offset:0x20  |*/
    __vo uint32_t IOPRSTR;   /*| I/O port reset register                                    Address offset:0x24  |*/
    __vo uint32_t AHBRSTR;   /*| AHB peripheral reset register                              Address offset:0x28  |*/
    __vo uint32_t APBRSTR1;  /*| APB peripheral reset register 1                            Address offset:0x2C  |*/
    __vo uint32_t APBRSTR2;  /*| APB peripheral reset register 2                            Address offset:0x30  |*/
    __vo uint32_t IOPENR;    /*| I/O port clock enable register                             Address offset:0x34  |*/
    __vo uint32_t AHBENR;    /*| AHB peripheral clock enable register                       Address offset:0x38  |*/
    __vo uint32_t APBENR1;   /*| APB peripheral clock enable register 1                     Address offset:0x3C  |*/
    __vo uint32_t APBENR2;   /*| APB peripheral clock enable register 2                     Address offset:0x40  |*/
    __vo uint32_t IOPSMENR;  /*| I/O port in sleep mode clock enable register               Address offset:0x44  |*/
    __vo uint32_t AHBSMENR;  /*| AHB peripheral clock enable in sleep/stop mode register    Address offset:0x48  |*/
    __vo uint32_t APBSMENR1; /*| APB peripheral clock enable in sleep/stop mode register 1  Address offset:0x4C  |*/
    __vo uint32_t APBSMENR2; /*| APB peripheral clock enable in sleep/stop mode register 2  Address offset:0x50  |*/
    __vo uint32_t CCIPR;     /*| Peripheral independent clock configuration register        Address offset:0x54  |*/
    __vo uint32_t CCCIPR2;   /*| Peripheral independent clock configuration register 2      Address offset:0x58  |*/
    __vo uint32_t BDCR;      /*| RTC domain control register                                Address offset:0x5C  |*/
    __vo uint32_t CSR;       /*| Contorl/status register                                    Address offset:0x60  |*/

} RCC_RegDef_t;

/*
 *   Peripheral register definition structure for EXTI
 */

typedef struct
{

    __vo uint32_t RTSR1;     /*| EXTI Rising Trigger Selection Register 1,        Address offset:   0x00 |*/
    __vo uint32_t FTSR1;     /*| EXTI Falling Trigger Selection Register 1,       Address offset:   0x04 |*/
    __vo uint32_t SWIER1;    /*| EXTI Software Interrupt event Register 1,        Address offset:   0x08 |*/
    __vo uint32_t RPR1;      /*| EXTI Rising Pending Register 1,                  Address offset:   0x0C |*/
    __vo uint32_t FPR1;      /*| EXTI Falling Pending Register 1,                 Address offset:   0x10 |*/
    uint32_t RESERVED1[3];   /*| Reserved 1,                                                0x14 -- 0x1C |*/
    uint32_t RESERVED2[5];   /*| Reserved 2,                                                0x20 -- 0x30 |*/
    uint32_t RESERVED3[11];  /*| Reserved 3,                                                0x34 -- 0x5C |*/
    __vo uint32_t EXTICR[4]; /*| EXTI External Interrupt Configuration Register,            0x60 -- 0x6C |*/
    uint32_t RESERVED4[4];   /*| Reserved 4,                                                0x70 -- 0x7C |*/
    __vo uint32_t IMR1;      /*| EXTI Interrupt Mask Register 1,                  Address offset:   0x80 |*/
    __vo uint32_t EMR1;      /*| EXTI Event Mask Register 1,                      Address offset:   0x84 |*/

} EXTI_RegDef_t;

/*
 *   Peripheral register definition structure for SYSCFG
 */
typedef struct
{
    __vo uint32_t CFGR1;          /*| SYSCFG configuration register 1,                   Address offset: 0x00 |*/
    uint32_t RESERVED0[5];        /*| Reserved,                                                   0x04 --0x14 |*/
    __vo uint32_t CFGR2;          /*| SYSCFG configuration register 2,                   Address offset: 0x18 |*/
    uint32_t RESERVED1[25];       /*| Reserved                                                           0x1C |*/
    __vo uint32_t IT_LINE_SR[32]; /*| SYSCFG configuration IT_LINE register,             Address offset: 0x80 |*/
} SYSCFG_RegDef_t;


/*
 * Peripheral register definition sturcture for SPI
 */

 typedef struct 
 {
    __vo uint32_t CR1;       /*| SPI control register 1,            Address offset: 0x00 |*/
    __vo uint32_t CR2;       /*| SPI control register 2,            Address offset: 0x04 |*/
    __vo uint32_t SR;        /*| SPI status register,               Address offset: 0x08 |*/
    __vo uint32_t DR;        /*| SPI data register,                 Address offset: 0x0C |*/
    __vo uint32_t CRCPR;     /*| SPI CRC polynomial register,       Address offset: 0x10 |*/
    __vo uint32_t RXCRCR;    /*| SPI Rx CRC register,               Address offset: 0x14 |*/
    __vo uint32_t TXCRCR;    /*| SPI Tx CRC register,               Address offset: 0x18 |*/
    __vo uint32_t I2SCFGR;   /*| SPIx_I2S configuration register,   Address offset: 0x1C |*/
    __vo uint32_t I2SPR;     /*| SPIxI2S prescaler resgister,       Address offset: 0x20 |*/
 }SPI_RegDef_t;
 


/*
 *   Peripheral definitions ( Peripheral base addresses typtecasted to xxx_RegDef_t)
 */
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)

#define RCC   ((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI  ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1   ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2   ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI3   ((SPI_RegDef_t *)SPI1_BASEADDR)

/*
 *   Clock Enable Macros for GPIOx peripheral
 */
#define GPIOA_PCLK_EN() (RCC->IOPENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->IOPENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->IOPENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->IOPENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->IOPENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->IOPENR |= (1 << 5))

/*
 *   Clock Enable Macros for I2Cx peripheral
 */
#define I2C1_PCLK_EN() (RCC->APBENR1 |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APBENR1 |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APBENR1 |= (1 << 23))

/*
 *   Clock Enable Macros for SPIx peripheral
 */
#define SPI1_PCLK_EN() (RCC->APBENR2 |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APBENR1 |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APBENR1 |= (1 << 15))

/*
 *   Clock Enable Macros for USARTx peripheral
 */
#define USART1_PCLK_EN() (RCC->APBENR2 |= (1 << 14))
#define USART2_PCLK_EN() (RCC->APBENR1 |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APBENR1 |= (1 << 19))
#define USART4_PCLK_EN() (RCC->APBENR1 |= (1 << 8))
#define USART5_PCLK_EN() (RCC->APBENR1 |= (1 << 9))
#define USART6_PCLK_EN() (RCC->APBENR1 |= (1 << 10))

/*
 *   Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APBENR2 |= (1 << 0))

/*
 *   Clock Disable GPIOx Macros for GPIOx peripheral
 */
#define GPIOA_PCLK_DI() (RCC->IOPENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->IOPENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->IOPENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->IOPENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->IOPENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->IOPENR &= ~(1 << 5))

/*
 *   Clock Enable Macros for SPIx peripheral
 */
#define SPI1_PCLK_DI() (RCC->APBENR2 &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APBENR1 &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APBENR1 &= ~(1 << 15))

/*
 *    Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()            \
    do                               \
    {                                \
        (RCC->IOPRSTR |= (1 << 0));  \
        (RCC->IOPRSTR &= ~(1 << 0)); \
    } while (0)
#define GPIOB_REG_RESET()            \
    do                               \
    {                                \
        (RCC->IOPRSTR |= (1 << 1));  \
        (RCC->IOPRSTR &= ~(1 << 1)); \
    } while (0)
#define GPIOC_REG_RESET()            \
    do                               \
    {                                \
        (RCC->IOPRSTR |= (1 << 2));  \
        (RCC->IOPRSTR &= ~(1 << 2)); \
    } while (0)
#define GPIOD_REG_RESET()            \
    do                               \
    {                                \
        (RCC->IOPRSTR |= (1 << 3));  \
        (RCC->IOPRSTR &= ~(1 << 3)); \
    } while (0)
#define GPIOE_REG_RESET()            \
    do                               \
    {                                \
        (RCC->IOPRSTR |= (1 << 4));  \
        (RCC->IOPRSTR &= ~(1 << 4)); \
    } while (0)
#define GPIOF_REG_RESET()            \
    do                               \
    {                                \
        (RCC->IOPRSTR |= (1 << 4));  \
        (RCC->IOPRSTR &= ~(1 << 4)); \
    } while (0)

/*
 * Returns port code for given GPIOx base address
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : (x == GPIOB) ? 1 \
                                                 : (x == GPIOA)   ? 2 \
                                                 : (x == GPIOC)   ? 3 \
                                                 : (x == GPIOD)   ? 4 \
                                                 : (x == GPIOE)   ? 5 \
                                                 : (x == GPIOF)   ? 6 \
                                                                  : 0)

/*
 * IRQ(Interrupt Request) Numbers of STM32G07x MCU
 */
#define IRQ_NO_EXTI0_1       5      // EXTI Line 0 and 1
#define IRQ_NO_EXTI2_3       6      // EXTI Line 2 and 3
#define IRQ_NO_EXTI4_15      7      // EXTI Line 4 to 15
#define IRQ_NO_SPI1          25     // SPI1 global interrupt
#define IRQ_NO_SPI2          26     // SPI2 global interrupt
#define IRQ_NO_I2C1          23     // I2C1 event and error
#define IRQ_NO_USART1        27     // USART1 global interrupt
#define IRQ_NO_USART2        28     // USART2 global interrupts
#define IRQ_NO_TIM1_BRK      24     // TIM1 Break, update, trigger
#define IRQ_NO_TIM2          15     // TIM2 global interrupt
#define IRQ_NO_TIM3          16     // TIM3 global interrupt


/***************************************************
 * General Macros for Peripherals and GPIOs
 **************************************************/
#define ENABLE             1     // Enable a peripheral
#define DISABLE            0     // Disable a peripheral

#define HIGH               1     // Logical HIGH state
#define LOW                0     // Logical LOW state

#define SET                HIGH  // Use for setting bits or flags
#define RESET              LOW   // Use for resetting bits or flags

#define GPIO_PIN_HIGH      HIGH  // GPIO High state
#define GPIO_PIN_LOW       LOW   // GPIO Low state

#define FLAG_SET           HIGH  // Use for checking SPI/UART/I2C flags
#define FLAG_RESET         LOW   // Use for clearing SPI/UART/I2C flags




/********************************************
 * Bit position definitions of SPI peripheral
 ********************************************/

/* SPI_CR1 register bit positions */
#define SPI_CR1_CPHA        0  // Clock Phase
#define SPI_CR1_CPOL        1  // Clock Polarity
#define SPI_CR1_MSTR        2  // Master Selection
#define SPI_CR1_BR          3  // Baud Rate Control (BR[2:0])
#define SPI_CR1_SPE         6  // SPI Enable
#define SPI_CR1_LSBFIRST    7  // Frame format (0: MSB first, 1: LSB first)
#define SPI_CR1_SSI         8  // Internal Slave Select
#define SPI_CR1_SSM         9  // Software Slave Management
#define SPI_CR1_RXONLY      10 // Receive Only Mode
#define SPI_CR1_CRCL        11 // CRC Length (0: 8-bit, 1: 16-bit)
#define SPI_CR1_CRCNEXT     12 // Transmit CRC Next
#define SPI_CR1_CRCEN       13 // Hardware CRC Calculation Enable
#define SPI_CR1_BIDIOE      14 // Output Enable in Bidirectional Mode
#define SPI_CR1_BIDIMODE    15 // Bidirectional Data Mode Enable

/* SPI_CR2 register bit positions */
#define SPI_CR2_RXDMAEN     0  // Rx Buffer DMA Enable
#define SPI_CR2_TXDMAEN     1  // Tx Buffer DMA Enable
#define SPI_CR2_SSOE        2  // SS Output Enable
#define SPI_CR2_NSSP        3  // NSS Pulse Management
#define SPI_CR2_FRF         4  // Frame Format (0: SPI, 1: TI mode)
#define SPI_CR2_ERRIE       5  // Error Interrupt Enable
#define SPI_CR2_RXNEIE      6  // RX Buffer Not Empty Interrupt Enable
#define SPI_CR2_TXEIE       7  // TX Buffer Empty Interrupt Enable
#define SPI_CR2_DS          8  // Data Size (DS[3:0], bits 11:8)
#define SPI_CR2_FRXTH       12 // FIFO Reception Threshold
#define SPI_CR2_LDMARX      13 // Last DMA Transfer for Reception
#define SPI_CR2_LDMATX      14 // Last DMA Transfer for Transmission

/* SPI_SR (Status Register) bit positions */
#define SPI_SR_RXNE         0  // Receive Buffer Not Empty
#define SPI_SR_TXE          1  // Transmit Buffer Empty
#define SPI_SR_CHSIDE       2  // Channel Side (used in I2S mode)
#define SPI_SR_UDR          3  // Underrun Flag
#define SPI_SR_CRCERR       4  // CRC Error Flag
#define SPI_SR_MODF         5  // Mode Fault
#define SPI_SR_OVR          6  // Overrun Flag
#define SPI_SR_BSY          7  // Busy Flag
#define SPI_SR_FRE          8  // TI Frame Format Error

/* SPI_I2SCFGR (I2S Configuration Register) bit positions */
#define SPI_I2SCFGR_CHLEN    0  // Channel Length (0: 16-bit, 1: 32-bit)
#define SPI_I2SCFGR_DATLEN   1  // Data Length (DATLEN[1:0], bits 2:1)
#define SPI_I2SCFGR_CKPOL    3  // Clock Polarity
#define SPI_I2SCFGR_I2SSTD   4  // I2S Standard Selection (bits 5:4)
#define SPI_I2SCFGR_PCMSYNC  7  // PCM Synchronization
#define SPI_I2SCFGR_I2SCFG   8  // I2S Configuration Mode (bits 9:8)
#define SPI_I2SCFGR_I2SE     10 // I2S Enable
#define SPI_I2SCFGR_I2SMOD   11 // I2S Mode Selection

// #include "stmG070RB_gpio_driver.h"
// #include "stmG070RB_spi_driver.h"
#include "../GPIO/stmG070RB_gpio_driver.h"
#include "../SPI/stmG070RB_spi_driver.h"


#endif /* INC_STM32G0XX_H_ */