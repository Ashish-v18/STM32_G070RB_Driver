/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Ashish Kumar
 * @brief          : GPIO LED Blink Test using STM32G070RB Driver Library
 ******************************************************************************
 * @attention
 *
 * This example demonstrates the use of the custom STM32G070RB low-level 
 * driver library by toggling an LED connected to GPIO pin PA5.
 * 
 * It serves as a functional test to verify that the GPIO configuration,
 * peripheral clock control, and output operations are working correctly
 * with the implemented library functions.
 * 
 * Target MCU     : STM32G070RB
 * Board          : Nucleo-G070RB
 * LED Pin        : PA5 (On-board LED on Nucleo board)
 *
 ******************************************************************************
 */

#include "stmG070RB.h"

/**
 * @brief Simple blocking delay function
 * Used here just to introduce a visible delay between LED toggles.
 */
void delay(void)
{
    for (volatile uint32_t i = 0; i <= 10000000; i++);
}

/**
 * @brief Main entry point
 * Initializes GPIOA pin 5 as output and toggles it in a loop.
 * This confirms the proper functioning of the GPIO driver code.
 */
int main(void)
{
    // Enable GPIOA peripheral clock
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // GPIO configuration structure
    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPubPdControl = GPIO_NO_PUPD;

    // Initialize GPIO pin
    GPIO_Init(&GpioLed);

    // Toggle LED in an infinite loop
    while (1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        delay();
    }
}
