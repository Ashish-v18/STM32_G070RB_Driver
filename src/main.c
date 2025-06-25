/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Ashish 
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */


 #include "stmG070RB.h"

 void delay(void)
 {
     for(volatile uint32_t i = 0; i <= 500000; i++ );
 }

 int main(void)
 {

    GPIO_PeriClockControl(GPIOA, ENABLE);

    GPIO_Handle_t  GpioLed;
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPubPdControl = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);
 
    while(1)
    {
        GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
        delay();
    }
 }
 