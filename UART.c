//********************************************************
//
// UART Module for Helicopter Project abstracted from uartDemo.c - Example code for ENCE361
//Sends packets of information on the helicopters altitude, yaw, set points, flying state and
// PWM duty cycles to the data queue
//UART.c
// Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
//
#include "UART.h"

void initialiseUSB_UART (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}
bool dputs (const char *string)
{
    while(*string) {
        if (!xQueueSend(_UART_queue, string++, 0)){
            return false;
        }
    }
    return true;
}

void UART_update(int16_t altitude_goal, int16_t altitude, int16_t yaw_goal, int16_t yaw, int16_t main_PWM, int16_t tail_PWM, flying_state_t state)
{

        char statusStr[MAX_STR_LEN + 1];

        usprintf (statusStr, "Yaw: %2d [%2d]\n\n", yaw, yaw_goal);
        dputs(statusStr);
        usprintf (statusStr, "Alt: %2d [%2d]\n\n", altitude, altitude_goal);
        dputs(statusStr);
        usprintf (statusStr, "PWM Main %%: %2d\n\n", main_PWM);
        dputs(statusStr);
        usprintf (statusStr, "PWM Tail %%: %2d\n\n", tail_PWM);
        dputs(statusStr);
        usprintf (statusStr, "State: %2d\n\n", state);
        dputs(statusStr);
        //usprintf (statusStr, "------------\n\n");
        //UART_send (statusStr);

}
