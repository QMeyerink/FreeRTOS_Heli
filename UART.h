//********************************************************
//
// UART Module for Helicopter Project
//UART.c
// Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
//
//


#ifndef UART_H_
#define UART_H_


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buttons4.h"
#include "FSM.h"
#include "PID_Controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS.h"
#include "queue.h"

//********************************************************
// Constants
//********************************************************

//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

#define MAX_STR_LEN             20

QueueHandle_t _UART_queue;

//********************************************************
// Prototypes
//********************************************************

void initialiseUSB_UART (void);

bool dputs (const char *string);

void UART_update(int16_t altitude_goal, int16_t altitude, int16_t yaw_goal, int16_t yaw, int16_t main_PWM, int16_t tail_PWM, flying_state_t state);

//********************************************************
// Global variables
//********************************************************



#endif /* FSM_H_ */
