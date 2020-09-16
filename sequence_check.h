/*
 * sequence_check.c
 *
 *
 *  Created on: 26/07/2020
 *  For ENCE 464 Helicopter Project
 *  Contains functions for comparing button press sequences to predefined command sequences
 *
 *
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */
#ifndef sequence_checker_H_
#define sequence_checker_H_

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "stdlib.h"


#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "utils/ustdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#define TIMER_LIM     1000
#define CLOCK_SPEED   16000


enum buttons {
    UP_L = 1,
    DOWN_L = 2,
    LEFT_L = 3,
    RIGHT_L = 4
};


bool check_equal_lists(uint8_t but_list[4], uint8_t check_list[4]);

bool write_list(uint8_t press);




#endif /*sequence_checker_h_*/
