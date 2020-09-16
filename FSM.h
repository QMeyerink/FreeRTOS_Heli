/*
 * FSM.h
 *
 *
 *  Created on: 26/07/2020
 *  For ENCE 464 Helicopter Project
 *  Contains a finite state machine for Yaw calcs
 *
 *
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */

#ifndef FSM_H_
#define FSM_H_

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "Yaw.h"

enum yaw_state {
    state_zero,
    state_one,
    state_two,
    state_three

};

enum flying_state {
    landed = 0,
    calibration,
    flying,
    landing,

};

typedef enum yaw_state yaw_state_t;
typedef enum flying_state flying_state_t;
#define NUM_OF_PINS 448

yaw_state_t previous_state;
flying_state_t fly_state;
extern int32_t yaw_ticks;


void direction_calculator(bool sensorA);

void init_state(bool sensorA, bool sensorB);

flying_state_t update_state(flying_state_t fly_state);

#endif /* FSM_H_ */
