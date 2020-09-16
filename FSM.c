/*
 * FSM.c
 *
 *
 *  Created on: 26/07/2020
 *  For ENCE 464 Helicopter Project
 *  Contains a finite state machine for Yaw calcs
 *
 *
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */

#include "FSM.h"


void direction_calculator(bool sensorA)
//De/Increments the distance value
//to reflect state change
//Takes previous state and moves to next state
//depending on which sensor changed
//Transitions are gray coded i.e only
//one sensor changes value (A)
{
    //If a full rotation has been completed set back to distance count back to 0
    if(yaw_ticks == NUM_OF_PINS/2) {
        yaw_ticks = -180;
    } else if(yaw_ticks == -NUM_OF_PINS/2) {
        yaw_ticks = 180;
    }

    switch(previous_state) {
        case state_zero : // A=0, B=0
            if(sensorA) {
                previous_state = state_three;
                yaw_ticks -= 1;
                break;

            } else {
                previous_state = state_one;
                yaw_ticks += 1;
                break;
            }

        case state_one : // A=0, B=1
            if(sensorA) {
                previous_state = state_two;
                yaw_ticks += 1;
                break;

            } else {
                previous_state = state_zero;
                yaw_ticks -= 1;
                break;
            }

        case state_two : // A=1, B=1
            if(sensorA) {
                previous_state = state_three;
                yaw_ticks += 1;
                break;

            } else {
                previous_state = state_one;
                yaw_ticks -= 1;
                break;
            }

        case state_three : // A=1, B=0
            if(sensorA) {
                previous_state = state_two;
                yaw_ticks -= 1;
                break;

            } else {
                previous_state = state_zero;
                yaw_ticks += 1;
                break;
            }
    }

}


void init_state(bool sensorA, bool sensorB)
//Selects start up state from the sensor input
{
    yaw_state_t return_state;
    yaw_ticks = 0;

    //States are gray-coded to allow simple transitions
    if(sensorA) {
        if(sensorB) {
            return_state = state_two; // A=1, B=1

        } else {
            return_state = state_three; // A=1, B=0
        }
    } else {
        if(sensorB) {
            return_state = state_one; // A=0, B=1

        } else {
            return_state = state_zero; // A=0, B=0
        }
    }
    previous_state = return_state;
}


flying_state_t update_state(flying_state_t fly_state) {

    if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_INT_PIN_7) == GPIO_INT_PIN_7) { //Check current position of slider switch.
        if(fly_state == landed) { //Move from landed to calibration state
            fly_state = calibration;
            yaw_calibration();
        }

    } else {
        if(fly_state == flying){ //slider down, move from flying to landing
            fly_state = landing;
        }
    }

    return fly_state;

}

