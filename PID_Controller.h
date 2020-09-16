/*
 * PID_Controller.h
 *
 *  Created on: 20/04/2019
 *  Contains a PID controller with static gains
 *  A function to keep PWM duty cycle values within bounds
 *  Two functions to access the current main and rotor duty cycles.
 *
 *  For ENCE 464 Helicopter Project
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */

#ifndef PID_Controller_H_
#define PID_Controller_H_


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
#include "PWM_Module.h"

//Main Rotor gains
#define M_Kp 0.3
#define M_Ki 0.0007
#define M_Kd 0.3

//Tail Rotor gains
#define T_Kp 1.0
#define T_Ki 0.0011
#define T_Kd 2.5

#define TAIL 0
#define MAIN 1
#define MAXIMUM_DUTY_CYCLE 70
#define MINIMUM_DUTY_CYCLE 2

#define CLK_SPEED 80000


static int16_t altitude_pervious_error = 0.0;
static int16_t yaw_pervious_error = 0.0;

int16_t altitude_control;
int16_t yaw_control;

static int16_t error_inter_main = 0.0;
static int16_t error_inter_tail = 0.0;

//void PWM_rate_set();

void pid_update(int16_t altitude, int16_t altitude_setpoint, int16_t yaw, int16_t yaw_setpoint);

int16_t get_altitude_control();

int16_t get_yaw_control();

#endif /* PID_Controller_H_ */
