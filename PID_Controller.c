/*
 * PID_Controller.c
 *
 *  Created on: 20/04/2019
 *  Contains a PID controller with static gains
 *  A function to keep PWM duty cycle values within bounds
 *  Two functions to access the current main and rotor duty cycles.
 *
 *  For ENCE 464 Helicopter Project
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */

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
#define M_Kp 0.4    //0.4
#define M_Ki 0.01 //0.0006
#define M_Kd 0.5

//Tail Rotor gains
#define T_Kp 0.15      //4
#define T_Ki 0.0008 //0.0009
#define T_Kd 3.5

#define TAIL 0
#define MAIN 1
#define MAXIMUM_DUTY_CYCLE 70
#define MINIMUM_DUTY_CYCLE 2

#define CLK_SPEED 16000
#define HOVER_LEVEL 43       //50 ~14 for Heli 2??~
#define TAIL_COEFFICENT 0.80
#define YAW_MAX_ERROR 20
#define ALT_MAX_ERROR 15
#define HALF_CIRCLE 180
#define FULL_CIRCLE 360

//File wide globals, an unideal way to do this
//Should consider other implimentation
static int16_t altitude_control = 0;
static int16_t yaw_control = 0;

static int16_t prev_time = 0;
static int16_t last_yaw_error = 0;
static int16_t last_altitude_error = 0;

//PID update algorithm this is a big task, using a lot of data
//Ideally we will reduce this
//We will need to add Semaphores for critical zones else we will see some terrible behavior
void pid_update(int16_t altitude, int16_t altitude_setpoint, int16_t yaw, int16_t yaw_setpoint)
{
    int16_t altitude_error, yaw_error, error_inter_main, error_inter_tail, error_deriv_main, error_deriv_tail;

    //Delta_t set to random value, will use FreeRTOS functions to calc change in time
    int16_t curr_time = xTaskGetTickCount();
    int16_t delta_t = ((curr_time - prev_time)/CLK_SPEED);
    prev_time = curr_time;
    //int16_t delta_t = 1000;


    //Error is the difference between where we are and where we want to be.
    altitude_error = (altitude_setpoint - altitude);
    yaw_error = (yaw_setpoint - yaw);


    //YAW_ERROR
    if(yaw_error > HALF_CIRCLE)
    {
       yaw_error = -(FULL_CIRCLE - abs(yaw_error));
    } else if(yaw_error < -HALF_CIRCLE)
    {
       yaw_error = FULL_CIRCLE - abs(yaw_error);
    }

    //Limit yaw error to avoid spinning out of control
    if(yaw_error > YAW_MAX_ERROR)
    {
      yaw_error = YAW_MAX_ERROR;
    } else if (yaw_error < -YAW_MAX_ERROR)
    {
      yaw_error = -YAW_MAX_ERROR;
    }

    if(altitude_error > ALT_MAX_ERROR)
    {
     altitude_error = ALT_MAX_ERROR;
    }


    //Calculate the integral error
    error_inter_main = altitude_error * delta_t;
    error_inter_tail = yaw_error * delta_t;

    //Calculate the derivative error
    error_deriv_main = (altitude_error - last_altitude_error)/delta_t;
    error_deriv_tail = (yaw_error - last_yaw_error)/delta_t;

    //Update the last error values
    last_yaw_error = yaw_error;
    last_altitude_error = altitude_error;

    //Set control level value (This will set the PWM duty-cycle)
    altitude_control = HOVER_LEVEL + (altitude_error*M_Kp) + (error_inter_main*M_Ki) + (error_deriv_main*M_Kd);
    yaw_control = TAIL_COEFFICENT*altitude_control + (yaw_error*T_Kp) + (error_inter_tail*T_Ki) + (error_deriv_tail*T_Kd);


    if(altitude_control > MAXIMUM_DUTY_CYCLE) {
        altitude_control = MAXIMUM_DUTY_CYCLE;
    } else if(altitude_control < MINIMUM_DUTY_CYCLE) {
        altitude_control = MINIMUM_DUTY_CYCLE;
    }
    if(yaw_control > MAXIMUM_DUTY_CYCLE) {
        yaw_control = MAXIMUM_DUTY_CYCLE;
    } else if(yaw_control < MINIMUM_DUTY_CYCLE) {
        yaw_control = MINIMUM_DUTY_CYCLE;
    }

}

int16_t get_altitude_control() {
    return altitude_control;
}

int16_t get_yaw_control() {
    return yaw_control;
}

