/*
 * PID_Controller.c
 *
 *  Created on: 15/05/2019
 *
 *  Contains the PWM initialise function
 *  Contains a function to set the current frequency and duty for PWM
 *
 *  For ENCE 464 Helicopter Project
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */
#include "PWM_Module.h"

void set_PWM (uint32_t main_duty, uint32_t tail_duty)
{

    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_RATE_HZ;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,ui32Period * main_duty / 100);

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32Period * tail_duty / 100);


}

void init_PWM (void)
{

    //Enable the output pins for the main rotor PWM
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    //Enable the output pins for the tail rotor PWM
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);


    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);


    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    set_PWM(0,0);

    //Turn the PWM on with 0 duty cycle.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

