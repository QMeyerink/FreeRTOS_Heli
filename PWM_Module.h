/*
 * PID_Controller.h
 *
 *  Created on: 15/05/2019
 *
 *  Contains the PWM initialise function
 *  Contains a function to set the current frequency and duty for PWM
 *
 *  For ENCE 464 Helicopter Project Milestone 2
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */

#ifndef PWM_Module_H_
#define PWM_Module_H_



#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "buttons4.h"

// PWM configuration
#define PWM_RATE_HZ        300
#define PWM_START_DUTY     2

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Tail Rotor PWM: PC5, J4-05
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

#define CALIBRATION_YAW_PWM_DUTY 80
#define CALIBRATION_ALTITUDE_PWM_DUTY 5

void set_PWM(uint32_t main_duty, uint32_t tail_duty);

void init_PWM(void);

#endif /*PWM_Module_h*/
