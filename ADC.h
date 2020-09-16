/*
 * ADC.h
 *
 *
 *  Created on: 26/07/2020
 *  For ENCE 464 Helicopter Project Milestone 2
 *  Contains ADC interrupt handler for altitude sensor and

 *
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */


#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "driverlib/debug.h"
#include "ADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "OrbitOLED/OrbitOLEDInterface.h"

circBuf_t g_inBuffer;
uint32_t g_ulSampCnt;
uint32_t g_kernal_counter;

#define ADC_TICK_RATE 10

#define SAMPLE_RATE_HZ 100      // Rate at which altitude is sampled

#define SAMPLE_RATE_HZ 100      // Rate at which altitude is sampled

void ADC_int_handler(void);

void init_ADC (void);

void ADCTrigger(void);

#endif /*ADC_H_*/
