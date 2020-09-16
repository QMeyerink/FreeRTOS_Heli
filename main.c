/*
 * Main program for Helicopter FreeRTOS project
 *
 *
 *  Created on: 23/07/2020
 *  For ENCE 464 Helicopter Project
 *
 *
 *
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */
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
#include "driverlib/debug.h"
#include "driverlib/uart.h"
#include "FreeRTOS.h"
#include "task.h"

#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"

#include "Calcs.h"
#include "FSM.h"
#include "Yaw.h"
#include "ADC.h"
#include "buttons4.h"
#include "PWM_Module.h"
#include "PID_Controller.h"
#include "sequence_check.h"
#include "UART.h"
#include "queue.h"
#include "semphr.h"


circBuf_t g_inBuffer;                                           //Create circular buffer value
int16_t bottom_altitude = 0;

#define LED_PIN_RED                 1                                   // RED Led pin

//Task wait times, random at this point, will alter for efficency
#define HEIGHT_CALC_RATE            250
#define PWM_UPDATE_RATE             200
#define PID_CONTROLLER_RATE         100
#define DISPLAY_RATE                100
#define BUTTON_OPERATION_RATE       5
#define STATE_UPDATE_RATE           100
#define BUTTON_POLL_RATE            5
#define UART_UPDATE_RATE            100

//Task stack depth, again will change (all currently running on display which is bad)
#define BLINK_TASK_STACK_DEPTH      32
#define DISPLAY_TASK_STACK_DEPTH    64
#define TASK_PRIORITY               4
#define BUF_SIZE                    25

#define YAW_STEP                    15
#define ALTITUDE_STEP               10
#define ALTITUDE_RANGE              1100

#define ALTITUDE_MAX                100
#define ALTITUDE_MIN                0
#define YAW_MIN                     -180
#define YAW_MAX                     180
#define REFERENCE                   5
#define ROTORS_OFF                  0

#define SET_ALT_GOAL                50

xTaskHandle xButtonTask;
xTaskHandle xPWMTask;

SemaphoreHandle_t xbutton_semaphore;
SemaphoreHandle_t xcontrol_semaphore;
SemaphoreHandle_t xcirc_buff_mutex;

QueueHandle_t _UART_queue;

int16_t called = 0;

struct HeliData {
    int16_t altitude;
    int16_t altitude_goal;
    int16_t yaw;
    int16_t yaw_goal;
    int16_t main_PWM;
    int16_t tail_PWM;
    flying_state_t state;

};


void find_alt_bottom(void)
{
    int16_t i = 0;

    for(i = 0; i < 100; i++) {
        SysCtlDelay (SysCtlClockGet ()/100);
        ADCProcessorTrigger(ADC0_BASE, 3);
    }

    bottom_altitude = calc_av();
}

void ADCTrigger(void)
{
    while(1) {

        xSemaphoreTake(xcirc_buff_mutex, portMAX_DELAY);

        ADCProcessorTrigger(ADC0_BASE, 3);

        xSemaphoreGive(xcirc_buff_mutex);

        vTaskDelay(ADC_TICK_RATE / portTICK_RATE_MS);
    }
}

void displayUpdate(void * data)
{
    // Displays on the OLED, the altitude data (raw or percentage scaled) and the yaw distance
    // depending on which page has been selected
    struct HeliData * heli_data = (struct HeliData *)data;

    while(1)
    {

        int8_t i = 0;
        for(i=0; i<10; i++) {
            char line1[17]; // Display fits 16 characters wide.
            char line2[17];
            char line3[17];
            char line4[17];

            int16_t altitude_goal = heli_data->altitude_goal;
            int16_t altitude = heli_data->altitude;

            int16_t yaw_goal = heli_data->yaw_goal;
            int16_t yaw = heli_data->yaw;

            //Copy text into string char types for OLED display
            usnprintf(line1, sizeof(line1), "Yaw goal     " );
            usnprintf(line2, sizeof(line2), "     %2d [%2d]     ",yaw, yaw_goal); //Checks goals to test
            usnprintf(line3, sizeof(line1), "Altitude goal");
            usnprintf(line4, sizeof(line2), "     %2d [%2d]   ",altitude, altitude_goal); //Checks goals to test

            //Draw specified strings.
            OLEDStringDraw (line1, 0, 0);
            OLEDStringDraw (line2, 0, 1);
            OLEDStringDraw (line3, 0, 2);
            OLEDStringDraw (line4, 0, 3);
            vTaskDelay(DISPLAY_RATE / portTICK_RATE_MS);
        }



    }

}
void calc_height(void * data)
//Task to calculate the height of the heli, and writes it to the data struct
{
    struct HeliData * heli_data = (struct HeliData *)data;

    while(1)
    {
        xSemaphoreTake(xcirc_buff_mutex,portMAX_DELAY);
        heli_data->altitude = calc_perc(calc_av(), bottom_altitude);

        xSemaphoreGive(xcirc_buff_mutex);

        vTaskDelay(HEIGHT_CALC_RATE / portTICK_RATE_MS);

    }
}

void calc_yaw(void * data)
//Task to calculate the yaw of the heli in degrees, and writes it to the data struct
{
    struct HeliData * heli_data = (struct HeliData *)data;

    while(1)
    {
        heli_data->yaw = tick_to_deg();
        vTaskDelay(HEIGHT_CALC_RATE / portTICK_RATE_MS);
    }
}

void set_heli_PWM(void * data)
//Task to set the PWM of the heli, will read data from struct
{
    struct HeliData * heli_data = (struct HeliData *)data;

    while(1)
    {
        //Check if control has been updated, wait until it has.
        xSemaphoreTake(xcontrol_semaphore, portMAX_DELAY);
        set_PWM(heli_data->main_PWM, heli_data->tail_PWM);
        vTaskDelay(PWM_UPDATE_RATE / portTICK_RATE_MS);
    }
}

//Query the PID functon and write result to data struct
void update_control(void * data)
{
    struct HeliData * heli_data = (struct HeliData *)data;

    while(1)
    {
        pid_update(heli_data->altitude, heli_data->altitude_goal, heli_data->yaw, heli_data->yaw_goal);
        heli_data->main_PWM = get_altitude_control();
        heli_data->tail_PWM = get_yaw_control();

        //Control has been updated so PWM can be updated
        xSemaphoreGive(xcontrol_semaphore);
        vTaskDelay(PID_CONTROLLER_RATE / portTICK_RATE_MS);
    }
}

//Increases and decreases the main rotor PWM (will change to altitude goal)
void check_buttons(void * data)
{
    //Cast void type pointer into unsigned 16 bit pointer
    struct HeliData * heli_data = (struct HeliData *)data;

    while(1)
    {
        //Make sure hardware buttons have been polled since last checking buttons.
        xSemaphoreTake(xbutton_semaphore, portMAX_DELAY);

        if (checkButton(UP) == PUSHED) {
            if (heli_data->altitude_goal > ALTITUDE_MAX-ALTITUDE_STEP) {
                heli_data->altitude_goal = ALTITUDE_MAX;
            } else {
                heli_data->altitude_goal += ALTITUDE_STEP;
            }
            if (write_list(UP_L))
                {
                heli_data->altitude_goal = SET_ALT_GOAL;
                }
        }

        if (checkButton(DOWN) == PUSHED) {
        //Increment altitude by -10% down to 0%
            if (heli_data->altitude_goal < ALTITUDE_MIN+ALTITUDE_STEP) {
                heli_data->altitude_goal = ALTITUDE_MIN;
            } else {
                heli_data->altitude_goal -= ALTITUDE_STEP;
            }
            if (write_list(DOWN_L))
                {
                heli_data->altitude_goal = SET_ALT_GOAL;
                }
        }
        if (checkButton(LEFT) == PUSHED) {
            ////Increment yaw by 15 deg down to -180
            if (heli_data->yaw_goal <= YAW_MIN+YAW_STEP) {
                heli_data->yaw_goal = YAW_MAX;
            } else {
                heli_data->yaw_goal -= YAW_STEP;
            }
            if (write_list(LEFT_L))
            {
                if(heli_data->yaw_goal >= 0)
                {
                    heli_data->yaw_goal -= YAW_MAX;
                } else
                {
                    heli_data->yaw_goal += YAW_MAX;
                }
            }
        }

        if (checkButton(RIGHT) == PUSHED) {
            //Increment yaw by 15 deg up to +180
            if (heli_data->yaw_goal >= YAW_MAX-YAW_STEP) {
                heli_data->yaw_goal = YAW_MIN;
            } else {
                heli_data->yaw_goal += YAW_STEP;
            }
            if (write_list(RIGHT_L))
                {
                    if(heli_data->yaw_goal >= 0)
                    {
                        heli_data->yaw_goal -= YAW_MAX;
                    } else
                    {
                        heli_data->yaw_goal += YAW_MAX;
                    }
                }
        }



        vTaskDelay(BUTTON_OPERATION_RATE / portTICK_RATE_MS);
    }
}

void poll_buttons(void)
{
    while(1)
    {
        updateButtons();

        if(checkButton(RESET) == PUSHED) {
            SysCtlReset();
        }
        //Give to binary Semaphore to allow checkbuttons to run once
        xSemaphoreGive(xbutton_semaphore);
        vTaskDelay(BUTTON_POLL_RATE / portTICK_RATE_MS);

    }
}

void state_check(void * data)
{
    struct HeliData * heli_data = (struct HeliData *)data;

    while(1)
    {
        heli_data->state = update_state(heli_data->state);

        if(heli_data->state == flying)
        {
            vTaskResume(xButtonTask);
            vTaskResume(xPWMTask);

        } else if(heli_data->state == landed)
        {
            vTaskSuspend(xButtonTask);
            vTaskSuspend(xPWMTask);

        } else if(heli_data->state == calibration)
        {
            if(at_ref())
            {
                heli_data->state = flying;
            }

        } else if(heli_data->state == landing)
        {
            vTaskResume(xPWMTask);
            vTaskSuspend(xButtonTask);

            heli_data->yaw_goal = 0;
            heli_data->altitude_goal = 0;
            if(heli_data->altitude <= abs(REFERENCE) && heli_data->yaw <= abs(REFERENCE))
            {
                set_PWM(ROTORS_OFF,ROTORS_OFF);
                heli_data->state = landed;
            }
        }
        vTaskDelay(STATE_UPDATE_RATE / portTICK_RATE_MS);
    }
}

void UART_send (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
        //use to produce string: usprintf (statusStr, "UP=%2d DN=%2d | \r\n", upPushes, downPushes);
    }
}


void CPU_check(char runtime_stats_buffer[512])
{
     portENTER_CRITICAL();

     vTaskGetRunTimeStats(runtime_stats_buffer);
     UART_send(runtime_stats_buffer);
     portEXIT_CRITICAL();


}

void update_UART_queue(void * data)
{
    struct HeliData * heli_data = (struct HeliData *)data;

    static char runtime_stats_buffer[512];
    while(1)
    {
        UART_update(heli_data->altitude_goal, heli_data->altitude, heli_data->yaw_goal, heli_data->yaw, heli_data->main_PWM, heli_data->tail_PWM,heli_data->state);
        called++;

        if(called > 50)
        {
            //CPU_check(runtime_stats_buffer);
            called = 0;
        }
        vTaskDelay(UART_UPDATE_RATE / portTICK_RATE_MS);
    }
}

void UART_print(void)
{
    while(1)
    {
        char ch;
        xQueueReceive(_UART_queue, &ch, portMAX_DELAY);
        UARTCharPut(UART_USB_BASE, ch);
    }
}

void initSys(void)
{
    OLEDInitialise ();
    init_ADC();
    initCircBuf(&g_inBuffer, BUF_SIZE);
    init_yaw();
    initButtons();
    init_PWM();
    initialiseUSB_UART();
    //vConfigureTimerForRunTimeStats()
}


int main(void)
{

    static struct HeliData heli_data = {0, 0, 0, 0, 0, 0, flying};

    xbutton_semaphore = xSemaphoreCreateBinary();
    xcontrol_semaphore = xSemaphoreCreateBinary();
    xcirc_buff_mutex = xSemaphoreCreateMutex();

    _UART_queue = xQueueCreate(200,1);



    //SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    //Run system init tasks (single use no need to make into tasks)
    initSys();
    find_alt_bottom();

    //Create display task with no variables
    if (pdTRUE != xTaskCreate(displayUpdate, "Display", DISPLAY_TASK_STACK_DEPTH, (void *) &heli_data, 2, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    //Create task to trigger ADC
    if (pdTRUE != xTaskCreate(ADCTrigger, "ADC trigger", DISPLAY_TASK_STACK_DEPTH, NULL, 9, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    //Poll the buttons
    if (pdTRUE != xTaskCreate(poll_buttons, "Update butt", DISPLAY_TASK_STACK_DEPTH, NULL, 4, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    //Use button values to change LED blink rate (eventually use to increase altitude just testing)
    if (pdTRUE != xTaskCreate(check_buttons, "check Butt", DISPLAY_TASK_STACK_DEPTH, (void *) &heli_data, 4, &xButtonTask))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(calc_height, "Altitude", DISPLAY_TASK_STACK_DEPTH, (void *) &heli_data, 10, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(calc_yaw, "Yaw", DISPLAY_TASK_STACK_DEPTH, (void *) &heli_data, 10, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(set_heli_PWM, "PWM", DISPLAY_TASK_STACK_DEPTH, (void *) &heli_data, 6, &xPWMTask))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(update_control, "PID", DISPLAY_TASK_STACK_DEPTH, (void *) &heli_data, 6, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(update_UART_queue, "UART_q", DISPLAY_TASK_STACK_DEPTH, (void *) &heli_data, 2, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(UART_print, "Uart_p", DISPLAY_TASK_STACK_DEPTH, NULL, 2, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(state_check, "state_check", DISPLAY_TASK_STACK_DEPTH, (void *) &heli_data, 8, NULL))
    {
        while(1);               // Oh no! Must not have had enough memory to create the task.
    }

    //Always boot to landed state so don't want to run button or PWM tasks
    vTaskSuspend(xButtonTask);
    vTaskSuspend(xPWMTask);

    vTaskStartScheduler();      // Start FreeRTOS!!

    while(1);                   // Should never get here since the RTOS should never "exit".
}
