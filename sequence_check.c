/*
 * sequence_check.h
 *
 *
 *  Created on: 26/07/2020
 *  For ENCE 464 Helicopter Project
 *  Contains functions for comparing button press sequences to predefined command sequences
 *
 *
 *      Authors: Quinlan Meyerink, Te Atawhai Maginness, Aidan Ogilvie
 */
#include "sequence_check.h"

int16_t list_prev_time = 0;
uint8_t button_list[4] = {0, 0, 0, 0};
uint8_t altitude_list[4] = {UP_L, DOWN_L, UP_L, DOWN_L};
uint8_t turn_180_list[4] = {LEFT_L, RIGHT_L, LEFT_L, RIGHT_L};

bool check_equal_lists(uint8_t but_list[4], uint8_t check_list[4])
{
    int i = 0;
    for(i = 0; i < 4; i++) {
        if (but_list[i] != check_list[i]) {
            return 0;
        }
    }
    return 1;
}

bool write_list(uint8_t press)
{
    int16_t list_curr_time = xTaskGetTickCount() / CLOCK_SPEED;

    if ((list_curr_time-list_prev_time) > TIMER_LIM)
        {
        int i;
        for(i = 0; i < 4; i++)
        {
            button_list[i] = 0;
        }
    }
    int j;
    for(j = 0; j < 3; j++ ) {
        button_list[j] = button_list[j+1];
    }
    button_list[3] = press;

    if(check_equal_lists(button_list, altitude_list)) {
        int i;
        for(i = 0; i < 4; i++)
        {
            button_list[i] = 0;
        }
        return 1;
    }
    if(check_equal_lists(button_list, turn_180_list)) {
        int i;
        for(i = 0; i < 4; i++)
        {
            button_list[i] = 0;
        }
        return 1;
    }

    list_prev_time = (list_curr_time / CLOCK_SPEED);

    return 0;
}
