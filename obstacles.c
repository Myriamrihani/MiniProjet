/*
 * obstacles.c
 *
 *      Author: Bryan Kheirallah
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <usbcfg.h>				//this one is not necessarily useful

#include <math.h>
#include "memory_protection.h"
#include "motors.h"
#include <obstacles.h>
#include <sensors/proximity.h>

static int16_t extra_speed = 0;
static SEARCHING_SIDE SEARCH_SIDE = NO_SEARCH_SIDE;

void find_proximity(void){
	extra_speed = 0;
	uint16_t current_proximity = 0;
	uint16_t minimum_proximity = 0;
	uint8_t current_sensor = NO_IR;
	for(unsigned int i = 1 ; i < PROXIMITY_NB_CHANNELS; i++ ){
		current_proximity = get_prox(i);
		if(current_proximity > minimum_proximity){
			if(current_proximity > PROXIMITY_THRESHOLD){//here, we suppose that we have to aim for the manual speed
			minimum_proximity = current_proximity;		//maybe remove it: example du pouce de myriam
			current_sensor = i;
			}
		}
	}

	if(current_sensor == IR3 || current_sensor == IR4){
		manual_speed(minimum_proximity);
	}
	else if (current_sensor == IR1 || current_sensor == IR2){
		chprintf((BaseSequentialStream *)&SD3, "sensor_IR  : %d \r\n" , current_sensor);
		chprintf((BaseSequentialStream *)&SD3, "proximity  : %d \r\n" , minimum_proximity);
		SEARCH_SIDE = SEARCH_LEFT;
	}
	else if (current_sensor == IR5 || current_sensor == IR6){
		chprintf((BaseSequentialStream *)&SD3, "sensor_IR  : %d \r\n" , current_sensor);
		chprintf((BaseSequentialStream *)&SD3, "proximity  : %d \r\n" , minimum_proximity);
		SEARCH_SIDE = SEARCH_RIGHT;
	}
}

void manual_speed(uint16_t distance){ //should only be called with IR3 and 4

	int16_t error = 0;
	error = log10(distance - PROXIMITY_THRESHOLD);

	if(fabs(error) < ERROR_THRESHOLD){
		extra_speed = 0;
	}
	extra_speed =  error; //maybe add a mini KP factor
}

int16_t get_extra_speed(void){
	find_proximity();
	return extra_speed;
}

SEARCHING_SIDE get_search_side(void){
	if(SEARCH_SIDE == NO_SEARCH_SIDE){
		find_proximity();
	}
	return SEARCH_SIDE;
}

void set_search_side(SEARCHING_SIDE side){
	SEARCH_SIDE = side;
}
