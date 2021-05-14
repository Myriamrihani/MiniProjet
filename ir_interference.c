#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <math.h>
#include "memory_protection.h"
#include "motors.h"
#include <ir_interference.h>
#include <sensors/proximity.h>


#define PROXIMITY_THRESHOLD		100
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera


static int16_t extra_speed = 0;
static SEARCHING_SIDE SEARCH_SIDE = NO_SEARCH_SIDE;


void manual_speed(uint16_t distance){
	int16_t error = 0;

	//extra_speed correspond to the log of the error because of the IRs design
	error = log10(distance - PROXIMITY_THRESHOLD);
	if(fabs(error) < ERROR_THRESHOLD){
		error = 0;
	}
	extra_speed =  error;
}


int16_t get_extra_speed(void){
	extra_speed = 0;
	uint16_t current_proximity = 0;
	uint16_t minimum_proximity = 0;

	current_proximity = get_prox(IR3);
	if((current_proximity > minimum_proximity)
		&& (current_proximity > PROXIMITY_THRESHOLD)){
		minimum_proximity = current_proximity;
	}

	current_proximity = get_prox(IR4);
	if((current_proximity > minimum_proximity)
		&& (current_proximity > PROXIMITY_THRESHOLD)){
		minimum_proximity = current_proximity;
	}
	manual_speed(minimum_proximity);

	return extra_speed;
}


SEARCHING_SIDE get_search_side(void){
	uint16_t current_proximity = 0;
	uint16_t minimum_proximity = 0;
	uint8_t current_sensor = NO_IR;

	for(unsigned int i = 1 ; i < PROXIMITY_NB_CHANNELS; i++ ){
		current_proximity = get_prox(i);
		if((current_proximity > minimum_proximity) && (current_proximity > PROXIMITY_THRESHOLD)){
			minimum_proximity = current_proximity;
			current_sensor = i;
		}
	}

	switch(current_sensor){

	//being close to any sensor on the right side activates the command
	case IR0:
	case IR1:
	case IR2:
	case IR3:
		SEARCH_SIDE = SEARCH_LEFT;
		break;

	//being close to any sensor on the left side activates the command
	case IR4:
	case IR5:
	case IR6:
	case IR7:
		SEARCH_SIDE = SEARCH_RIGHT;
		break;
	default:
		break;
	}

	return SEARCH_SIDE;
}

void set_search_side(SEARCHING_SIDE side){
	SEARCH_SIDE = side;
}
