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

//void find_proximity(void){
//	extra_speed = 0;
//	uint16_t current_proximity = 0;
//	uint16_t minimum_proximity = 0;
//	uint8_t current_sensor = NO_IR;
//	for(unsigned int i = 1 ; i < PROXIMITY_NB_CHANNELS; i++ ){
//		current_proximity = get_prox(i);
//		if((current_proximity > minimum_proximity) && (current_proximity > PROXIMITY_THRESHOLD)){
//			minimum_proximity = current_proximity;
//			current_sensor = i;
//		}
//	}
//
//	switch(current_sensor){
//	case IR3:
//	case IR4:
//		manual_speed(minimum_proximity);
//		break;
//	case IR1:
//	case IR2:
//		SEARCH_SIDE = SEARCH_LEFT;
//		break;
//	case IR5:
//	case IR6:
//		SEARCH_SIDE = SEARCH_RIGHT;
//		break;
//	default:
//		break;
//	}
//}

void manual_speed(uint16_t distance){
	int16_t error = 0;
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
	if((current_proximity > minimum_proximity) && (current_proximity > PROXIMITY_THRESHOLD)){
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
	case IR1:
	case IR2:
	case IR3:
		SEARCH_SIDE = SEARCH_LEFT;
		break;

	case IR4:
	case IR5:
	case IR6:
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
