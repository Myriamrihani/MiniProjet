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


int16_t find_proximity(void){
	uint16_t current_proximity = 0;
	uint16_t minimum_proximity = 0;
	uint8_t current_sensor = NEITHER;
	for(unsigned int i = 1 ; i <= PROXIMITY_NB_CHANNELS; i++ ){
		current_proximity = get_prox(i-1);
		if(current_proximity > minimum_proximity){
			if(current_proximity > PROXIMITY_THRESHOLD){//here, we suppose that we have to aim for the manual speed
			minimum_proximity = current_proximity;		//maybe remove it: example du pouce de myriam
			current_sensor = i;
			}
		}
	}

	if(current_sensor == IR3 || current_sensor == IR4){
		chprintf((BaseSequentialStream *)&SD3, "sensor_IR  : %d \r\n" , current_sensor -1);
		chprintf((BaseSequentialStream *)&SD3, "proximity  : %d \r\n" , minimum_proximity);
		return manual_speed(current_sensor, minimum_proximity);
	}



	return 0;
}

int16_t manual_speed(uint8_t sensor, uint16_t distance){ //should only be called with IR3 and 4
	int16_t extra_speed = 0;
	int16_t error = 0;
	error = log10(distance - PROXIMITY_THRESHOLD);

	//if we are far, error=PROX_THRESH-a_big_distance will be small
	//if we are close, error=PROX_THRESH-a_small_distance will be big
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}
	extra_speed =  error; //maybe add a KP factor
	return extra_speed;
}


//void avoid_obstacle(uint8_t sensor, uint16_t distance){
//	int16_t speed = 0;
//	int16_t error = 0;
//	error = PROXIMITY_THRESHOLD - distance; //normalement toujours positive puisque distance<threshold
//	speed = 800 * error; ///800 was like a PID, TO BE CHANGED - MAGIC NUMBER RN
//	int16_t speed_correction = 0; //=sensor - se retrouver dos au mur qu'on evite
//	switch (sensor)
//	{
//	case 0:
//		//Allumer LED1
//		//tourner vers la gauche de 180degres
//		//speed_correction = ...; //MAXIMUM HERE
//		break;
//	case 1:
//		//Allumer LED1 et LED3
//		//tourner vers la gauche de 135degres
//		//speed_correction = ...;
//		break;
//	case 2:
//		//Allumer LED3
//		//tourner vers la gauche de 90degres
//		//speed_correction = ...;
//		break;
//	case 3:
//		//Allumer LED5
//		//avancer
//		speed_correction = 0;
//		break;
//	case 4:
//		//Allumer LED5
//		//avancer
//		speed_correction = 0;
//		break;
//	case 5:
//		//Allumer LED7
//		//tourner vers la droite de 90degres
//		//speed_correction = ...
//		break;
//	case 6:
//		//Allumer LED1 et LED7
//		//tourner vers la gauche de 45degres
//		//speed_correction = ...
//		break;
//	case 7:
//		//Allumer LED1
//		//tourner vers la gauche de 135degres
//		//speed_correction = ...
//		break;
//	default:
//		break;
//	}
//
//	left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
//	right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
//
//}
