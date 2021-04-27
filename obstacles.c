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


void find_proximity(void){
	uint16_t current_proximity = 0;
	uint16_t minimum_proximity = PROXIMITY_THRESHOLD;
	uint8_t current_sensor = 0;
	for(unsigned int i = 0 ; i < PROXIMITY_NB_CHANNELS; i++ ){
		current_proximity = get_prox(i);
		if (current_proximity < minimum_proximity){
			minimum_proximity = current_proximity;
			current_sensor = i;
		}
	}
//	if (minimum_proximity <= PROXIMITY_THRESHOLD){
//		avoid_obstacle(current_sensor, minimum_proximity);
//	}

	chprintf((BaseSequentialStream *)&SD3, "sensor_IR  : %d \r\n" , current_sensor);
	chprintf((BaseSequentialStream *)&SD3, "proximity  : %d \r\n" , minimum_proximity);
}

void manual_speed(uint8_t sensor, uint16_t distance){ //should only be called with IR3 and 4
	int16_t extra_speed = 0;
	int16_t error = 0;
	error = PROXIMITY_THRESHOLD - distance;
	extra_speed = 800.0f * error; //maybe put KP in main.h!!!
	left_motor_set_speed(extra_speed); //here, no Speed_correction, but might have one if turning
	right_motor_set_speed(extra_speed); //we should try....
}
void avoid_obstacle(uint8_t sensor, uint16_t distance){
	int16_t speed = 0;
	int16_t error = 0;
	error = PROXIMITY_THRESHOLD - distance; //normalement toujours positive puisque distance<threshold
	speed = 800 * error; ///800 was like a PID, TO BE CHANGED - MAGIC NUMBER RN
	int16_t speed_correction = 0; //=sensor - se retrouver dos au mur qu'on evite
	switch (sensor)
	{
	case 0:
		//Allumer LED1
		//tourner vers la gauche de 180degres
		//speed_correction = ...; //MAXIMUM HERE
		break;
	case 1:
		//Allumer LED1 et LED3
		//tourner vers la gauche de 135degres
		//speed_correction = ...;
		break;
	case 2:
		//Allumer LED3
		//tourner vers la gauche de 90degres
		//speed_correction = ...;
		break;
	case 3:
		//Allumer LED5
		//avancer
		speed_correction = 0;
		break;
	case 4:
		//Allumer LED5
		//avancer
		speed_correction = 0;
		break;
	case 5:
		//Allumer LED7
		//tourner vers la droite de 90degres
		//speed_correction = ...
		break;
	case 6:
		//Allumer LED1 et LED7
		//tourner vers la gauche de 45degres
		//speed_correction = ...
		break;
	case 7:
		//Allumer LED1
		//tourner vers la gauche de 135degres
		//speed_correction = ...
		break;
	default:
		break;
	}

	left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
	right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);

}
