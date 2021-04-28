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
		if (current_proximity < minimum_proximity){		//here, we suppose that with have to aim for the manual speed
			minimum_proximity = current_proximity;		//maybe remove it: example du pouce de myriam
			current_sensor = i;
		}
	}
	if(minimum_proximity <= PROXIMITY_THRESHOLD){
//		avoid_obstacle(current_sensor, minimum_proximity);
		if(current_sensor == 3 || current_sensor == 4){		//MAGIC_NUMBERS!!!!!
			manual_speed(current_sensor, minimum_proximity);
		}
	}

	chprintf((BaseSequentialStream *)&SD3, "sensor_IR  : %d \r\n" , current_sensor);
	chprintf((BaseSequentialStream *)&SD3, "proximity  : %d \r\n" , minimum_proximity);
}


int16_t pi_regulator(int16_t error){

	float speed = 0;
	static float sum_error = 0;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;			//this part is not yet good

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;
    return (int16_t)speed;
}

void manual_speed(uint8_t sensor, uint16_t distance){ //should only be called with IR3 and 4
	int16_t extra_speed = 0;
	int16_t error = 0;
	error = PROXIMITY_THRESHOLD - distance;
	//if we are far, error=PROX_THRESH-a_big_distance will be small
	//if we are close, error=PROX_THRESH-a_small_distance will be big
	extra_speed = pi_regulator(error);
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
