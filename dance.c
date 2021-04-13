/*
 * dance.c
 *
 *  Created on: 8 Apr 2021
 *      Author: myriamrihani
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ch.h>
#include <hal.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <dance.h>
#include <chprintf.h>
#include "motors.h"


static mvmt_robot tilt;
static int count_step = 0;
mvmt_robot dance_memo[NB_PAS] = {0};


static bool dance_memo_complete = 0;


static thread_t *danceThd;

static THD_WORKING_AREA(waDance, 128);
static THD_FUNCTION(Dance, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);



    while(chThdShouldTerminateX() == false){
        chThdSleepMilliseconds(1000);


    }

}


void dance_start(void){
    imu_start();

	danceThd = chThdCreateStatic(waDance, sizeof(waDance), NORMALPRIO, Dance, NULL);
}

//function that fills the dancing vector to memorize
void show_gravity(imu_msg_t *imu_values){

    //variable to measure the time some functions take
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time = 0;

    /*
    *   Use this to reset the timer counter and prevent the system
    *   to switch to another thread.
    *   Place it at the beginning of the code you want to measure
    */
    chSysLock();
    //reset the timer counter
    GPTD11.tim->CNT = 0;

    float acc_x = imu_values->acceleration[X_AXIS];
    float acc_y = imu_values->acceleration[Y_AXIS];

    /*
       *   Use this to capture the counter and stop to prevent
       *   the system to switch to another thread.
       *   Place it at the end of the code you want to measure
       */
       time = GPTD11.tim->CNT;
       chSysUnlock();

	    if((fabs(acc_x) < MIN_GRAV_VALUE) & (fabs(acc_y) < MIN_GRAV_VALUE))
	    {
	    	//on etient toutes les LED car le robot est tout droit
	    	palSetPad(GPIOD, GPIOD_LED1);
			palSetPad(GPIOD, GPIOD_LED3);
			palSetPad(GPIOD, GPIOD_LED5);
			palSetPad(GPIOD, GPIOD_LED7);
			return;
	    }

	    //le robot est en rotation selon y, donc une acceleration en x
	    if(fabs(acc_x) > fabs(acc_y))
	    {
	    	palSetPad(GPIOD, GPIOD_LED1);
	    	palSetPad(GPIOD, GPIOD_LED5);

	    	if(acc_x > 0)
	    	{
	    		//GAUCHE
	    		palClearPad(GPIOD, GPIOD_LED7);
	    		palSetPad(GPIOD, GPIOD_LED3);
	    		tilt = LEFT;
	    	}
	    	else
	    	{
	    		//DROITE
	    		palClearPad(GPIOD, GPIOD_LED3);
	    		palSetPad(GPIOD, GPIOD_LED7);
	    		tilt = RIGHT;
	    	}

	    }else
	    {
	    	//le robot est en rotation selon x, donc une acceleration en y
	        palSetPad(GPIOD, GPIOD_LED3);
	       	palSetPad(GPIOD, GPIOD_LED7);

	        if(acc_y < 0)
	        {
	        	//FRONT
	        	palClearPad(GPIOD, GPIOD_LED1);
	        	palSetPad(GPIOD, GPIOD_LED5);
	        	tilt = FRONT;
	        }else
	        {
	        	//BACK
	        	palClearPad(GPIOD, GPIOD_LED5);
	        	palSetPad(GPIOD, GPIOD_LED1);
	        	tilt = BACK;
	        }
	    }


        chprintf((BaseSequentialStream *)&SD3, "count : %d \r\n" , count_step);
        chprintf((BaseSequentialStream *)&SD3, "tilt : %d \r\n" , tilt);

        dance_memo[count_step] = tilt;
        count_step++;
        if (count_step >= NB_PAS){
        	count_step = 0;
        	dance_memo_complete = 1;
        	chprintf((BaseSequentialStream *)&SD3, "complete  : %d \r\n" , dance_memo_complete);
     	}

    	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[0]);
    	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[1]);
    	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[2]);
    	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[3]);


}

bool dance_memorized(void){
	return dance_memo_complete;
}

//function that takes memorized dance and starts the motors
void dancing(void){
	for(int i = 0; i < NB_PAS; i++)
	{
    	chprintf((BaseSequentialStream *)&SD3, "dance  : \r\n" , dance_memo[i]);

		if(dance_memo[i] == FRONT) {
			left_motor_set_speed(600);
			right_motor_set_speed(600);
		} else if(dance_memo[i] == BACK) {
			left_motor_set_speed(-600);
			right_motor_set_speed(-600);
		} else if(dance_memo[i] == RIGHT) {
			left_motor_set_speed(600);
			right_motor_set_speed(-600);
		} else if(dance_memo[i] == LEFT) {
			left_motor_set_speed(-600);
			right_motor_set_speed(600);
		}

//		switch(dance_memo[i])
//		{
//			case FRONT:
//				left_motor_set_speed(600);
//				right_motor_set_speed(600);
//		        chThdSleepMilliseconds(1000);
//				break;
//			case BACK:
//				left_motor_set_speed(-600);
//				right_motor_set_speed(-600);
//		        chThdSleepMilliseconds(1000);
//				break;
//			case RIGHT:
//				left_motor_set_speed(600);
//				right_motor_set_speed(-600);
//		        chThdSleepMilliseconds(1000);
//				break;
//			case LEFT:
//				left_motor_set_speed(-600);
//				right_motor_set_speed(600);
//		        chThdSleepMilliseconds(1000);
//				break;
//			default:
//				left_motor_set_speed(0);
//				right_motor_set_speed(0);
//				break;
//		}
	}
}