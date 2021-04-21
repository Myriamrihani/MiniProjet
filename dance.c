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
#include "audio_processing.h"
#include "camera_processing.h"
#include "audio/play_melody.h"
#include "audio/audio_thread.h"


static mvmt_robot tilt;
static int count_step = 0;
static uint8_t nb_pas = 0;

static mvmt_robot dance_memo[NB_PAS] = {STOP};


float acc_x = 0;
float acc_y = 0;

static bool dance_memo_complete = 0;
static bool dance_cleared = 1;

//static thread_t *danceThd;
//
//static THD_WORKING_AREA(waDance, 128);
//static THD_FUNCTION(Dance, arg) {
//	(void) arg;
//    chRegSetThreadName(__FUNCTION__);
//
//    while(chThdShouldTerminateX() == false){
//        chThdSleepMilliseconds(1000);
//		chprintf((BaseSequentialStream *)&SD3, "dance thread \r\n");
//    }
//
//}

void set_nb_pas(uint8_t nb){
	nb_pas = nb;
}

void dance_start(void){
    imu_start();
    motors_init();

//	danceThd = chThdCreateStatic(waDance, sizeof(waDance), NORMALPRIO, Dance, NULL);
}

void set_acc(float imu_ac_x, float imu_ac_y){
	acc_x = imu_ac_x;
	acc_y = imu_ac_y;
}



void imu_display(imu_msg_t imu_values)
{
    chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
            imu_values.acc_raw[X_AXIS], imu_values.acc_raw[Y_AXIS], imu_values.acc_raw[Z_AXIS],
            imu_values.gyro_raw[X_AXIS], imu_values.gyro_raw[Y_AXIS], imu_values.gyro_raw[Z_AXIS]);

    //prints raw values with offset correction
    chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
            imu_values.acc_raw[X_AXIS]-imu_values.acc_offset[X_AXIS],
            imu_values.acc_raw[Y_AXIS]-imu_values.acc_offset[Y_AXIS],
            imu_values.acc_raw[Z_AXIS]-imu_values.acc_offset[Z_AXIS],
            imu_values.gyro_raw[X_AXIS]-imu_values.gyro_offset[X_AXIS],
            imu_values.gyro_raw[Y_AXIS]-imu_values.gyro_offset[Y_AXIS],
            imu_values.gyro_raw[Z_AXIS]-imu_values.gyro_offset[Z_AXIS]);

    //prints values in readable units
    chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n",
            imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
            imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
            imu_values.status);
}

//function that fills the dancing vector to memorize
void show_gravity(void){
	chprintf((BaseSequentialStream *)&SD3, " gravity \r\n");

       if(count_step == 0){
	    	palSetPad(GPIOD, GPIOD_LED1);
			palSetPad(GPIOD, GPIOD_LED3);
			palSetPad(GPIOD, GPIOD_LED5);
			palSetPad(GPIOD, GPIOD_LED7);
       }

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
        if (count_step >= nb_pas){
        	count_step = 0;
        	dance_memo_complete = 1;
        	dance_cleared = 0;
        	//chprintf((BaseSequentialStream *)&SD3, "complete  : %d \r\n" , dance_memo_complete);
     	}

        //display_dance();

}

bool get_dance_memo_complete(void){
	return dance_memo_complete;
}

//function that takes memorized dance and starts the motors
void dancing(void){

//    chprintf((BaseSequentialStream *)&SD3, "count : %d \r\n" , count_step);
//	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[count_step]);

	if ( count_step <= nb_pas-1) {
		if(dance_memo[count_step] == FRONT) {
			left_motor_set_speed(600);
			right_motor_set_speed(600);
		} else if(dance_memo[count_step] == BACK) {
			left_motor_set_speed(-600);
			right_motor_set_speed(-600);
		} else if(dance_memo[count_step] == RIGHT) {
			left_motor_set_speed(600);
			right_motor_set_speed(-600);
		} else if(dance_memo[count_step] == LEFT) {
			left_motor_set_speed(-600);
			right_motor_set_speed(600);
		} else if(dance_memo[count_step] == STOP) {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
		count_step++;
	} else {
    	//chprintf((BaseSequentialStream *)&SD3, "dancing \r\n");

		stopCurrentMelody();
		//chprintf((BaseSequentialStream *)&SD3, "AVANT : \r\n" );

		display_dance();
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		count_step = 0;
		reset_dance();
    	reset_line();
    	nb_pas = 0;
    	//chprintf((BaseSequentialStream *)&SD3, "APRES : \r\n" );

    	//display_dance();
	}

}

void clear_dance(void){

	for(int i = 0; i<nb_pas; i++){
		dance_memo[i] = STOP;
	}
	dance_cleared = 1;
}

bool is_dance_clear(void){
	return (dance_cleared & (nb_pas != 0));
}

void display_dance(void){
//	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[0]);
//	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[1]);
//	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[2]);
//	chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[3]);

	for(int i = 0; i<nb_pas; i++){
		chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[i]);
	}
}

void reset_dance(void){
	clear_dance();

	dance_memo_complete = 0;
	set_start_dance(0);
}

uint8_t get_nb_pas(void){
	return nb_pas;
}

