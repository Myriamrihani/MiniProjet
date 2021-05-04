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
#include "camera_processing.h"
#include "audio/play_melody.h"
#include "audio/audio_thread.h"
#include "selector.h"
#include <obstacles.h>
#include "motor_managmt.h"




static mvmt_robot tilt;
static uint8_t count_step = 0;
static uint8_t nb_pas = 0;
static uint8_t freq_counter = 0;

static mvmt_robot dance_memo[NB_PAS] = {STOP};

static bool dance_memo_complete = 0;
static bool dance_cleared = 1;

static FREQUENCY_TO_DETECT past_freq = 0;

void set_nb_pas(uint8_t nb){
	nb_pas = nb;
}

//function that fills the dancing vector to memorize
void fill_dance(imu_msg_t *imu_values){

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
  	}
}

bool get_dance_memo_complete(void){
	return dance_memo_complete;
}

//function that takes memorized dance and starts the motors
void dancing(void){
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
		reset_dance();
        chThdSleepMilliseconds(3000);
        set_line_type(LINE_POSITION);
		change_search_state(true);	//////////////////

	}
}

void dance(FREQUENCY_TO_DETECT freq, imu_msg_t *imu_values){
	++freq_counter;

	if(freq_counter == 1) {
		past_freq = freq;
	}
	set_line_type(NUMBER_OF_LINES);

	if(past_freq == get_frequency()){ //we want to make sure that we did not change the frequency type

		if(get_number_of_lines() > 0) {
			set_mode(DANCE);
			set_nb_pas(get_number_of_lines());
			change_search_state(false);

			chprintf((BaseSequentialStream *)&SD3, "nb lines in dance  : %d \r\n" , get_number_of_lines());
			chprintf((BaseSequentialStream *)&SD3, "start dance  : %d \r\n" , get_start_dance());

			if(get_dance_memo_complete() == 1){
				wait_start_signal();
				chprintf((BaseSequentialStream *)&SD3, "start dance  : %d \r\n" , get_start_dance());
			    if (get_start_dance() == 1) {
			    	if(freq == WOMAN) {playMelody(MARIO, ML_SIMPLE_PLAY, NULL);}
			    	if(freq == MAN) {playMelody(RUSSIA, ML_SIMPLE_PLAY, NULL);}
			    	dancing();
			    }
			} else  if (is_dance_clear()) {fill_dance(imu_values);}
		} else if((get_number_of_lines() == 0)){
			change_search_state(true);
		    chThdSleepMilliseconds(2000);
		}
	} else {
	    chprintf((BaseSequentialStream *)&SD3, "changed frequency \r\n");
		freq_counter = 0;
		reset_dance();
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

	for(int i = 0; i<nb_pas; i++){
		chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[i]);
	}
}

void reset_dance(void){
	clear_dance();
	stopCurrentMelody();
	display_dance();
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	count_step = 0;
	reset_line();
	nb_pas = 0;
	dance_memo_complete = 0;
	chprintf((BaseSequentialStream *)&SD3, "start dance  : %d \r\n" , get_start_dance());
	set_start_dance(0);
	chprintf((BaseSequentialStream *)&SD3, "start dance  : %d \r\n" , get_start_dance());
	//set_mode(VOICE);
}

uint8_t get_nb_pas(void){
	return nb_pas;
}

