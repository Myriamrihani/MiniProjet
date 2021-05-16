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


#define NB_STEPS 				10
#define STANDARD_GRAVITY   		9.80665f
#define DEG2RAD(deg) 			(deg / 180 * M_PI)
#define GRAVITY_CALIBRATION 	32768.0f
#define GYRO_CALIBRATION 		(3.814f/1000.0f)
#define NB_SAMPLES_OFFSET     	200
#define MIN_GRAV_VALUE 			(STANDARD_GRAVITY*0.1)


static uint8_t count_step = 0;
static uint8_t nb_steps = 0;
static MVMT_ROBOT tilt;
static MVMT_ROBOT dance_memo[NB_STEPS] = {STOP};
static bool dance_memo_complete = false;
static bool dance_cleared = true;


////Function that uses IMU to fill the dance moves into the dance vector
void fill_dance(imu_msg_t *imu_values){

    float acc_x = imu_values->acceleration[X_AXIS];
    float acc_y = imu_values->acceleration[Y_AXIS];

    if(count_step == 0){
	    	palSetPad(GPIOD, GPIOD_LED1);
			palSetPad(GPIOD, GPIOD_LED3);
			palSetPad(GPIOD, GPIOD_LED5);
			palSetPad(GPIOD, GPIOD_LED7);
    }

    if((fabs(acc_x) < MIN_GRAV_VALUE) & (fabs(acc_y) < MIN_GRAV_VALUE))
    {
    	//turn off all the LEDs as the robot is straight flat
    	palSetPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		return;
    }

    //robot is rotating with respect to y, thus acceleration is in x
    if(fabs(acc_x) > fabs(acc_y)){
    	palSetPad(GPIOD, GPIOD_LED1);
    	palSetPad(GPIOD, GPIOD_LED5);

    	if(acc_x > 0){
    		//LEFT
    		palClearPad(GPIOD, GPIOD_LED7);
    		palSetPad(GPIOD, GPIOD_LED3);
    		tilt = LEFT;
    	} else{
    		//RIGHT
    		palClearPad(GPIOD, GPIOD_LED3);
    		palSetPad(GPIOD, GPIOD_LED7);
    		tilt = RIGHT;
    	}

    } else{
    //robot is rotating with respect to x, thus acceleration is in y
        palSetPad(GPIOD, GPIOD_LED3);
       	palSetPad(GPIOD, GPIOD_LED7);

        if(acc_y < 0){
        	//FRONT
        	palClearPad(GPIOD, GPIOD_LED1);
        	palSetPad(GPIOD, GPIOD_LED5);
        	tilt = FRONT;
        } else{
        	//BACK
        	palClearPad(GPIOD, GPIOD_LED5);
        	palSetPad(GPIOD, GPIOD_LED1);
        	tilt = BACK;
        }
    }

    dance_memo[count_step] = tilt;
    count_step++;
    if (count_step >= nb_steps){
    	count_step = 0;
     	dance_memo_complete = true;
     	dance_cleared = false;
  	}
}


bool get_dance_memo_complete(void){
	return dance_memo_complete;
}

//Function that takes memorized dance and starts the motors
void dancing(void){
	if ( count_step <= nb_steps-1) {
		if(dance_memo[count_step] == FRONT) {
			left_motor_set_speed(MOTOR_SPEED);
			right_motor_set_speed(MOTOR_SPEED);
		} else if(dance_memo[count_step] == BACK) {
			left_motor_set_speed(-MOTOR_SPEED);
			right_motor_set_speed(-MOTOR_SPEED);
		} else if(dance_memo[count_step] == RIGHT) {
			left_motor_set_speed(MOTOR_SPEED);
			right_motor_set_speed(-MOTOR_SPEED);
		} else if(dance_memo[count_step] == LEFT) {
			left_motor_set_speed(-MOTOR_SPEED);
			right_motor_set_speed(MOTOR_SPEED);
		} else if(dance_memo[count_step] == STOP) {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
		count_step++;
	} else{
		reset_dance();
        chThdSleepMilliseconds(3000);
        set_line_type(LINE_POSITION);
		set_line_search_state(true);
		set_mic_mode(VOICE);
	}
}


void dance(imu_msg_t *imu_values){
	if(get_number_of_lines() > 0) {
		palSetPad(GPIOB, GPIOB_LED_BODY);
		set_mic_mode(DANCE);
		nb_steps = get_number_of_lines();
		set_line_search_state(false);

		if(get_dance_memo_complete() == true){
			wait_start_signal();
		    if (get_start_dance() == true) {
		    	playMelody(MARIO, ML_SIMPLE_PLAY, NULL);
		    	dancing();
		    }
		} else if(is_dance_clear()) {fill_dance(imu_values);}
	} else if((get_number_of_lines() == 0)){
		stopCurrentMelody();
		set_line_search_state(true);
	    chThdSleepMilliseconds(2000);
	}
}


void clear_dance(void){
	for(int i = 0; i<nb_steps; i++){
		dance_memo[i] = STOP;
	}
	dance_cleared = 1;
}


bool is_dance_clear(void){
	return (dance_cleared & (nb_steps != 0));
}


void display_dance(void){
	for(int i = 0; i<nb_steps; i++){
		chprintf((BaseSequentialStream *)&SD3, "dance  : %d \r\n" , dance_memo[i]);
	}
}


void reset_dance(void){
	clear_dance();
	stopCurrentMelody();
	motor_stop();
	count_step = 0;
	reset_line();
	nb_steps = 0;
	dance_memo_complete = false;
	set_start_dance(false);
	palClearPad(GPIOB, GPIOB_LED_BODY);
}



