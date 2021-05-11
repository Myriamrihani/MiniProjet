#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <arm_math.h>
#include "motors.h"
#include "motor_managmt.h"
#include "camera_processing.h"
#include "obstacles.h"
#include "audio_processing.h"
#include "audio/play_melody.h"


static MODE mode = DANCE;

static int16_t right_speed = 0; 			    // in [step/s]
static int16_t left_speed = 0; 					// in [step/s]
static uint16_t turning_counter = 0;
static int16_t speed_correction = 0;


void set_mode(MODE new_mode){
	mode = new_mode;
}

MODE get_mode(void){
	return mode;
}

void motor_follow_voice(float angle){

	if((angle > 0) & (angle <= 180)){
		right_speed = -MOTOR_SPEED;
		left_speed = MOTOR_SPEED;
	} else if((angle < 0) & (angle > -180)){
		right_speed = MOTOR_SPEED;
		left_speed = -MOTOR_SPEED;
	} else if (angle == 0){
		right_speed = MOTOR_SPEED;
		left_speed = MOTOR_SPEED;
	}

}

void motor_stop(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void motor_path_mode(void) {
	if ((get_listening_voice() == 1)){
    	left_motor_set_speed(right_speed);
    	right_motor_set_speed(left_speed);
	} else {
		moving_the_robot();
	}
}

void moving_the_robot(void){
    int16_t extra_speed = 0;

	if(get_number_of_lines() > 0){
    	turning_counter = 0;
    	set_search_side(NO_SEARCH_SIDE);
		//computes the speed to give to the motors
		extra_speed = get_extra_speed();

	    //computes a correction factor to let the robot rotate to be in front of the line
	    speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
	    //if the line is nearly in front of the camera, don't rotate
	    if(abs(speed_correction) < ROTATION_THRESHOLD){
	           speed_correction = 0;
	    }

	    if(get_listening_voice() == 1){
	    	left_motor_set_speed(right_speed);
	    	right_motor_set_speed(left_speed);
	    } else {
		    //applies the speed from the extra_speed and the correction for the rotation
		    right_motor_set_speed((1+extra_speed)*(MOTOR_SPEED_LIMIT/3 - speed_correction));
		    left_motor_set_speed((1+extra_speed)*(MOTOR_SPEED_LIMIT/3 + speed_correction));
			reset_line();
			change_search_state(true);
	    }
	}
	else{
	    if(get_listening_voice() == 1){
	    	left_motor_set_speed(right_speed);
	    	right_motor_set_speed(left_speed);
	    }
	    else if(turning_counter < 20){
	    	if(speed_correction < 0){
	    		if(get_search_side() == SEARCH_RIGHT){
	    			right_motor_set_speed(-MOTOR_SPEED);
		    		left_motor_set_speed(MOTOR_SPEED);
		    		++turning_counter;
	    		}
	    		else {
	    			right_motor_set_speed(MOTOR_SPEED);
		    		left_motor_set_speed(-MOTOR_SPEED);
		    		++turning_counter;
	    		}
	    	}
	    	else{
	    		if(get_search_side() == SEARCH_LEFT){
	    			right_motor_set_speed(MOTOR_SPEED);
		    		left_motor_set_speed(-MOTOR_SPEED);
		    		++turning_counter;
	    		}
	    		else {
	    			right_motor_set_speed(-MOTOR_SPEED);
		    		left_motor_set_speed(MOTOR_SPEED);
		    		++turning_counter;
	    		}
	    	}
	    }
	    else {
	        chThdSleepMilliseconds(2000);
	    	if(get_listening_voice() == 1){
	    		left_motor_set_speed(right_speed);
	    		right_motor_set_speed(left_speed);
	    	} else {
	    		playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
		    	motor_stop();
		    	speed_correction = 0;
		    	turning_counter = 0;
		    	set_search_side(NO_SEARCH_SIDE);
		        chThdSleepMilliseconds(2000);
		    	set_line_type(NUMBER_OF_LINES);
		    	reset_line();
		    	change_search_state(true);
		    }
	    }
	}
}
