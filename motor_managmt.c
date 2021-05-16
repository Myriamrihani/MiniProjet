#include "ch.h"
#include "hal.h"
#include <main.h>
#include <chprintf.h>
#include <arm_math.h>
#include "motors.h"
#include "motor_managmt.h"
#include "camera_processing.h"
#include "ir_interference.h"
#include "audio_processing.h"
#include "audio/play_melody.h"


#define SEARCH_ROTATION_SPEED	200							// in [step/s]
#define SEARCH_MAX_COUNTER		60
#define ROTATION_THRESHOLD		10


static int16_t right_speed = 0; 			   				// in [step/s]
static int16_t left_speed = 0; 								// in [step/s]
static uint16_t turning_counter = 0;
static int16_t speed_correction = 0;
static MIC_MODE mode = DANCE;


void set_mic_mode(MIC_MODE new_mode){
	mode = new_mode;
}


MIC_MODE get_mic_mode(void){
	return mode;
}


void motor_follow_voice(float angle){

	if((angle > 0) & (angle <= 180)){
		left_speed = MOTOR_SPEED;
		right_speed = -MOTOR_SPEED;
	} else if((angle < 0) & (angle > -180)){
		left_speed = -MOTOR_SPEED;
		right_speed = MOTOR_SPEED;
	} else if (angle == 0){
		left_speed = MOTOR_SPEED;
		right_speed = MOTOR_SPEED;
	}
}


void motor_stop(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}


void motor_path_mode(void) {
	if((get_listening_voice() == 1)){
		left_motor_set_speed(right_speed);
    	right_motor_set_speed(left_speed);
	} else{
		if(get_number_of_lines() > 0){
			motor_follow_path();
		} else{
			motor_find_path();
		}
	}
}


void motor_follow_path(void){
	int16_t extra_speed = 0;
	turning_counter = 0;
	set_search_side(NO_SEARCH_SIDE);

	//computes the speed a user want to add to the motors
	extra_speed = get_extra_speed();

    //computes a correction factor to let the robot rotate to be in front of the line
	speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
	//if the line is nearly in front of the camera, don't rotate
	if(abs(speed_correction) < ROTATION_THRESHOLD){
		speed_correction = 0;
	}

	//applies the speed from the extra_speed and the correction for the rotation
	left_motor_set_speed((1+extra_speed)*(MOTOR_SPEED + speed_correction));
	right_motor_set_speed((1+extra_speed)*(MOTOR_SPEED - speed_correction));

	reset_line();
	set_line_search_state(true);
}


void motor_find_path(void){

	//turning_counter prevents the robot from rotating by 180degres
	//and going back on the same line in the opposite direction
	if(turning_counter < SEARCH_MAX_COUNTER){
		 SEARCHING_SIDE side = get_search_side();

		 if(speed_correction < 0){
			 //the user can force the rotation to a side or another,
			 //for example at the intersection of 2 paths
			 if(side == SEARCH_RIGHT){
				 left_motor_set_speed(SEARCH_ROTATION_SPEED);
				 right_motor_set_speed(-SEARCH_ROTATION_SPEED);
				 ++turning_counter;
			 } else {
				 left_motor_set_speed(-SEARCH_ROTATION_SPEED);
				 right_motor_set_speed(SEARCH_ROTATION_SPEED);
				 ++turning_counter;
			 }
		 } else{
			 if(side == SEARCH_LEFT){
				 left_motor_set_speed(-SEARCH_ROTATION_SPEED);
				 right_motor_set_speed(SEARCH_ROTATION_SPEED);
				 ++turning_counter;
			 } else {
				 left_motor_set_speed(SEARCH_ROTATION_SPEED);
				 right_motor_set_speed(-SEARCH_ROTATION_SPEED);
				 ++turning_counter;
			 }
		 }
	 } else{
		 //a 2 seconds pause allows us to make, if wanted, one last vocal command
		 chThdSleepMilliseconds(2000);
		 playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
		 motor_stop();
		 speed_correction = 0;
		 turning_counter = 0;
		 set_search_side(NO_SEARCH_SIDE);

		 //a 2 seconds pause gives us enough time to choose
		 //the number of dance steps for a new dance
		 chThdSleepMilliseconds(2000);
		 set_line_type(NUMBER_OF_LINES);
		 reset_line();
		 set_line_search_state(true);
	 }
}
