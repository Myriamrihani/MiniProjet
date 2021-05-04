#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <arm_math.h>
#include "motors.h"
#include "motor_managmt.h"

#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

static MODE mode = DANCE;

static int16_t right_speed = 0; 			    // in [step/s]
static int16_t left_speed = 0; 					// in [step/s]
//static int16_t counter_step_right = 0;          // in [step]
//static int16_t counter_step_left = 0;
static float perimeter = 0;
static int16_t pos_r = 0;
static int16_t pos_l = 0;


void set_mode(MODE new_mode){
	mode = new_mode;
}

MODE get_mode(void){
	return mode;
}

void motor_take_direction(float angle){
	perimeter = angle * PI * WHEEL_DISTANCE / 360;

	pos_r = -perimeter * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	pos_l = perimeter * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	if((angle > 0) & (angle <= 180)){
		right_speed = -400;
		left_speed = 400;
	} else if((angle < 0) & (angle > -180)){
		right_speed = 400;
		left_speed = -400;
	} else if (angle == 0){
		right_speed = 400;
		left_speed = 400;
	}

	left_motor_set_speed(right_speed);
	right_motor_set_speed(left_speed);
}

void motor_stop(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

