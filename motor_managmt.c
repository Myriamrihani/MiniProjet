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
		right_speed = 0;
		left_speed = 0;
	}

	left_motor_set_speed(right_speed);
	right_motor_set_speed(left_speed);
}

//void check_position(void){
//	uint8_t i = 0;
//    if ((abs(counter_step_right) > abs(pos_r))) {
//    	counter_step_right = 0;
//    	right_motor_set_pos(counter_step_right);
//    	left_motor_set_speed(0);
//    	right_motor_set_speed(0);
//    }
//    else {
//        if (right_speed > 0) {
//            i = (i + 1) & 3;
//            counter_step_right++;
//        	right_motor_set_speed(0);
//        } else if (right_speed < 0) {
//            i = (i - 1) & 3;
//            right_motor_update(step_table[i]);
//            counter_step_right--;
//        	right_motor_set_pos(counter_step_right);
//        } else {
//            right_motor_update(step_halt);
//        }
//    }
//
//	uint8_t j = 0;
//
//    // Check if position is reached
//    if ((abs(counter_step_left) > abs(pos_l))) {
//    	counter_step_left = 0;
//    	left_motor_set_pos(counter_step_left);
//    	left_motor_update(step_halt);
//    }
//    else {
//        if (left_speed > 0) {
//            j = (j + 1) & 3;
//            left_motor_update(step_table[j]);
//            counter_step_left++;
//        	left_motor_set_pos(counter_step_left);
//        } else if (left_speed < 0) {
//            j = (j - 1) & 3;
//            left_motor_update(step_table[j]);
//            counter_step_left--;
//        	left_motor_set_pos(counter_step_left);
//        } else {
//            left_motor_update(step_halt);
//        }
//    }
//}
