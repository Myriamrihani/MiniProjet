/*
 * moto_managmt.h
 *
 *  Created on: 27 Apr 2021
 *      Author: myriamrihani
 */

#ifndef MOTOR_MANAGMT_H_
#define MOTOR_MANAGMT_H_

typedef enum {
	DANCE = 0,
	VOICE,
}MODE;

typedef enum{
	FRONT,
	BACK,
	LEFT,
	RIGHT,
	STOP,
}mvmt_robot;

void set_mode(MODE new_mode);
MODE get_mode(void);

void motor_take_direction(float angle);
void check_position(void);

#endif /* MOTOR_MANAGMT_H_ */
