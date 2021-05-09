/*
 * obstacles.h
 *
 *  Created on: 21 Apr 2021
 *      Author: Bryan Kheirallah
 */

#ifndef OBSTACLES_H_
#define OBSTACLES_H_


#define PROXIMITY_THRESHOLD		100
#define ROTATION_COEFF			2		//hyper random,might change
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

typedef enum {
	IR0,
	IR1,
	IR2,
	IR3,
	IR4,
	IR5,
	IR6,
	IR7,
	NO_IR,

}IR_NUMBER;

typedef enum {
	NO_SEARCH_SIDE = 0,
	SEARCH_LEFT,
	SEARCH_RIGHT,

}SEARCHING_SIDE;

void find_proximity(void);
void manual_speed(uint16_t distance);
int16_t get_extra_speed(void);
SEARCHING_SIDE get_search_side(void);
void set_search_side(SEARCHING_SIDE side);

#endif /* OBSTACLES_H_ */
