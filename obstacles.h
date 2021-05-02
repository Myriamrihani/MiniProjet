/*
 * obstacles.h
 *
 *  Created on: 21 Apr 2021
 *      Author: Bryan Kheirallah
 */

#ifndef OBSTACLES_H_
#define OBSTACLES_H_


#define PROXIMITY_THRESHOLD			100
#define ROTATION_COEFF				2		//hyper random,might change
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

typedef enum {
	NEITHER = 0,
	IR0,
	IR1,
	IR2,
	IR3,
	IR4,
	IR5,
	IR6,
	IR7,

}IR_NUMBER;

int16_t find_proximity(void);
void find_obstacles(void);
void avoid_obstacle(uint8_t sensor, uint16_t distance);
int16_t pi_regulator(int16_t error);
int16_t manual_speed(uint8_t sensor, uint16_t distance);

#endif /* OBSTACLES_H_ */
