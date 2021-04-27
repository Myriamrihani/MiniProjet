/*
 * obstacles.h
 *
 *  Created on: 21 Apr 2021
 *      Author: Bryan Kheirallah
 */

#ifndef OBSTACLES_H_
#define OBSTACLES_H_


#define PROXIMITY_THRESHOLD			50
#define ROTATION_COEFF				2		//hyper random,might change

void find_proximity(void);
void find_obstacles(void);
void avoid_obstacle(uint8_t sensor, uint16_t distance);
void manual_speed(uint8_t sensor, uint16_t distance);

#endif /* OBSTACLES_H_ */
