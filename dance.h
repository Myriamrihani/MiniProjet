/*
 * dance.h
 *
 *  Created on: 8 Apr 2021
 *      Author: myriamrihani
 */

#ifndef DANCE_H_
#define DANCE_H_

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sensors/mpu9250.h"
#include "sensors/imu.h"
#include "msgbus/messagebus.h"
#include <i2c_bus.h>

#define NB_PAS 4

typedef enum{
	FRONT,
	BACK,
	LEFT,
	RIGHT,
}mvmt_robot;

void dance_start(void);
void show_gravity(imu_msg_t *imu_values);
void dancing(void);
bool dance_memorized(void);




#endif /* DANCE_H_ */
