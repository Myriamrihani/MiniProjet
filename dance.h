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

#define NB_PAS 2

#define STANDARD_GRAVITY    9.80665f
#define DEG2RAD(deg) (deg / 180 * M_PI)
#define GRAVITY_CALIBRATION 32768.0f
#define GYRO_CALIBRATION (3.814f/1000.0f)
#define NB_SAMPLES_OFFSET     200
#define MIN_GRAV_VALUE (STANDARD_GRAVITY*0.1)

typedef enum{
	FRONT,
	BACK,
	LEFT,
	RIGHT,
	STOP,
}mvmt_robot;

void set_nb_pas(uint8_t nb);
void dance_start(void);
void set_acc(float imu_ac_x, float imu_ac_y);
void show_gravity(void);
void dancing(void);
bool get_dance_memo_complete(void);
void clear_dance(void);
bool is_dance_clear(void);
void display_dance(void);
void reset_dance(void);
uint8_t get_nb_pas(void);

#endif /* DANCE_H_ */
