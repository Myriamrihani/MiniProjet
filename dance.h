#ifndef DANCE_H_
#define DANCE_H_

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sensors/mpu9250.h"
#include "sensors/imu.h"
#include "msgbus/messagebus.h"
#include <i2c_bus.h>
#include <audio_processing.h>

#define NB_PAS 10

#define STANDARD_GRAVITY    9.80665f
#define DEG2RAD(deg) (deg / 180 * M_PI)
#define GRAVITY_CALIBRATION 32768.0f
#define GYRO_CALIBRATION (3.814f/1000.0f)
#define NB_SAMPLES_OFFSET     200
#define MIN_GRAV_VALUE (STANDARD_GRAVITY*0.1)

void set_nb_pas(uint8_t nb);
uint8_t get_nb_pas(void);


//Function that uses IMU to fill the dance moves into the dance vector
void fill_dance(imu_msg_t *imu_values);

//Function that executes the dances with the motor
void dancing(void);

bool get_dance_memo_complete(void);

//Main Function of the dance module
void dance(imu_msg_t *imu_values);

//clears the dance vector
void clear_dance(void);

bool is_dance_clear(void);

//displays the dance vector -> used for debugging
void display_dance(void);

//resets all parameters for the dance module
void reset_dance(void);

#endif /* DANCE_H_ */
