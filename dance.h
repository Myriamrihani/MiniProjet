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
