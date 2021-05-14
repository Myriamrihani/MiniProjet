#ifndef MOTOR_MANAGMT_H_
#define MOTOR_MANAGMT_H_


#define MOTOR_SPEED				MOTOR_SPEED_LIMIT/3 		// in [step/s]


typedef enum{
	FRONT,
	BACK,
	LEFT,
	RIGHT,
	STOP,
}MVMT_ROBOT;


//Function that allows the robot to move towards the detected voice
void motor_follow_voice(float angle);

//Function that manages the motor commands when following a path
void motor_path_mode(void);

//Function allows the robot to rotate left or right to find sharp turns in the path
void motor_find_path(void);

void motor_follow_path(void);
void motor_stop(void);


#endif /* MOTOR_MANAGMT_H_ */
