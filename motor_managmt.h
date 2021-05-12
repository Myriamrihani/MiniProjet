
#ifndef MOTOR_MANAGMT_H_
#define MOTOR_MANAGMT_H_

#define ROTATION_THRESHOLD		10
#define MOTOR_SPEED				MOTOR_SPEED_LIMIT/3
#define SEARCH_MOTOR_SPEED		200
#define SEARCH_MAX_COUNTER		70



typedef enum{
	FRONT,
	BACK,
	LEFT,
	RIGHT,
	STOP,
}MVMT_ROBOT;


void motor_stop(void);
void motor_follow_voice(float angle);
void motor_path_mode(void);
void motor_follow_path(void);
void motor_find_path(void);

#endif /* MOTOR_MANAGMT_H_ */
