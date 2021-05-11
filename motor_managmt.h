
#ifndef MOTOR_MANAGMT_H_
#define MOTOR_MANAGMT_H_

#define ROTATION_THRESHOLD		10
#define MOTOR_SPEED				MOTOR_SPEED_LIMIT/3


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

void motor_stop(void);
void motor_follow_voice(float angle);
void check_position(void);
void motor_path_mode(void);
void moving_the_robot(void);

#endif /* MOTOR_MANAGMT_H_ */
