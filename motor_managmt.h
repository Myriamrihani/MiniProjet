
#ifndef MOTOR_MANAGMT_H_
#define MOTOR_MANAGMT_H_

#define ROTATION_THRESHOLD		10
#define MOTOR_SPEED				MOTOR_SPEED_LIMIT/3
#define SEARCH_MOTOR_SPEED		200
#define SEARCH_MAX_COUNTER		70


typedef enum {			///why this enum is not in audio_proc.c?
	DANCE = 0,
	VOICE,
}MODE;					/////too vague, should be called smthing like MICROPHONE_MODE

typedef enum{			///why this enum is not in dance.c?
	FRONT,
	BACK,
	LEFT,
	RIGHT,
	STOP,
}mvmt_robot;		///wrong write, should be in Caps, and maybe call DANCE_MOVE or a funny name to grab that extra point /6

void set_mode(MODE new_mode);
MODE get_mode(void);

void motor_stop(void);
void motor_follow_voice(float angle);
void check_position(void);			///cette fonction n'existe pas
void motor_path_mode(void);
void motor_follow_path(void);
void motor_find_path(void);

#endif /* MOTOR_MANAGMT_H_ */
