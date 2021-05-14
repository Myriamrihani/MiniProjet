#ifndef OBSTACLES_H_
#define OBSTACLES_H_


typedef enum {
	IR0,
	IR1,
	IR2,
	IR3,
	IR4,
	IR5,
	IR6,
	IR7,
	NO_IR,

}IR_NUMBER;

typedef enum {
	NO_SEARCH_SIDE = 0,
	SEARCH_LEFT,
	SEARCH_RIGHT,

}SEARCHING_SIDE;


void find_proximity(void);
void manual_speed(uint16_t distance);
void set_search_side(SEARCHING_SIDE side);
SEARCHING_SIDE get_search_side(void);

//Function that only focuses on IR3 and IR4 - the sensors placed at the back
int16_t get_extra_speed(void);


#endif /* OBSTACLES_H_ */
