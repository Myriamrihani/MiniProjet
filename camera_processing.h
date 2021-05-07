/*
 * camera_processing.h
 *
 *      Author: Bryan Kheirallah
 */

#ifndef CAMERA_PROCESSING_H_
#define CAMERA_PROCESSING_H_

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			30					//fingers are small, we might reduce this number
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f

typedef enum {
	NO_LINE_TYPE = 0,
	NUMBER_OF_LINES,
	LINE_POSITION,

}LINE_TYPE_EXTRACT;

void SendImageToSystem(uint8_t* data, uint16_t size);		////NOT USED////
float get_distance_cm(void);
uint16_t get_line_position(void);
uint8_t get_number_of_lines(void);
void change_search_state(bool new_state);
bool get_search_state(void);
void process_image_start(void);
void wait_image_detected(void);
bool line_is_searching(void);
void reset_line(void);
void set_line_type(LINE_TYPE_EXTRACT type);
LINE_TYPE_EXTRACT get_line_type(void);
void pi_regulator_start(void);
bool get_line_found(void);

#endif /* CAMERA_PROCESSING_H_ */
