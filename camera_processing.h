#ifndef CAMERA_PROCESSING_H_
#define CAMERA_PROCESSING_H_

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			30
#define CAMERA_POSITION			478

typedef enum {
	NO_LINE_TYPE = 0,
	NUMBER_OF_LINES,
	LINE_POSITION,

}LINE_TYPE_EXTRACT;


uint16_t get_line_position(void);
uint8_t get_number_of_lines(void);
void set_search_state(bool new_state);
bool get_search_state(void);
void process_image_start(void);
void wait_image_detected(void);
void reset_line(void);
void set_line_type(LINE_TYPE_EXTRACT type);
LINE_TYPE_EXTRACT get_line_type(void);
bool get_line_found(void);

#endif /* CAMERA_PROCESSING_H_ */
