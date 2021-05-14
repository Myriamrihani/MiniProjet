#ifndef CAMERA_PROCESSING_H_
#define CAMERA_PROCESSING_H_


#define IMAGE_BUFFER_SIZE		640


typedef enum {
	NO_LINE_TYPE = 0,
	NUMBER_OF_LINES,
	LINE_POSITION,

}LINE_TYPE_EXTRACT;


uint16_t get_line_position(void);
uint8_t get_number_of_lines(void);
LINE_TYPE_EXTRACT get_line_type(void);
void set_line_search_state(bool new_state);
void set_line_type(LINE_TYPE_EXTRACT type);
void reset_line(void);
void process_image_start(void);
void wait_image_detected(void);


#endif /* CAMERA_PROCESSING_H_ */
