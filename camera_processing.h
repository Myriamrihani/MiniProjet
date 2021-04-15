/*
 * camera_processing.h
 *
 *      Author: Bryan Kheirallah
 */

#ifndef CAMERA_PROCESSING_H_
#define CAMERA_PROCESSING_H_

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40					//fingers are small, we might reduce this number
//#define ROTATION_THRESHOLD		10				//left them in comment cz will be useful later
//#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
//#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
//#define KP						800.0f
//#define KI 						3.5f	//must not be zero
//#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

void SendImageToSystem(uint8_t* data, uint16_t size);
float get_distance_cm(void);
uint16_t get_line_position(void);
uint8_t get_number_of_lines(void);
void process_image_start(void);
void wait_image_detected(void);
bool line_is_searching(void);



#endif /* CAMERA_PROCESSING_H_ */
