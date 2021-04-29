/*
 * camera_processing.c
 *
 *      Author: Bryan Kheirallah
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <usbcfg.h>				//this one is not necessarily useful

#include <math.h>
#include "memory_protection.h"
#include <camera/po8030.h>
#include "motors.h"
#include <camera_processing.h>
#include <obstacles.h>
#include <camera/dcmi_camera.h>


static float distance_cm = 0;
static uint8_t number_of_lines = 0;					//very important!
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static bool searching_for_lines = false;

static LINE_TYPE_EXTRACT line_type = NOPE;
static uint16_t width = 0; //better if we can put argument to threads

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


void SendImageToSystem(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

/*
 *  Computes the line's width extracted from the image buffer given
 *  Gives 0 if line not found
 */
void extract_line(uint8_t *buffer, bool searching_for_lines){


	uint16_t i = 0, line_beginning = 0, line_ending = 0;
	uint8_t stop_line_limit_search = 0, wrong_line = 0, line_found = 1;
	uint32_t mean = 0;
	width = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	if(searching_for_lines){
		//performs an average
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			mean += buffer[i];
		}
		mean /= IMAGE_BUFFER_SIZE;

		do{
			chprintf((BaseSequentialStream *)&SD3, "we entered the monster with lines  : %d \r\n" , get_number_of_lines());
			wrong_line = 0;
			//search for a line_beginning
			while(stop_line_limit_search == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
				//the slope must at least be WIDTH_SLOPE wide and is compared
				//to the mean of the image
				if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean){
					line_beginning = i;
					stop_line_limit_search = 1;
				}
				i++;
			}
			//if a line_beginning was found, search for a line_ending
			if(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && line_beginning){
				stop_line_limit_search = 0;

				while(stop_line_limit_search == 0 && i < IMAGE_BUFFER_SIZE){
					if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean){
						line_ending = i;
						stop_line_limit_search = 1;
					}
					i++;
				}
				//if a line_ending was not found
				if(i > IMAGE_BUFFER_SIZE || !line_ending){
					line_found = 0;
				}
			}
			else{				//if no line_beginning was found
				line_found = 0;
			}

			if(line_found){
				//if a line too small has been detected, continues the search
				if((line_ending-line_beginning) < MIN_LINE_WIDTH){
					i = line_ending;
					line_beginning = 0;
					line_ending = 0;
					stop_line_limit_search = 0;
					wrong_line = 1;
				}
				else{
					last_width = width = (line_ending - line_beginning);
					line_position = (line_beginning + line_ending)/2; //gives the line position.
					line_beginning = 0;
					line_ending = 0;
					stop_line_limit_search = 0;
					wrong_line = 1;
					++number_of_lines;
					if(line_type == NUMBER_OF_LINES){
					    chprintf((BaseSequentialStream *)&SD3, "nb lines \r\n");

						i = line_ending;			//search for next lines
					}
					else if(line_type == LINE_POSITION){
						i = IMAGE_BUFFER_SIZE;		//stop when we know there is only one line
					}
				}
			}
		}while(wrong_line);
		chprintf((BaseSequentialStream *)&SD3, "we exited the monster with lines  : %d \r\n" , get_number_of_lines());
		if(!line_found){
			line_beginning = 0;
			line_ending = 0;
			width = last_width;
		}
		//sets a maximum width
		if((PXTOCM/width) > MAX_DISTANCE){
			width = PXTOCM/MAX_DISTANCE;
		}
	}

	if(number_of_lines > 0) {		//ATTENTION
		change_search_state(false);
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();								//maybe open the sampling to get more than 2 lines!
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}



static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = true;

    while(chThdShouldTerminateX() == false){
    	//waits until an image has been captured
		wait_image_detected();
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i += 2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for the number of lines in the image
		extract_line(image, searching_for_lines);        //ATTENTIONNNNNNNNNNNNN
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

void process_image_start(void){
    dcmi_start();
    po8030_start();

	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void wait_image_detected(void){
	chBSemWait(&image_ready_sem);
}

void change_search_state(bool new_state){
	searching_for_lines = new_state;
}

bool state(void){
	return searching_for_lines;
}

void set_line_type(LINE_TYPE_EXTRACT type){
	line_type = type;
}

LINE_TYPE_EXTRACT get_line_type(void){
	return line_type;
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void reset_line(void){
	number_of_lines = 0;
	chprintf((BaseSequentialStream *)&SD3, "RESET : \r\n" );

}

uint8_t get_number_of_lines(void){
	switch (number_of_lines)
	{
	case 0: 		//All LEDs are off
		break;
	case 1:
		palClearPad(GPIOD, GPIOD_LED1);		//one LED is on
		break;
	case 2:
		palClearPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED3);
		break;
	case 3:
		palClearPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED3);
		palClearPad(GPIOD, GPIOD_LED5);
		break;
	case 4:
		palClearPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED3);
		palClearPad(GPIOD, GPIOD_LED5);
		palClearPad(GPIOD, GPIOD_LED7);
		break;
	default:
		break;
	}
	palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);

	return number_of_lines;
}



							//////***PI REGULATOR PART, WHICH IS NOT A PI ANYMORE ******///////

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t extra_speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        if((line_type == LINE_POSITION) && ((number_of_lines) > 0)){
        	chprintf((BaseSequentialStream *)&SD3, "I'm in the line_pos and lines>0 \r\n" );
        	chprintf((BaseSequentialStream *)&SD3, "lines: %d \r\n" , number_of_lines );

            //computes the speed to give to the motors
            //distance_cm is modified by the image processing thread
            extra_speed = find_proximity();			///NONONON, APPELER LES IRs
            //disables the extra_speed if the IR input is to small
            //this avoids to always move as we cannot exactly be where we want and IR is a bit noisy
            if(fabs(extra_speed) < ERROR_THRESHOLD){
            	extra_speed = 0;
            }

            //computes a correction factor to let the robot rotate to be in front of the line
            speed_correction = (line_position - (IMAGE_BUFFER_SIZE/2));
            //if the line is nearly in front of the camera, don't rotate
            if(abs(speed_correction) < ROTATION_THRESHOLD){
            	speed_correction = 0;
            }

        	//applies the speed from the PI regulator and the correction for the rotation
        	right_motor_set_speed(MOTOR_SPEED_LIMIT/2 + extra_speed - ROTATION_COEFF * speed_correction);
        	left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + extra_speed + ROTATION_COEFF * speed_correction);
        	reset_line();
        }
        else if((line_type == LINE_POSITION) && ((number_of_lines) == 0)){
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    		searching_for_lines = false;
        }
        speed_correction = 0;
        extra_speed = 0;

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}


////////***********Extra code with all the old parts removed************///////
///*
// * camera_processing.c
// *
// *      Author: Bryan Kheirallah
// */
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <ch.h>
//#include <hal.h>
//#include <chprintf.h>
//#include <usbcfg.h>				//this one is not necessarily useful
//
//#include <math.h>
//#include "memory_protection.h"
//#include <camera/po8030.h>
//#include "motors.h"
//#include <camera_processing.h>
//#include <obstacles.h>
//#include <camera/dcmi_camera.h>
//
//
//static float distance_cm = 0;
//static uint8_t number_of_lines = 0;					//very important!
//static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
//static bool searching_for_lines = false;
//
//static LINE_TYPE_EXTRACT line_type = NOPE;
//static uint16_t width = 0; //better if we can put argument to threads
//
////semaphore
//static BSEMAPHORE_DECL(image_ready_sem, TRUE);
//
//
//void SendImageToSystem(uint8_t* data, uint16_t size)
//{
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
//}
//
///*
// *  Computes the line's width extracted from the image buffer given
// *  Gives 0 if line not found
// */
//void extract_line(uint8_t *buffer, bool searching_for_lines){
//
//
//	uint16_t i = 0, line_beginning = 0, line_ending = 0;
//	uint8_t stop_line_limit_search = 0, wrong_line = 0, line_found = 1;
//	uint32_t mean = 0;
//	width = 0;
//
//	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;
//
//	if(searching_for_lines){
//		//performs an average
//		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
//			mean += buffer[i];
//		}
//		mean /= IMAGE_BUFFER_SIZE;
//
//		do{
//			chprintf((BaseSequentialStream *)&SD3, "we entered the monster with lines  : %d \r\n" , get_number_of_lines());
//			wrong_line = 0;
//			//search for a line_beginning
//			while(stop_line_limit_search == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
//				//the slope must at least be WIDTH_SLOPE wide and is compared
//				//to the mean of the image
//				if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean){
//					line_beginning = i;
//					stop_line_limit_search = 1;
//				}
//				i++;
//			}
//			//if a line_beginning was found, search for a line_ending
//			if(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && line_beginning){
//				stop_line_limit_search = 0;
//
//				while(stop_line_limit_search == 0 && i < IMAGE_BUFFER_SIZE){
//					if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean){
//						line_ending = i;
//						stop_line_limit_search = 1;
//					}
//					i++;
//				}
//				//if a line_ending was not found
//				if(i > IMAGE_BUFFER_SIZE || !line_ending){
//					line_found = 0;
//				}
//			}
//			else{				//if no line_beginning was found
//				line_found = 0;
//			}
//
//			if(line_found){
//				//if a line too small has been detected, continues the search
//				if((line_ending-line_beginning) < MIN_LINE_WIDTH){
//					i = line_ending;
//					line_beginning = 0;
//					line_ending = 0;
//					stop_line_limit_search = 0;
//					wrong_line = 1;
//				}
//				else{
//					last_width = width = (line_ending - line_beginning);
//					line_position = (line_beginning + line_ending)/2; //gives the line position.
//					line_beginning = 0;
//					line_ending = 0;
//					stop_line_limit_search = 0;
//					wrong_line = 1;
//					++number_of_lines;
//					if(line_type == NUMBER_OF_LINES){
//					    chprintf((BaseSequentialStream *)&SD3, "nb lines \r\n");
//
//						i = line_ending;			//search for next lines
//					}
//					else if(line_type == LINE_POSITION){
//						i = IMAGE_BUFFER_SIZE;		//stop when we know there is only one line
//					}
//				}
//			}
//		}while(wrong_line);
//		chprintf((BaseSequentialStream *)&SD3, "we exited the monster with lines  : %d \r\n" , get_number_of_lines());
//		if(!line_found){
//			line_beginning = 0;
//			line_ending = 0;
//			width = last_width;
//		}
//		//sets a maximum width
//		if((PXTOCM/width) > MAX_DISTANCE){
//			width = PXTOCM/MAX_DISTANCE; //used to be a return with "else{return width;}"
//		}
//	}
//
//	if(number_of_lines > 0) {		//ATTENTION
//		change_search_state(false);
//	}
//}
//
//static THD_WORKING_AREA(waCaptureImage, 256);
//static THD_FUNCTION(CaptureImage, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
//	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
//	dcmi_enable_double_buffering();								//maybe open the sampling to get more than 2 lines!
//	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
//	dcmi_prepare();
//
//    while(1){
//        //starts a capture
//		dcmi_capture_start();
//		//waits for the capture to be done
//		wait_image_ready();
//		//signals an image has been captured
//		chBSemSignal(&image_ready_sem);
//    }
//}
//
//
//
//static THD_WORKING_AREA(waProcessImage, 1024);
//static THD_FUNCTION(ProcessImage, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//
//	uint8_t *img_buff_ptr;
//	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
//
//	bool send_to_computer = true;
//
//    while(chThdShouldTerminateX() == false){
//    	//waits until an image has been captured
//		wait_image_detected();
//		//gets the pointer to the array filled with the last image in RGB565
//		img_buff_ptr = dcmi_get_last_image_ptr();
//
//		//Extracts only the red pixels
//		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i += 2){
//			//extracts first 5bits of the first byte
//			//takes nothing from the second byte
//			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
//		}
//
//		//search for the number of lines in the image
//		extract_line(image, searching_for_lines);        //ATTENTIONNNNNNNNNNNNN
//
////		//converts the width into a distance between the robot and the camera
////		if(width && type == 1){ //assure we actually got a line and the type==1 is for safety, aka useless==1
////			distance_cm = PXTOCM/width;
////		}
////		else if(width == 0 && type == 1){
////			distance_cm = GOAL_DISTANCE;		//if width = 0, donc on est au bout de la ligne et distance-goal=0 no?
////		}
//
////		if(send_to_computer){
////			//sends to the computer the image
////			SendImageToSystem(image, IMAGE_BUFFER_SIZE);
////		}
//		//invert the bool
//		send_to_computer = !send_to_computer;
//
//		//chThdSleepUntilWindowed(time, time + MS2ST(4)); //reduced the sample rate to 250Hz
//
//    }
//}
//
//void process_image_start(void){
//    dcmi_start();
//    po8030_start();
//
//	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
//	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
//}
//
//void wait_image_detected(void){
//	chBSemWait(&image_ready_sem);
//}
//
//void change_search_state(bool new_state){
//	searching_for_lines = new_state;
//}
//
//bool state(void){
//	return searching_for_lines;
//}
//
//void set_line_type(LINE_TYPE_EXTRACT type){
//	line_type = type;
//}
//
//LINE_TYPE_EXTRACT get_line_type(void){
//	return line_type;
//}
//
//float get_distance_cm(void){
//	return distance_cm;
//}
//
//uint16_t get_line_position(void){
//	return line_position;
//}
//
//void reset_line(void){
//	number_of_lines = 0;
//	chprintf((BaseSequentialStream *)&SD3, "RESET : \r\n" );
//
//}
//
//uint8_t get_number_of_lines(void){
//	switch (number_of_lines)
//	{
//	case 0: 		//All LEDs are off
//		break;
//	case 1:
//		palClearPad(GPIOD, GPIOD_LED1);		//one LED is on
//		break;
//	case 2:
//		palClearPad(GPIOD, GPIOD_LED1);
//		palClearPad(GPIOD, GPIOD_LED3);
//		break;
//	case 3:
//		palClearPad(GPIOD, GPIOD_LED1);
//		palClearPad(GPIOD, GPIOD_LED3);
//		palClearPad(GPIOD, GPIOD_LED5);
//		break;
//	case 4:
//		palClearPad(GPIOD, GPIOD_LED1);
//		palClearPad(GPIOD, GPIOD_LED3);
//		palClearPad(GPIOD, GPIOD_LED5);
//		palClearPad(GPIOD, GPIOD_LED7);
//		break;
//	default:
//		break;
//	}
//	palSetPad(GPIOD, GPIOD_LED1);
//	palSetPad(GPIOD, GPIOD_LED3);
//	palSetPad(GPIOD, GPIOD_LED5);
//	palSetPad(GPIOD, GPIOD_LED7);
//
//	return number_of_lines;
//}
//
//
//
//							//////***PI REGULATOR PART, WHICH IS NOT A PI ANYMORE ******///////
//
//////simple PI regulator implementation
////int16_t pi_regulator(float distance, float goal){
////
////	float error = 0;
////	float speed = 0;
////	static float sum_error = 0;
////	error = distance - goal;
////	//disables the PI regulator if the error is to small
////	//this avoids to always move as we cannot exactly be where we want and
////	//the camera is a bit noisy
////	if(fabs(error) < ERROR_THRESHOLD){
////		return 0;
////	}
////	sum_error += error;
////	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
////	if(sum_error > MAX_SUM_ERROR){
////		sum_error = MAX_SUM_ERROR;
////	}
////	else if(sum_error < -MAX_SUM_ERROR){
////		sum_error = -MAX_SUM_ERROR;
////	}
////	speed = KP * error + KI * sum_error;
////
////    return (int16_t)speed;
////}
//
//static THD_WORKING_AREA(waPiRegulator, 256);
//static THD_FUNCTION(PiRegulator, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//    systime_t time;
//
//    int16_t extra_speed = 0;
//    int16_t speed_correction = 0;
//
//    while(1){
//        time = chVTGetSystemTime();
//
//        if((line_type == LINE_POSITION) && ((number_of_lines) > 0)){
//        	chprintf((BaseSequentialStream *)&SD3, "I'm in the line_pos and lines>0 \r\n" );
//        	chprintf((BaseSequentialStream *)&SD3, "lines: %d \r\n" , number_of_lines );
//
//            //computes the speed to give to the motors
//            //distance_cm is modified by the image processing thread
//            extra_speed = find_proximity();			///NONONON, APPELER LES IRs
//            //disables the extra_speed if the IR input is to small
//            //this avoids to always move as we cannot exactly be where we want and IR is a bit noisy
//            if(fabs(extra_speed) < ERROR_THRESHOLD){
//            	extra_speed = 0;
//            }
//
//            //computes a correction factor to let the robot rotate to be in front of the line
//            speed_correction = (line_position - (IMAGE_BUFFER_SIZE/2));
//            //if the line is nearly in front of the camera, don't rotate
//            if(abs(speed_correction) < ROTATION_THRESHOLD){
//            	speed_correction = 0;
//            }
//
//        	//applies the speed from the PI regulator and the correction for the rotation
//        	right_motor_set_speed(MOTOR_SPEED_LIMIT/2 + extra_speed - ROTATION_COEFF * speed_correction);
//        	left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + extra_speed + ROTATION_COEFF * speed_correction);
//        	reset_line();
//        }
//        else if((line_type == LINE_POSITION) && ((number_of_lines) == 0)){
//    		right_motor_set_speed(0);
//    		left_motor_set_speed(0);
//    		searching_for_lines = false;
//        }
//        speed_correction = 0;
//        extra_speed = 0;
//
//        //100Hz
//        chThdSleepUntilWindowed(time, time + MS2ST(10));
//    }
//}
//
//void pi_regulator_start(void){
//	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
//}
