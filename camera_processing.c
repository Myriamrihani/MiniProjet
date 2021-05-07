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
#include <camera/dcmi_camera.h>
#include <audio_processing.h>


static float distance_cm = 0;	/////NOT USED////
static uint8_t number_of_lines = 0;					//very important!
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static bool searching_for_lines = false;
static bool line_found = 1;
static LINE_TYPE_EXTRACT line_type = NO_LINE_TYPE;
static uint16_t width = 0; //better if we can put argument to threads //WHY static?!

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


void extract_line(uint8_t *buffer, bool searching_for_lines){

	uint16_t i = 0, line_beginning = 0, line_ending = 0;
	uint8_t stop_line_limit_search = 0, wrong_line = 0;
	uint32_t mean = 0;
	width = 0;
	line_found = true;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	if(searching_for_lines){
		//performs an average
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			mean += buffer[i];
		}
		mean /= IMAGE_BUFFER_SIZE;

		do{
			wrong_line = 0;
			//search for a line_beginning
			while(stop_line_limit_search == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){

				//the slope must at least be WIDTH_SLOPE wide and is compared
				//to the mean of the image
				if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean){			///////////////
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
					//chprintf((BaseSequentialStream *)&SD3, "no end found \r\n");
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
					i = line_ending;
					line_ending = 0;
					stop_line_limit_search = 0;
					wrong_line = 1;
					++number_of_lines;
					chprintf((BaseSequentialStream *)&SD3, "line position  : %d \r\n" , line_position);

					if(line_type == LINE_POSITION){
						i = IMAGE_BUFFER_SIZE;	//allows us to stop after 1 line
					}
					else if(line_type == NUMBER_OF_LINES){	//this part is just comments, to be removed
						chThdSleepMilliseconds(300);	//allows us to slide a paper in before the first line is detected alone
						chprintf((BaseSequentialStream *)&SD3, "nb lines: %d \r\n", number_of_lines);
						chprintf((BaseSequentialStream *)&SD3, "value should be as line_pos+width/2 : %d \r\n" , i);
					}
				}
			}
		}while(wrong_line);

		if(!line_found){
			line_beginning = 0;
			line_ending = 0;
			width = last_width;
		}
		//sets a maximum width
		if((PXTOCM/width) > MAX_DISTANCE){		/////NOT USED////
			width = PXTOCM/MAX_DISTANCE;		/////NOT USED////
		}
	}

	if(number_of_lines > 0) {		//to stop searching
		change_search_state(false);
	}
}

bool get_line_found(void){
	return line_found;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 478, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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



static THD_WORKING_AREA(waProcessImage, 2048);
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
		extract_line(image, searching_for_lines);
		if((get_line_type() == LINE_POSITION)){
			motor_path_mode();
		}

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

bool get_search_state(void){
	return searching_for_lines;
}

void set_line_type(LINE_TYPE_EXTRACT type){
	line_type = type;
}

LINE_TYPE_EXTRACT get_line_type(void){
	return line_type;
}

float get_distance_cm(void){		/////NOT USED////
	return distance_cm;				/////NOT USED////
}

uint16_t get_line_position(void){
	return line_position;
}

void reset_line(void){
	number_of_lines = 0;
	chprintf((BaseSequentialStream *)&SD3, "RESET : \r\n" );
}

uint8_t get_number_of_lines(void){
	if(line_type == NUMBER_OF_LINES){
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
    chThdSleepMilliseconds(500);
	palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);
	}

	return number_of_lines;
}

