#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <math.h>
#include "memory_protection.h"
#include <camera/po8030.h>
#include <camera_processing.h>
#include <camera/dcmi_camera.h>
#include <motor_managmt.h>


#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			30
#define CAMERA_POSITION			478


static uint8_t number_of_lines = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;
static bool searching_for_lines = false;
static bool line_found = true;
static LINE_TYPE_EXTRACT line_type = NO_LINE_TYPE;
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


void extract_line(uint8_t *buffer, bool searching_for_lines){

	uint16_t i = 0, line_beginning = 0, line_ending = 0;
	uint8_t stop_line_limit_search = 0, wrong_line = 0;

	//this element is a uint32_t because a cumulative addition is done
	uint32_t mean = 0;

	line_found = true;

	if(searching_for_lines){

		//performs an average to reduce the influence of ambient light
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			mean += buffer[i];
		}
		mean /= IMAGE_BUFFER_SIZE;

		do{
			wrong_line = 0;
			while(stop_line_limit_search == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){

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
				if(i > IMAGE_BUFFER_SIZE || !line_ending){
					line_found = 0;
				}
			} else{
				line_found = 0;
			}

			if(line_found){
				if((line_ending-line_beginning) < MIN_LINE_WIDTH){
					i = line_ending;
					line_beginning = 0;
					line_ending = 0;
					stop_line_limit_search = 0;
					wrong_line = 1;
				} else{
					line_position = (line_beginning + line_ending)/2;
					line_beginning = 0;
					i = line_ending;
					line_ending = 0;
					stop_line_limit_search = 0;
					wrong_line = 1;
					++number_of_lines;
					if(line_type == LINE_POSITION){
						i = IMAGE_BUFFER_SIZE;	//allows us to stop after 1 line
					}
				}
			}

		}while(wrong_line);

		if(!line_found){
			line_beginning = 0;
			line_ending = 0;
		}
	}

	if(number_of_lines > 0) {
		set_line_search_state(false);
	}
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	po8030_advanced_config(FORMAT_RGB565, 0, CAMERA_POSITION, IMAGE_BUFFER_SIZE,
											2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
		dcmi_capture_start();
		wait_image_ready();
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(chThdShouldTerminateX() == false){
		wait_image_detected();

		img_buff_ptr = dcmi_get_last_image_ptr();

		//extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i += 2){
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		extract_line(image, searching_for_lines);

		if((get_line_type() == LINE_POSITION)){
			motor_path_mode();
		}
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


void set_line_search_state(bool new_state){
	searching_for_lines = new_state;
}


void set_line_type(LINE_TYPE_EXTRACT type){
	line_type = type;
}


LINE_TYPE_EXTRACT get_line_type(void){
	return line_type;
}


uint16_t get_line_position(void){
	return line_position;
}


void reset_line(void){
	number_of_lines = 0;
}


uint8_t get_number_of_lines(void){

	if((line_type == NUMBER_OF_LINES) && (searching_for_lines == true)){
	switch (number_of_lines){
	case 0:
		break;		//All LEDs are off
	case 1:
		palClearPad(GPIOD, GPIOD_LED1);
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

