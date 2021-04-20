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


static float distance_cm = 0;
static uint8_t number_of_lines = 0;					//very important!
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static bool searching_for_lines = false;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


void SendImageToSystem(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
void extract_line_amount(uint8_t *buffer, bool searching_for_lines){

	uint16_t i = 0, line_beginning = 0, line_ending = 0;
	uint8_t stop_line_limit_search = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	if(searching_for_lines) {
		//performs an average
		for(uint16_t j = 0 ; j < IMAGE_BUFFER_SIZE ; j++){
			mean += buffer[j];
		}
		mean /= IMAGE_BUFFER_SIZE;

		do{
			wrong_line = 0;
			//search for a line_beginning
			while(stop_line_limit_search == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
			{
				//the slope must at least be WIDTH_SLOPE wide and is compared
				//to the mean of the image
				if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
				{
					line_beginning = i;
					stop_line_limit_search = 1;
				}
				i++;
			}
			//if a line_beginning was found, search for a line_ending
			if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && line_beginning)
			{
				stop_line_limit_search = 0;

				while(stop_line_limit_search == 0 && i < IMAGE_BUFFER_SIZE)
				{
					if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
					{
						line_ending = i;
						stop_line_limit_search = 1;
					}
					i++;
				}
				//if an line_ending was not found
				if (i > IMAGE_BUFFER_SIZE || !line_ending)
				{
					line_not_found = 1;
				}
			}
			else //if no line_beginning was found
			{
				line_not_found = 1;
			}

			//if a line too small has been detected, continues the search
			if(!line_not_found && (line_ending-line_beginning) < MIN_LINE_WIDTH){
				i = line_ending;
				line_beginning = 0;
				line_ending = 0;
				stop_line_limit_search = 0;
				wrong_line = 1;
			} else if(!line_not_found){		//if a line is valid, searches for the next line
				i = line_ending;
				line_beginning = 0;
				line_ending = 0;
				stop_line_limit_search = 0;
				wrong_line = 1;
				++number_of_lines;
			}
		}while(wrong_line);

	}
	if(get_number_of_lines() > 0) {
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
bool state(void){
	return searching_for_lines;
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
//		wait_image_detected(); //maybe add this
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for the number of lines in the image
		extract_line_amount(image, searching_for_lines);


//		if(send_to_computer){
//			//sends to the computer the image
//			SendImageToSystem(image, IMAGE_BUFFER_SIZE);
//		}
		//invert the bool
		send_to_computer = !send_to_computer;

		//chThdSleepUntilWindowed(time, time + MS2ST(4)); //reduced the sample rate to 250Hz

    }
}

void wait_image_detected(void){
	chBSemWait(&image_ready_sem);
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}


uint8_t get_number_of_lines(void){
	switch (number_of_lines)
	{
	case 0: //All LEDs are off
		palSetPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		break;
	case 1:
		palClearPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		break;
	case 2:
		palClearPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		break;
	case 3:
		palClearPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED3);
		palClearPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
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
	//after indicating the nb_lines, we can turn back OFF all the LEDs
	palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);
	return number_of_lines;
}

void reset_line(void){
	number_of_lines = 0;
	chprintf((BaseSequentialStream *)&SD3, "RESET : \r\n" );
	chprintf((BaseSequentialStream *)&SD3, "nb_lines just after RESET (should = 0) : \r\n" , get_number_of_lines());

}
void change_search_state(bool new_state){
	searching_for_lines = new_state;
}

void process_image_start(void){
    dcmi_start();
    po8030_start();

	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
