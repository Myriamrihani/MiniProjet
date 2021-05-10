#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <com_mic.h>
#include "motors.h"
#include "motor_managmt.h"
#include "dance.h"
#include "camera_processing.h"


//semaphore
static BSEMAPHORE_DECL(micro_ready_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	20000
#define MIN_DIFFERENCE_VALUE 10000


#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_WOMAN		16	//250Hz
#define FREQ_MAN		12	//296Hz
//#define FREQ_RIGHT		23	//359HZ
//#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_WOMAN_L		(FREQ_WOMAN-5)
#define FREQ_WOMAN_H		(FREQ_WOMAN+5)
#define FREQ_MAN_L			(FREQ_MAN-1)
#define FREQ_MAN_H			(FREQ_MAN+1)

static bool start_dance = 0;
static float angle = 0;
static bool listening_voice = 0;

static FREQUENCY_TO_DETECT frequency = NO_FREQ;

static mvmt_robot voice_fb = STOP;
static mvmt_robot voice_rl = STOP;


float highest_peak(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
		}
	}

	return max_norm;
}

void compare_mic(float* right, float* left, float* back, float* front){
	if((highest_peak(left) - highest_peak(right)) > MIN_DIFFERENCE_VALUE){
		//turn left
	    palClearPad(GPIOD, GPIOD_LED7);
	    palSetPad(GPIOD, GPIOD_LED3);
		voice_rl = LEFT;
	} else if((highest_peak(right) - highest_peak(left)) > MIN_DIFFERENCE_VALUE){
		//turn right
		palSetPad(GPIOD, GPIOD_LED7);
		palClearPad(GPIOD, GPIOD_LED3);
		voice_rl = RIGHT;
	} else {
		//nor left nor right
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED7);
		voice_rl = STOP;
	}

	if((highest_peak(front) - highest_peak(back)) > MIN_DIFFERENCE_VALUE){
		//go forward
	    palClearPad(GPIOD, GPIOD_LED1);
	    palSetPad(GPIOD, GPIOD_LED5);
		voice_fb = FRONT;

	} else if((highest_peak(back) - highest_peak(front)) > MIN_DIFFERENCE_VALUE){
		//do a 180
		palSetPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED5);
		voice_fb = BACK;
	}else {
		//none
		palSetPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED5);
		voice_fb = STOP;
	}

	if((voice_fb != STOP)) {
		listening_voice = 1;
		set_motor_angle();
	} else if((voice_rl != STOP)){
		listening_voice = 1;
		set_motor_angle();
	} else {
		listening_voice = 0;
		//set_line_type(LINE_POSITION);
	}
}

void set_motor_angle(void){
	if(voice_fb == FRONT) {
		if(voice_rl == LEFT) {
			angle = 22.5;
		} else if(voice_rl == RIGHT){
			angle = -22.5;
		} else if(voice_rl == STOP){
			angle = 0;
		}
	} else if(voice_fb == BACK) {
		if(voice_rl == RIGHT) {
			angle = -67.5;
		} else if(voice_rl == LEFT){
			angle = 67.5;
		} else if(voice_rl == STOP){
			angle = 180;
		}
	}else if(voice_fb == STOP) {
		if(voice_rl == LEFT) {
			angle = 45;
		} else if(voice_rl == RIGHT) {
			angle = -45;
		} else if(voice_rl == STOP) {
			listening_voice = 0;
		}
	}
	motor_take_direction(angle);
}

bool get_listening_voice(void){
	return listening_voice;
}

void set_listening_void(bool state){
	listening_voice = state;
}

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	if(get_dance_memo_complete() ==1 ){
		switch(frequency){
			case 0 :
				break;
			case 1:
				if(max_norm_index >= FREQ_WOMAN_L && max_norm_index <= FREQ_WOMAN_H){
					start_dance = 1;
				}
				break;
			case 2:
				if(max_norm_index >= FREQ_MAN_L && max_norm_index <= FREQ_MAN_H){
					start_dance = 1;
				}
		}
	}
}

void set_frequency(FREQUENCY_TO_DETECT freq){
	frequency = freq;
}

FREQUENCY_TO_DETECT get_frequency(void){
	return frequency;
}

bool get_start_dance(void){
	return start_dance;
}

void set_start_dance(bool state) {
	start_dance = state;
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&micro_ready_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		if(get_mode() == DANCE){
			sound_remote(micLeft_output);
		}

		if(get_mode() == VOICE){
			compare_mic(micRight_output, micLeft_output, micBack_output, micFront_output);
		}
	}
}



void wait_start_signal(void){
	chBSemWait(&micro_ready_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}


