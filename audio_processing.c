#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#include "motor_managmt.h"
#include "dance.h"


#define MIN_VALUE_THRESHOLD		20000
#define MIN_DIFFERENCE_VALUE 	10000
#define MIN_FREQ				10	//we don't analyze before and after these
#define MAX_FREQ				30	//indexes to minimize the resources used
#define FREQ_HUMAN				16	//250Hz
#define FREQ_HUMAN_L			(FREQ_HUMAN-5)
#define FREQ_HUMAN_H			(FREQ_HUMAN+5)
#define MIN_ROTATION_ANGLE		 22.5

typedef struct complex_float{
	float real;
	float imag;
}complex_float;


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

static bool start_dance = 0;
static float angle = 0;				//angle of the direction of the voice
static bool listening_voice = 0;

static MVMT_ROBOT voice_fb = STOP;	//voice direction front or back
static MVMT_ROBOT voice_rl = STOP;	//voice direction right or left

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);

}

//function used to detect highest amplitude peak in data
float highest_peak(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
		}
	}
	return max_norm;
}


//depending on the voice's direction detected in compare_mic,
//we set the direction angle and call the motor management function
void set_motor_angle(void){
	if(voice_fb == FRONT) {
		if(voice_rl == LEFT) {
			angle = MIN_ROTATION_ANGLE;
		} else if(voice_rl == RIGHT){
			angle = -MIN_ROTATION_ANGLE;
		} else if(voice_rl == STOP){
			angle = 0;
		}
	} else if(voice_fb == BACK) {
		if(voice_rl == RIGHT) {
			angle = -3*MIN_ROTATION_ANGLE;
		} else if(voice_rl == LEFT){
			angle = 3*MIN_ROTATION_ANGLE;
		} else if(voice_rl == STOP){
			angle = 8*MIN_ROTATION_ANGLE;
		}
	}else if(voice_fb == STOP) {
		if(voice_rl == LEFT) {
			angle = 2*MIN_ROTATION_ANGLE;
		} else if(voice_rl == RIGHT) {
			angle = -2*MIN_ROTATION_ANGLE;
		} else if(voice_rl == STOP) {
			listening_voice = 0;
		}
	}
	motor_follow_voice(angle);
}

//compares the 4 mics amplitude and sets the direction of the voice
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
		//do a 180 turn
		palSetPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED5);
		voice_fb = BACK;
	}else {
		//nor front nor back
		palSetPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED5);
		voice_fb = STOP;
	}

	if((voice_fb != STOP)){
		listening_voice = 1;
		set_motor_angle();
	} else if((voice_rl != STOP)){
		listening_voice = 1;
		set_motor_angle();
	} else {
		listening_voice = 0;
	}
}

bool get_listening_voice(void){
	return listening_voice;
}


void set_listening_voice(bool state){
	listening_voice = state;
}

//Simple function used to detect the highest value's frequency in a buffer
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
		if(max_norm_index >= FREQ_HUMAN_L && max_norm_index <= FREQ_HUMAN_H){
			start_dance = 1;
		}
	}
}


bool get_start_dance(void){
	return start_dance;
}


void set_start_dance(bool state) {
	start_dance = state;
}

//	Callback called when the demodulation of the four microphones is done.
void processAudioData(int16_t *data, uint16_t num_samples){

	//	We get 160 samples per mic every 10ms (16kHz). So we fill the
	//	sample buffers to reach 1024 samples, then we compute the FFTs.
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

		//stop when the buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){

		//FFT proccessing
		//This FFT function stores the results in the input buffer given.
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		//Magnitude processing - Computes the magnitude of the complex numbers and
		//stores them in a buffer of FFT_SIZE because it only contains real numbers
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		if(mustSend > 8){

			chBSemSignal(&micro_ready_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		if(get_mic_mode() == DANCE){
			sound_remote(micLeft_output);
		}

		if(get_mic_mode() == VOICE){
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


