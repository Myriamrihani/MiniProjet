#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include "motor_managmt.h"


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

typedef enum {
	NO_FREQ = 0,
	WOMAN,
	MAN,
}FREQUENCY_TO_DETECT;


void compare_mic(float* right, float* left, float* back, float* front);
float highest_peak(float* data);
void set_motor_angle(void);
void processAudioData(int16_t *data, uint16_t num_samples);
void set_frequency(FREQUENCY_TO_DETECT freq);
FREQUENCY_TO_DETECT get_frequency(void);
bool get_start_dance(void);
void set_start_dance(bool state);
bool get_listening_voice(void);
void set_listening_voice(bool state);


/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_start_signal(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
