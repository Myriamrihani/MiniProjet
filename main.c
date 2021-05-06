#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ch.h>
#include <hal.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "memory_protection.h"
#include <main.h>

#include "sensors/mpu9250.h"
#include "sensors/imu.h"
#include "audio/microphone.h"
#include "msgbus/messagebus.h"
#include <i2c_bus.h>
#include <dance.h>
#include <audio_processing.h>
#include <fft.h>
#include "com_mic.h"
#include <camera_processing.h>
#include <sensors/proximity.h>
#include "audio/play_melody.h"
#include "audio/audio_thread.h"
#include "button.h"
#include "audio/play_sound_file.h"
#include "selector.h"
#include "motors.h"
#include "motor_managmt.h"


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg); // UART3. Connected to the second com port of the programmer
}

static THD_WORKING_AREA(selector_freq_thd_wa, 2048);
static THD_FUNCTION(selector_freq_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    while(1) {

		switch(get_selector()) {
			case 0:
				set_frequency(NO_FREQ);
				break;

			case 1:
				set_frequency(WOMAN);
				break;

			case 2:
				set_frequency(MAN);
				break;

			default: break;
		}
    }
}

int main(void)
{
    /* System init */
	halInit();
    chSysInit();
    serial_start();
	dac_start();
    usb_start();


	/** Inits the Inter Process Communication bus. */
	messagebus_init(&bus, &bus_lock, &bus_condvar);
    imu_start();
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    motors_init();
  // pi_regulator_start();
    process_image_start();
    proximity_start();
    playMelodyStart();
    playSoundFileStart();
    mic_start(&processAudioData);

    chThdCreateStatic(selector_freq_thd_wa, sizeof(selector_freq_thd_wa), NORMALPRIO, selector_freq_thd, NULL);
    chThdSleepMilliseconds(1000);
    uint8_t freq_counter = 0;
    uint8_t past_freq = 0;

    change_search_state(true);
    set_line_type(NUMBER_OF_LINES);
    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
        //wait for new measures to be published
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
		switch(get_frequency()) {
			case 0: //only used for testing
				set_line_type(LINE_POSITION);
				set_mode(VOICE);

				break;

			case 1:
//				if(get_line_type() == LINE_POSITION){ change_search_state(true); }
//			    	chprintf((BaseSequentialStream *)&SD3, "setting line position\r\n");
//			    	change_search_state(true);
//			    	set_line_type(LINE_POSITION);
				if(get_line_type() == NUMBER_OF_LINES){
					dance(WOMAN, &imu_values);
				}
//				if(get_line_type() == LINE_POSITION){
//					set_mode(VOICE);
//				}
				break;

			case 2:
				chprintf((BaseSequentialStream *)&SD3, "i'm in 2 \r\n");
				if(get_line_type() == NUMBER_OF_LINES){ dance(WOMAN, &imu_values); }
				if(get_line_type() == LINE_POSITION){ change_search_state(true); }
				set_mode(DANCE);
	        	set_line_type(NUMBER_OF_LINES);
	        	dance(MAN, &imu_values);
				break;
		}
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
