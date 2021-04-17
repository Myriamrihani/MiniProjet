#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ch.h>
#include <hal.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <chprintf.h>
#include "sensors/mpu9250.h"
#include "sensors/imu.h"
#include "audio/microphone.h"
#include "msgbus/messagebus.h"
#include <i2c_bus.h>
#include <dance.h>
#include <audio_processing.h>
#include <fft.h>
#include <com_mic.h>
#include <camera_processing.h>
#include "audio/play_melody.h"
#include "audio/audio_thread.h"
#include "button.h"



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



int main(void)
{
    /* System init */
    halInit();
    chSysInit();
    serial_start();

    /** Inits the Inter Process Communication bus. */
     messagebus_init(&bus, &bus_lock, &bus_condvar);
     dance_start();


     //process_image_start();

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;

     playMelodyStart();
     mic_start(&processAudioData);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
        //wait for new measures to be published
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));


//        chprintf((BaseSequentialStream *)&SD3, "searching line  : %d \r\n" , line_is_searching());
//        chprintf((BaseSequentialStream *)&SD3, "nb_line  : %d \r\n" , get_number_of_lines());
//
//
//        if(line_is_searching()){
//        	//when true on chercher pour le nombre de line jusqu'a trouver au moins une ligne
//        	//quand on trouve au moins une ligne on change line_searching to false et la on commence la procedure de dance
//        }

        //Je ne trouve pas le gpio du user button...
//        if (button_is_pressed){
//        	reset_dance();
//        }

        if(get_dance_memo_complete() == 1){
            wait_start_signal();
            if (get_start_dance() == 1) {
            	//playMelody(MARIO, ML_FORCE_CHANGE, NULL);
            	dancing();
            }
        } else  if (is_dance_clear()) {show_gravity(&imu_values);}
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
