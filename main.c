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
#include "msgbus/messagebus.h"
#include <i2c_bus.h>
#include <dance.h>


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

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
        //wait for new measures to be published
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

        //prints raw values
        //imu_display(imu_values);

        //this if doesn't work properly (aborts de debbuging) -> big chance that it has to do with dancing
    	chprintf((BaseSequentialStream *)&SD3, "complete  : %d \r\n" , dance_memorized());
    	chprintf((BaseSequentialStream *)&SD3, "true?  : %d \r\n" , true);


        if(dance_memorized() == 1){
        	chprintf((BaseSequentialStream *)&SD3, "bonjour \r\n" );
        	dancing();
        } else  show_gravity(&imu_values);

        //show_gravity(&imu_values);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
