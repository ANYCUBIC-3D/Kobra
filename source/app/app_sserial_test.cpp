#include "hc32_ddl.h"
#include "bsp_timer.h"
#include "SoftwareSerial.h"


#if TEST_SOFT_SERIAL

SoftwareSerial mySerial(PA3, PA2);


void app_sserial_test(void)
{
    timer0B_init();

    mySerial.begin(115200);

    mySerial.println("SoftwareSerial print test\n");

    while(1) {

        if(mySerial.read() == 'A') {
            mySerial.println("SoftwareSerial print\n");
        }
    }
}

#endif

