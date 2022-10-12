#include "tone.h"
#include "timers.h"
#include "bsp_timer.h"
#include "bsp_pwm.h"


#define TIMRE_MAX_DURATION_MS   671


static volatile uint32_t duration_cnt = 0;
static volatile uint16_t duration_spare = 0;


void tone(uint8_t _pin, uint16_t frequency, uint32_t duration)
{
    duration_cnt   = duration / TIMRE_MAX_DURATION_MS;
    duration_spare = duration % TIMRE_MAX_DURATION_MS;

    if(duration_cnt) {
        timer01B_set_overflow(TIMRE_MAX_DURATION_MS);
    } else {
        timer01B_set_overflow(duration);
    }
    beep_pwm_set_frequency(frequency, 50);
}


void noTone(uint8_t _pin, bool destruct)
{

}

HAL_TONE_TIMER_ISR()
{
    if(duration_cnt) {
        duration_cnt--;
    } else if(duration_spare) {
        timer01B_set_overflow(duration_spare);
        duration_spare = 0;
    } else {
        beep_pwm_stop();
        timer01B_disable();
    }
}

