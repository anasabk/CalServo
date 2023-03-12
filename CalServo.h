#ifndef CALSERVO_H_
#define CALSERVO_H_

#include <cmath>

#include "lwip/sys.h"
#include "driver/ledc.h"


class CalServo {
private:
    int pin;
    uint32_t freq;
    ledc_channel_t channel;
    ledc_timer_t timer;
    ledc_timer_bit_t timer_res;
    double a, b;

public:
    CalServo(int pin, uint32_t freq, ledc_channel_t channel, ledc_timer_t timer, ledc_timer_bit_t timer_res);

    void init();

    void refresh_fitter(int* pwm_list, int* degree_list, int data_len);

    void set_PWM(int pwm);

    void set_degree(int degree);
};

#endif