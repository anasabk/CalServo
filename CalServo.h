#ifndef CALSERVO_H_
#define CALSERVO_H_

#include <cmath>

#include "lwip/sys.h"
#include "driver/ledc.h"


class CalServo {
public:
    CalServo(int pin, uint32_t freq, ledc_channel_t channel, ledc_timer_t timer, ledc_timer_bit_t timer_res);

    void init();

    void refresh_fitter(int* pwm_list, int* degree_list, int data_len);

    void set_PWM(int pwm);

    void set_degree(int degree);

private:
    int m_pin;
    uint32_t m_freq;
    ledc_channel_t m_channel;
    ledc_timer_t m_timer;
    ledc_timer_bit_t m_timer_res;
    double m_a, m_b;
};

#endif