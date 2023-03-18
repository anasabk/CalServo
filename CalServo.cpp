#include "CalServo.h"

CalServo::CalServo(int pin, uint32_t freq, ledc_channel_t channel, ledc_timer_t timer, ledc_timer_bit_t timer_res) 
    : m_pin(pin), m_freq(freq), m_channel(channel), m_timer(timer), m_timer_res(timer_res) {
    m_a = 0;
    m_b = 1;
}

/**
 * @brief Refresh the linear equation used to calculate PWM
 * values from given degrees.
 * @param pwm_list Pointer to array of PWM signals.
 * @param degree_list Pointer to array of degrees.
 * @param data_len Length of the arrays.
 * @attention Both pwm_list and degree_list should be of the same length.
 */
void CalServo::refresh_fitter(int* pwm_list, int* degree_list, int data_len) {
    //Check if there is no data.
    if(data_len < 1) {
        m_a = 0;
        m_b = 1;
    }

    else {
        float sumX = 0, 
              sumY = 0, 
              sumXSquare = 0, 
              sumXY = 0;
            
        //Calculate the summations needed by the linear fitting formula.
        for(int i = 0; i < data_len; i++){
            sumX += degree_list[i];
            sumY += pwm_list[i];
            sumXSquare += degree_list[i] * degree_list[i];
            sumXY += degree_list[i] * pwm_list[i];
        }

        //Calculate and store the constants of the linear equation.
        m_a = (float) ((sumY * sumXSquare - sumX * sumXY) / (data_len * sumXSquare - sumX * sumX));
        m_b = (float) ((data_len * sumXY - sumX * sumY) / (data_len * sumXSquare - sumX * sumX));
    }
}

/**
 * @brief Send PWM signal to the servo motor.
 * @param pwm The PWM signal to send
 */
void CalServo::set_PWM(int pwm) {
    //Calculate the maximum duty value.
    int max_duty = pow(2, ((double) m_timer_res)) - 1;

    //Calculate the desired duty value.
    uint32_t duty = max_duty * pwm * m_freq / 1000000;

    //Send the signal to the pin.
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, m_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, m_channel));
}

/**
 * @brief Set the servo motor to the given degree.
 * @param degree The degree to set the servo motor at.
 */
void CalServo::set_degree(int degree) {
    //Calculate the PWM value equivalent to the given degree.
    int pwm = (int) (m_a + m_b * degree);

    //Send the PWM value.
    set_PWM(pwm);
}

/**
 * @brief Initialize the servo motor configurations.
 */
void CalServo::init() {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = m_timer_res,
        .timer_num        = m_timer,
        .freq_hz          = m_freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = m_pin,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = m_channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = m_timer,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
