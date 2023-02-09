#include "MyServo.h"

MyServo::MyServo(int pin) {
    this->pin = pin;
    this->a = 0;
    this->b = 1;
}

void MyServo::refreshFitter(int* pwm_list, int* degree_list, int data_len) {
    if(data_len < 1) {
        this->a = 0;
        this->b = 1;
    }

    else {
        float sumX = 0, 
            sumY = 0, 
            sumXSquare = 0, 
            sumXY = 0;
            
        for(int i = 0; i < data_len; i++){
            sumX += degree_list[i];
            sumY += pwm_list[i];
            sumXSquare += degree_list[i] * degree_list[i];
            sumXY += degree_list[i] * pwm_list[i];
        }

        this->a = (float) ((sumY * sumXSquare - sumX * sumXY) / (data_len * sumXSquare - sumX * sumX));
        this->b = (float) ((data_len * sumXY - sumX * sumY) / (data_len * sumXSquare - sumX * sumX));
    }
}

int MyServo::getPWM(int degree) {
    return (int) (this->a + this->b * degree);
}