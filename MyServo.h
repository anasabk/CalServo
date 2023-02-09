#ifndef SERVO_H_
#define SERVO_H_

class MyServo {
private:
    int pin;
    double a, b;

public:
    MyServo(int servoPin);

    void refreshFitter(int* pwm_list, int* degree_list, int data_len);

    int getPWM(int degree);
};

#endif