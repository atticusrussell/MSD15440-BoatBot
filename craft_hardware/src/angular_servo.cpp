#include <iostream>
#include <csignal>
#include <cstdlib>
#include <functional>
#include <pigpiod_if2.h>
#include "craft_hardware/angular_servo.hpp"

using namespace std;

// TODO revisit private vs protected vars - maybe swap to private 

Servo::Servo(int pin) {
    __pin=pin;
    int rcpi = pigpio_start(0, 0); // Connect to the pigpio daemon
    if (rcpi < 0) {
        throw "Cannot connect to pigpio daemon!";
    }
    pi = rcpi;
    int rc;
    rc = set_mode(pi, __pin, PI_OUTPUT);
    if ( rc == PI_BAD_GPIO){
        throw "Invalid GPIO pin Error!";
    } else if ( rc == PI_BAD_MODE) {
        throw "Inalid GPIO mode Error!";
    }
    
    // initializing to zero with pigpio is no input
    rc = set_servo_pulsewidth(pi, __pin, 0);
    if (rc == PI_BAD_USER_GPIO) {
        throw "Invalid user GPIO pin Error!";
    } else if (rc == PI_BAD_PULSEWIDTH) {
        throw "Invalid pulsewidth Error!";
    }
}

int Servo::getPulseWidth() {
    int rc = get_servo_pulsewidth(pi, __pin);
    if (rc == PI_BAD_USER_GPIO) {
        throw "Invalid user GPIO pin Error!";
    } else if (rc == PI_NOT_SERVO_GPIO) {
        throw "Invalid servo GPIO Error!";
    }
    return rc;
}

void Servo::setPulseWidth(int pulseWidth){
    int rc = set_servo_pulsewidth(pi, __pin, pulseWidth);
    if (rc == PI_BAD_USER_GPIO) {
        throw "Invalid user GPIO pin Error!";
    } else if (rc == PI_BAD_PULSEWIDTH) {
        throw "Invalid pulsewidth Error!";
    }
}

AngularServo::AngularServo(int pin, float minAngle, float maxAngle, int minPulseWidthUs, int maxPulseWidthUs) : Servo(pin) {
            __minAngle = minAngle;
            __maxAngle = maxAngle;
            __minPulseWidthUs = minPulseWidthUs;
            __maxPulseWidthUs = maxPulseWidthUs;
}

void AngularServo::setAngle(float angle){
    __angle = angle;
    // make sure angle is within bounds
    if (__angle < __minAngle){
        __angle = __minAngle;
    }
    if (__angle > __maxAngle){
        __angle = __maxAngle;
    }
    int pulseWidth = __minPulseWidthUs + (__angle - __minAngle) * (__maxPulseWidthUs - __minPulseWidthUs) / (__maxAngle - __minAngle);

    setPulseWidth(pulseWidth);
}

int AngularServo::getAngle(){
    int pulseWidth = getPulseWidth();
    float angle = __minAngle + (pulseWidth - __minPulseWidthUs) * (__maxAngle - __minAngle) / (__maxPulseWidthUs - __minPulseWidthUs);
    return angle;
}
