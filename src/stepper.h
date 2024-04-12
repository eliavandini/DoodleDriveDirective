#ifndef stepper_H
#define stepper_H
#include <Arduino.h>
#include "DRV8825.h"

class Stepper : public DRV8825
{
public:
    double degPermm = 10;
    double position = 0;
    Stepper(short steps, short dir_pin, short step_pin);
    Stepper(short steps, short dir_pin, short step_pin, short enable_pin);
    Stepper(short steps, short dir_pin, short step_pin, short mode0_pin, short mode1_pin, short mode2_pin);
    Stepper(short steps, short dir_pin, short step_pin, short enable_pin, short mode0_pin, short mode1_pin, short mode2_pin);
    void walk(double distance);
    double calcRotationForDistance(double distance);
};
#endif // DRV8825_H
