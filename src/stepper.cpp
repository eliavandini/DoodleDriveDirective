#include "stepper.h"
#include "DRV8825.h"

Stepper::Stepper(short steps, short dir_pin, short step_pin)
    : DRV8825(steps, dir_pin, step_pin)
{
}

Stepper::Stepper(short steps, short dir_pin, short step_pin, short enable_pin)
    : DRV8825(steps, dir_pin, step_pin, enable_pin)
{
}

Stepper::Stepper(short steps, short dir_pin, short step_pin, short mode0_pin, short mode1_pin, short mode2_pin)
    : DRV8825(steps, dir_pin, step_pin, mode0_pin, mode1_pin, mode2_pin)
{
}

Stepper::Stepper(short steps, short dir_pin, short step_pin, short enable_pin, short mode0_pin, short mode1_pin, short mode2_pin)
    : DRV8825(steps, dir_pin, step_pin, enable_pin, mode0_pin, mode1_pin, mode2_pin)
{
}

void Stepper::walk(double distance)
{
    rotate(calcRotationForDistance(distance));
}

double Stepper::calcRotationForDistance(double distance)
{
    return distance / degPermm;
}