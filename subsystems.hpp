#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor outtake(2);
inline pros::Motor intake(3);

inline Piston centerGoal('A');
inline Piston descorer('B', true);
inline Piston matchloader('C');

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');