#ifndef AIRBRAKES_KALMAN_H
#define AIRBRAKES_KALMAN_H
#include "rocket.h"


void kalman(Rocket* rocket, float  altitude, float acceleration, float dt, float reference_v);

#endif
