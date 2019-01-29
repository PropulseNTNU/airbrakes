#ifndef AIRBRAKES_CONTROLL_H
#define AIRBRAKES_CONTROLL_H
#include "rocket.h"
#include <NewPing.h>

//structs
typedef struct Parameters_t {
  float kpp;
  float kpi;
  float kpd;
}Parameters;
A_d<2,2>={0,1,0,3};
B_d<2,1>={0,1};
C_d<1,2>={1,0};
E_d<2,2>={0.00001,0.0000002,0.000000004,0.0000003};
Q<2,2>={3,0.0000001};
R=0.0000001; //Litt usikker på om R skal være en matrise eller en skalar. trodde egentlig den skulle være en matrise.
I<2,2>={1,0,0,1}

P_k_bar<2,2>={1,0,0.1,0};
x_hat_bar<2,1>={2000,300};
//functions
float calk_ref(int altitude);
// the sonar parameter is just for demo testing
float controller(Rocket* rocket,float* error, Parameters* parameters, float* riemann_sum, float* derivative, NewPing* sonar, float dt);
float integrate(float prev_sum, float value, float step);
float derivate(float prev_val, float cur_val, float step);
float kalman(float  altitude, float acceleration);

#endif
