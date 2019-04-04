#include "controll.h"
#include <math.h>


//varibles for testing
float flap_width=0.1106; // the with of the air brake flap in meters. This is only for testing, and will not be used during flight
float max_extention=0.02; //the max length of the air brake flaps in meters. This is only for testing, and will not be used during flight


float integrate(float prev_sum, float value, float step){
  return prev_sum + (value * step);
}

float controller(float* error, Parameters* parameters, float* riemann_sum, float dt){ //PI-controller
  // add a lower and upper bound to prevernt overflow
  *riemann_sum = integrate(*riemann_sum, *error, dt); //integrates error
  return (parameters->kpp*(*error)) + (parameters->kpi*(*riemann_sum));
}

//functions for testing
float test_calculate_area(float u){
  return 3*flap_width*max_extention*sin(u);
}

float test_modifications(float ref_u, float prev_u, float dt){//Calculates the actual actuation based on servospeed
  if (ref_u==prev_u){
    return ref_u;
  }
  if (ref_u>prev_u){
    prev_u+=(60/0.13)*dt;//Calculates the servo position based on rotationspeed.
    if(prev_u>ref_u){//In this case the servo has reaced its reference, and ref_u can be returned
      return ref_u;
    }
    return prev_u;
  }
  prev_u-=(60/0.13)*dt;//Calculates the servo position based on rotationspeed.
  if (prev_u<ref_u) {//In this case the servo has reaced its reference, and ref_u can be returned
    return ref_u;
  }
  return prev_u;
}
