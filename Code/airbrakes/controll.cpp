#include "controll.h"

float calk_ref(int altitude){
  if (altitude<2000){
    return -1;
  }
  return (-0.5*altitude+500);//random function
}

float controller(int reference, long step, float *riemann_sum, float cur_velocity, float prev_velocity, int kpp, int kpd, float kpi){
  float error= reference-cur_velocity;
  (*riemann_sum)=integrate((*riemann_sum), error, step);
  return (kpp*error)+(kpi*(*riemann_sum))+(kpd*deriver(prev_velocity, cur_velocity, step));
}

float integrate(float prev_sum, float value, long step){
  return prev_sum+(value*step);
}

float deriver(float prev_val, float cur_val, long step){
  return (prev_val-cur_val)/step;
}
