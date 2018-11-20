//functions

float calk_ref(int altitude);
float controller(int reference, int velocity, long step, float *riemann_sum, float cur_velocity, float prev_velocity, int kpp, int kpd, float kpi);
float integrate(float prev_sum, float value, long step);
float deriver(float prev_val, float cur_val, long step); 
