
//functions
float calk_ref(int altitude);
float controller(int reference, float step, float *riemann_sum, float cur_velocity, float prev_velocity, int kpp, int kpd, float kpi);
float integrate(float prev_sum, float value, float step);
float deriver(float prev_val, float cur_val, float step);
