#include <Servo.h>

#include "airbrakes_setup.h"
#include "controll.h"
#include "kalman.h"
#include "interpolation.h"
#include "serial_reader.h"

Servo _servo;

//initilises variables
float error = 0; //error used in controller
float riemann_sum = 0; //used in integrator, witch is used in controller
float u = 0; //sets servo to 45 degrees, this causes the air brakes to brake at 50% capasaty
float prev_u=0;//only used for testing
float dt = 0; //time step used in integrator and kalman filter
float simDt = 0; // the timestep we we need to be in sync with Penumbra
Parameters parameters = { 20 , 0.01 , 1 }; //Control parameters (Kp, Ki, Kd)
unsigned long time_new, time_old = 0; // time variables for delta time

float sensor_data[4]={0,0,0,0}; //Barometer at index 0 and accelrometer (z-direction)at index 1. Utvides kanskje senere m/pitch
float estimates[2]; //Estimates from Kalman filter. [height, velocity]
float reference_v= 0; //reference_velovity

void setup() { //initiates servo and printing
  pinMode(SERVO_PIN, OUTPUT);
  _servo.attach(SERVO_PIN);
  Serial.begin(9600);
  while(!Serial) {};
  Serial.clear();
}


void loop() {
  //Updats dt
  time_new = micros();
  dt = (float)(time_new - time_old);
  dt /= (float)1000000; // converted to seconds
  time_old = time_new;

  updateSensorData(sensor_data);
  
  Serial.print("t_h");
  Serial.println(sensor_data[0]);
  Serial.print("t_a");
  Serial.println(sensor_data[1]);
  Serial.print("iter");
  Serial.println(sensor_data[2], 4);
  Serial.print("vel");
  Serial.println(sensor_data[3]);
  Serial.print("error");
  Serial.println(error);
  
  if(dt > 0 && sensor_data[2] > 0){
    simDt = 0.03/(sensor_data[2]/dt);
    }
  else{
      simDt = 0.01;
    }

  if(sensor_data[0]<1287){
    Serial.print("est_h");
    Serial.println("0");

    Serial.print("est_v");
    Serial.println("0");

    Serial.print("c_s");
    Serial.println("0");
  }
  else{
    kalman(estimates, sensor_data[0], sensor_data[1], simDt, reference_v);

    Serial.print("est_h");
    Serial.println(estimates[0],2);

    Serial.print("est_v");
    Serial.println(estimates[1],2);
    
    reference_v = getReferenceVelocity(estimates[0]);
    error = estimates[1] - reference_v;
    u = controller(&error, &parameters, &riemann_sum, simDt); //updates controll signal
    prev_u = test_modifications(u, prev_u, simDt);
    u = prev_u;
    if (u < 0){
        u = 0;
    }
    else if (u > 75){
        u = 75;
    }

    Serial.print("c_s");
    Serial.println(test_calculate_area(u),6);
  }
  delay(10);
}
