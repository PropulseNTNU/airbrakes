#include "MPU9250.h"
#include "airbrakes_sensors.h"

  int imuInit(MPU9250 &IMU){
    int status;
    // start communication with IMU 
    status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
    }
    // setting the accelerometer full scale range to +/-8G 
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);

  }
