#ifndef SERIAL_READER_H
#define SERIAL_READER_H
#include <BasicLinearAlgebra.h>

void parseData();
void recvWithStartEndMarkers();
bool updateSensorData(float* sensor_data);


#endif
