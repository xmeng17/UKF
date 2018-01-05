#ifndef MEASUREMENT
#define MEASUREMENT

#include "Eigen/Dense"
#include "sensor.h"

class Measurement{
public:
  Sensor::SensorType sensortype;
  Eigen::VectorXd z; //measurement vector
  long t; //timestamp
};

#endif
