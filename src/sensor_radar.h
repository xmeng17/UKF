#ifndef SENSOR_RADAR
#define SENSOR_RADAR

#include "sensor.h"
#include <cmath>

#define STD_R 0.3
#define STD_TH 0.03
#define STD_PV 0.3

#include <iostream>

using namespace std;
using Eigen::VectorXd;

class Radar : public Sensor{
public:
  Radar();
  virtual ~Radar();
  
  VectorXd GetMeasurement(VectorXd x);
  VectorXd GetInitState(VectorXd z);
  MatrixXd GetInitCovariance();
};

#endif

