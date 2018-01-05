#ifndef SENSOR_LIDAR
#define SENSOR_LIDAR

#include "sensor.h"

#define STD_PX 0.15
#define STD_PY 0.15

class Lidar : public Sensor{
public:
  Lidar();
  virtual ~Lidar();

  VectorXd GetMeasurement(VectorXd x);
  VectorXd GetInitState(VectorXd z);
  MatrixXd GetInitCovariance();
};

#endif

