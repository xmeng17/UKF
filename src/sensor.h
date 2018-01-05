#ifndef SENSOR
#define SENSOR

#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class Sensor{
public:
  int z_dim;
  VectorXd std_sensor;
  enum SensorType{
    LIDAR,
    RADAR
  } sensor_type;

  Sensor(){};
  virtual ~Sensor(){};

  virtual VectorXd GetMeasurement(VectorXd x){};
  virtual VectorXd GetInitState(VectorXd z){};
  virtual MatrixXd GetInitCovariance(){};
  MatrixXd GetCovarianceNoise(){
    return (std_sensor.array() * std_sensor.array()).matrix().asDiagonal();
  };
};

#endif
