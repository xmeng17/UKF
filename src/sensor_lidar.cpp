#include "sensor_lidar.h"


//uses the ctrv model

Lidar::Lidar() : Sensor(){
	z_dim = 2;
	sensor_type = Sensor::LIDAR;
	std_sensor = VectorXd(z_dim);
	std_sensor << STD_PX,STD_PY;
}

Lidar::~Lidar(){}

VectorXd Lidar::GetMeasurement(VectorXd x){
  VectorXd z(2);
  z << x(0), x(1);
  return z;
}

VectorXd Lidar::GetInitState(VectorXd z){
  VectorXd x(5);
  x << z(0), z(1), 0,0,0;
  return x;
}

MatrixXd Lidar::GetInitCovariance() {
  MatrixXd P(5,5);
  P << 1,0,0,0,0,
       0,1,0,0,0,
       0,0,1000,0,0,
       0,0,0,1000,0,
       0,0,0,0,1;
  return P;
}
