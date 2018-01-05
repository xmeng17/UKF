#include "sensor_radar.h"


//uses the ctrv model

Radar::Radar() : Sensor(){
	z_dim = 3;
	sensor_type = Sensor::RADAR;
	std_sensor = VectorXd(z_dim);
	std_sensor<<STD_R,STD_TH,STD_PV;
}

Radar::~Radar(){}

VectorXd Radar::GetMeasurement(VectorXd x){
  VectorXd z(3);
//  if (fabs(x(0)) < 0.001) x(0) = ((x(0)>0) - (x(0)<0)) * 0.001;
//  if (fabs(x(1)) < 0.001) x(1) = ((x(1)>0) - (x(1)<0)) * 0.001;
  z(0) = sqrt(x(0)*x(0) + x(1)*x(1)); //r = sqrt(px2 + py2)
  z(1) = atan2(x(1),x(0)); // th = atan(py,px)
  z(2) = (x(0)*x(2)*cos(x(3)) + x(1)*x(2)*sin(x(3))) / z(0); // pv = (px*vx + py*vy) / r
                                                           // where vx = s * cos(theta_v)
                                                           //       vy = s * sin(theta_v)
  return z;
}

VectorXd Radar::GetInitState(VectorXd z){
  VectorXd x(5);
  x << z(0)*cos(z(1)), z(0)*sin(z(1)), 0,0,0; // px = r*cos(th), py = r*sin(th)
  return x;
}

MatrixXd Radar::GetInitCovariance() {
  MatrixXd P(5,5);
  P = MatrixXd::Identity(5,5);
  P << 1,0,0,0,0,
       0,1,0,0,0,
       0,0,1000,0,0,
       0,0,0,1000,0,
       0,0,0,0,1;
  return P;
}
