#include "ukf.h"
#include <iostream>


int main(){
  UKF* ukf = new UKF();
  Measurement meas;
  meas.z = VectorXd(3);
  meas.z << 8.46642,0.0287602,-3.04035;
  meas.sensortype = Sensor::RADAR;
  meas.t = 1477010443399637;
  ukf->Fuse(meas);
  meas.z = VectorXd(2);
  meas.z << 8.44818,0.251553;
  meas.sensortype = Sensor::LIDAR;
  meas.t = 1477010443449633;
  ukf->Fuse(meas);
  cout<<"x_="<<ukf->x_<<endl;
  cout<<"P_="<<ukf->P_<<endl;
}

