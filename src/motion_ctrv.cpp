#include "motion_ctrv.h"

/*
  * aug_x (augmented state vector):
           aug_x(0) = px (x coordinate)
           aug_x(1) = py (y coordinate)
           aug_x(2) = s (speed)
           aug_x(3) = theta_v (yaw)
           aug_x(4) = omega (yaw speed)
           aug_x(5) = a (accelaration along v)
           aug_x(6) = alpha (yaw acceleration)

  * x (state vector):
               x(0) = px
               x(1) = py
               x(2) = s
               x(3) = theta_v
               x(4) = omega
*/
CTRV::CTRV() : Motion(){
	x_dim = 5;
	aug_dim = 7;
	acc = VectorXd(2);
	acc << STD_A, STD_ALPHA;
  }

CTRV::~CTRV(){}

VectorXd CTRV::GetMotion(VectorXd aug_x,double dt){
  VectorXd x_m(5); //motion vector 

  if (fabs(aug_x(4)) > 0.001) {
    x_m(0) = aug_x(2) * (sin(aug_x(3)+aug_x(4)*dt) - sin(aug_x(3))) / aug_x(4); // px_m = s * (sin(theta_v+omega*dt) - sin(theta_v)) / omega
    x_m(1) = aug_x(2) * (-cos(aug_x(3)+aug_x(4)*dt) + cos(aug_x(3))) / aug_x(4); // py_m = s * (-cos(theta_v+omega*dt) + cos(theta_v)) / omega
  } else {
    x_m(0) = aug_x(2) * cos(aug_x(3)) * dt; // px_m = s * cos(theta_v) * dt
    x_m(1) = aug_x(2) * sin(aug_x(3)) * dt; // py_m = s * sin(theta_v) * dt
  }
  x_m(2) = 0; // s_m = 0
  x_m(3) = aug_x(4) * dt; // theta_x_m = omega * dt
  x_m(4) = 0; // omega_m = 0
  return x_m;
}
  
VectorXd CTRV::GetNoise(VectorXd aug_x,double dt){
  VectorXd x_n(5); //noise vector

  x_n(0) = aug_x(5) * cos(aug_x(3)) * dt*dt/2; // px_n = a * cos(theta_v) * dt2/2
  x_n(1) = aug_x(5) * sin(aug_x(3)) * dt*dt/2; // py_n = a * sin(theta_v) * dt2/2 
  x_n(2) = aug_x(5) * dt; // s_n = a * dt
  x_n(3) = aug_x(6)* dt*dt/2; // theta_v_n = alpha * dt2/2
  x_n(4) = aug_x(6) * dt; // omega_n = alpha * dt
  return x_n;
}

VectorXd CTRV::Predict(VectorXd aug_x,double dt){
  return aug_x.head(5) + GetMotion(aug_x,dt) + GetNoise(aug_x,dt);
}
