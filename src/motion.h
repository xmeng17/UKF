#ifndef MOTION
#define MOTION

#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class Motion{
public:

  int x_dim;
  int aug_dim;
  VectorXd acc;

  Motion(){};
  virtual ~Motion(){};

  VectorXd AugmentX(VectorXd x){
	VectorXd aug_x(aug_dim);
	aug_x.fill(0.0);
	aug_x.head(x_dim) = x;
	return aug_x;
  };

  MatrixXd AugmentP(MatrixXd P){
	int acc_dim = aug_dim - x_dim;
	if (acc_dim != acc.size()){
	  cout<<"Error when Augmenting P: Acceleration Vector Dimension Doesn't Match State and Augmentation Dimension"<<endl;
	  return MatrixXd::Zero(aug_dim,aug_dim);
	}

	MatrixXd aug_P(aug_dim,aug_dim);
	aug_P.fill(0.0);
	aug_P.topLeftCorner(x_dim,x_dim) = P;
	aug_P.bottomRightCorner(acc_dim,acc_dim) = (acc.array() * acc.array()).matrix().asDiagonal();
	return aug_P;
  };

  virtual VectorXd GetMotion(VectorXd aug_x,double dt){};
  virtual VectorXd GetNoise(VectorXd aug_x,double dt){};
  virtual VectorXd Predict(VectorXd aug_x,double dt){};//combine state, moion, and noise

};

#endif
