#include <iostream>
#include "stats.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Stats::Stats(){}
Stats::~Stats(){}

//math and statistic tools for ctrv, radar, and inspection
VectorXd Stats::NormalizeAngle(VectorXd x){
  int ind = -1;
  if (x.size() == 5) ind = 3; // if is state vector
  else if(x.size() == 3) ind = 1; // if is radar measurement
  else return x;
  while(x(ind) > M_PI) x(ind) -= 2 * M_PI;
  while(x(ind) < -M_PI) x(ind) += 2 * M_PI;
  return x;
}

VectorXd Stats::GetDifference(VectorXd x, VectorXd std) {
  if(x.size() != std.size()){
	cout << "Error Calculating Difference: x and std Doesn't Match" << endl;
	return VectorXd::Zero(x.size());
  }
  VectorXd dx = x - std;
  dx = NormalizeAngle(dx);
  return dx;
}

VectorXd Stats::WeighedMean(MatrixXd sigma, VectorXd weights) {
  if(weights.size() != sigma.cols()){
	cout << "Error Calculating Mean: Weight and Sigma Matrix Doesn't Match" << endl;
	return VectorXd::Zero(sigma.cols());
  }
  VectorXd x(sigma.rows());
  x.fill(0.0);
  for (int i = 0; i < sigma.cols(); i++) x += weights(i) * sigma.col(i);
  return x;
}

MatrixXd Stats::WeighedCovariance(MatrixXd sigma, VectorXd mean, VectorXd weights) {
  if(weights.size() != sigma.cols()){
	cout << "Error Calculating Covariance: Weight and Sigma Matrix Doesn't Match" << endl;
	return MatrixXd::Zero(sigma.cols(),sigma.cols());
  }
  MatrixXd P(sigma.rows(),sigma.rows());
  P.fill(0.0);
  for (int i = 0; i< sigma.cols(); i++) {
	VectorXd x_diff = GetDifference(sigma.col(i),mean);
	P += weights(i) * x_diff * x_diff.transpose();
  }
  return P;
}

MatrixXd Stats::WeighedCovariance(MatrixXd x_sigma, VectorXd x_mean, MatrixXd z_sigma, VectorXd z_mean, VectorXd weights) {
  if((weights.size() != x_sigma.cols()) || (weights.size() != z_sigma.cols())){
	cout << "Error Calculating Inter Covariance: Weight and Sigma Matrix Doesn't Match" << endl;
	return MatrixXd::Zero(weights.size(),weights.size());
  }
  MatrixXd T(x_sigma.rows(),z_sigma.rows());
  T.fill(0.0);
  for (int i = 0; i< weights.size(); i++) {
	VectorXd x_diff = GetDifference(x_sigma.col(i),x_mean);
	VectorXd z_diff = GetDifference(z_sigma.col(i),z_mean);
	T += weights(i) * x_diff * z_diff.transpose();
  }
  return T;
}

VectorXd Stats::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse = VectorXd::Zero(4);

  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Error calculating RMSE: Size doesn't match" << endl;
    return rmse;
  }

  for(int i=0;i<estimations.size();i++){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}
