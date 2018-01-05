#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


//statistic tools for ctrv or radar
class Stats {
public:

  Stats();
  virtual ~Stats();

  VectorXd NormalizeAngle(VectorXd x);
  VectorXd GetDifference(VectorXd x, VectorXd std);

  VectorXd WeighedMean(MatrixXd sigma, VectorXd weights);
  MatrixXd WeighedCovariance(MatrixXd sigma, VectorXd mean, VectorXd weights);
  MatrixXd WeighedCovariance(MatrixXd x_sigma, VectorXd x_mean, MatrixXd z_sigma, VectorXd z_mean, VectorXd weights);

  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
};

#endif /* TOOLS_H_ */
