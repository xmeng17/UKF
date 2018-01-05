#ifndef UKF_H
#define UKF_H

#include "bayes.h"
#include "stats.h"
#include "motion_ctrv.h"
#include "sensor_radar.h"
#include "sensor_lidar.h"

#include <cmath>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class UKF : public Bayes {
public:

  bool use_lidar_;
  bool use_radar_;

  bool is_initialized;

  VectorXd x_;
  MatrixXd P_;
  double dt_;

  Stats stats;
  double lambda_;
  VectorXd weights_;
  MatrixXd sigma_x_;


  UKF();
  virtual ~UKF();

  MatrixXd GenerateAugSigma(VectorXd x, MatrixXd P, double lambda);
  MatrixXd PredictSigmaX(MatrixXd aug_sigma, double dt);
  MatrixXd CalculateSigmaZ(MatrixXd sigma_x);

  double Fuse(Measurement meas);
  void Init();
  void Prediction();
  double Correction();

};

#endif /* UKF_H */
