#include "ukf.h"

UKF::UKF() : Bayes(){
  use_lidar_ = true;
  use_radar_ = true;

  is_initialized = false;

  motion = new CTRV();
  x_ = VectorXd(motion->x_dim);
  P_ = MatrixXd(motion->x_dim,motion->x_dim);

  lambda_ = 3 - motion->x_dim;
  weights_ = VectorXd(2*motion->aug_dim+1);
  weights_(0) = lambda_ / (lambda_ + motion->aug_dim);
  for (int i = 1; i < 2*motion->aug_dim+1; i++) weights_(i) = 0.5 / (lambda_ + motion->aug_dim);
}

UKF::~UKF(){}


MatrixXd UKF::GenerateAugSigma(VectorXd x, MatrixXd P, double lambda){
  MatrixXd aug_sigma = MatrixXd(motion->aug_dim, 2*motion->aug_dim+1);
  VectorXd aug_x = motion->AugmentX(x);
  MatrixXd aug_P = motion->AugmentP(P);
  MatrixXd L = aug_P.llt().matrixL();
  aug_sigma.col(0) = aug_x;
  for (int i = 0; i < motion->aug_dim; i++){
	aug_sigma.col(i+1) = aug_x + sqrt(lambda+motion->aug_dim) * L.col(i);
	aug_sigma.col(i+1+motion->aug_dim) = aug_x - sqrt(lambda+motion->aug_dim) * L.col(i);
  }
  return aug_sigma;
}

MatrixXd UKF::PredictSigmaX(MatrixXd aug_sigma,double dt){
  MatrixXd sigma_x(motion->x_dim, 2*motion->aug_dim+1);
  for (int i = 0; i < 2*motion->aug_dim+1; i++){
	sigma_x.col(i) = motion->Predict(aug_sigma.col(i),dt);
  }
  return sigma_x;
}

MatrixXd UKF::CalculateSigmaZ(MatrixXd sigma_x){
  MatrixXd sigma_z(sensor->z_dim, 2*motion->aug_dim+1);
  for (int i = 0; i < 2*motion->aug_dim+1; i++){
	sigma_z.col(i) = sensor->GetMeasurement(sigma_x.col(i));
  }
  return sigma_z;
}

double UKF::Fuse(Measurement meas){
  meas_ = meas;

  if (meas_.sensortype == Sensor::RADAR && use_radar_) sensor = new Radar();
  else if (meas_.sensortype == Sensor::LIDAR && use_lidar_) sensor = new Lidar();
  else {
    if (! meas_.sensortype == Sensor::RADAR && ! meas_.sensortype == Sensor::LIDAR) cout << "Unrecognized Sensor"<<endl;
    return 0.0;
  }

  if(!is_initialized){
	Init();
    return 0.0;
  }

  dt_ = (meas_.t - prev_meas_.t) / 1000000.0;
  Prediction();
  double nis = Correction();
  prev_meas_ = meas_;

  return nis;
}

void UKF::Init(){
  x_ = sensor->GetInitState(meas_.z);
  P_ = sensor->GetInitCovariance();
  prev_meas_ = meas_;
  is_initialized = true;
}

void UKF::Prediction(){
  MatrixXd aug_sigma = GenerateAugSigma(x_,P_,lambda_);
  sigma_x_ = PredictSigmaX(aug_sigma,dt_);

  x_ = stats.WeighedMean(sigma_x_,weights_);
  P_ = stats.WeighedCovariance(sigma_x_,x_,weights_);
}

double UKF::Correction(){
  MatrixXd sigma_z = CalculateSigmaZ(sigma_x_);
  VectorXd z_pred = stats.WeighedMean(sigma_z,weights_);

  MatrixXd S = stats.WeighedCovariance(sigma_z,z_pred,weights_) + sensor->GetCovarianceNoise();
  MatrixXd S_inverse = S.inverse();
  MatrixXd T = stats.WeighedCovariance(sigma_x_,x_,sigma_z,z_pred,weights_);
  MatrixXd K = T * S_inverse;

  VectorXd y = stats.GetDifference(meas_.z,z_pred);
  x_ += K * y;
  P_ -= K * S * K.transpose();
  return (y.transpose() * S_inverse * y)(0);
}

