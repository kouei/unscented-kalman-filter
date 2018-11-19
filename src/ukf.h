#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Radar space dimension
  int n_z_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Augmented sigma points
  VectorXd z_;

  ///* Augmented sigma points
  MatrixXd Xsig_aug_;

  ///* Predicted sigma points
  MatrixXd Xsig_pred_;

  ///* sigma points in measurement space
  MatrixXd Zsig_;

  ///* Predicted state mean in measurement space
  VectorXd z_pred_;

  ///* Predicted covariance matrix in measurement space
  MatrixXd S_;

  ///* Normalized Innovation Squared
  double NIS_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Generate augmented sigma points
   */
  void AugmentedSigmaPoints();
  
  /**
   * Generate predicted sigma points
   * @param delta_t diff in time
   */
  void SigmaPointPrediction(double delta_t);
  
  /**
   * Calculate the predicted mean state and covariance from sigma points
   */
  void PredictMeanAndCovariance();

  /**
   * Transform sigma points to radar space
   */
  void PredictRadarMeasurement();

  /**
   * Transform sigma points to lidar space
   */
  void PredictLidarMeasurement();

  /**
   * Transform sigma points to measurement space
   */
  void UpdateState();

  /**
   * Normalize an angle so that it is within [-pi, +pi]
   * @param angle The angle to be normalized
   */
  double NormalizeAngle(double angle);
};

#endif /* UKF_H */
