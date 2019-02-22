#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix
  H_laser_ << 1,0,0,0,
              0,1,0,0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  // Initialize with first measurement
  if (!is_initialized_) {

    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    MatrixXd P(4,4);
    P << 0.5,0,0,0,
         0,0.5,0,0,
         0,0,0.5,0,
         0,0,0,0.5;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      ekf_.x_[0] = cos(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = sin(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
      ekf_.x_[2] = 1;
      ekf_.x_[3] = 1;
      ekf_.P_ = P;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
      ekf_.x_[2] = 1;
      ekf_.x_[3] = 1;
      ekf_.P_ = P;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;

    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /**
   * Prediction
   */

  // Update F since it depends on delta t
  float dt = (measurement_pack.timestamp_-previous_timestamp_)/1000000.;

  ekf_.F_ = tools.F(dt);

  ekf_.Q_ = tools.Q(dt);

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    // set angle to range -pi,pi
    float r = sqrt(ekf_.x_[0]*ekf_.x_[0]+ekf_.x_[1]*ekf_.x_[1]);
    VectorXd hx(3);
    float angle = atan2(ekf_.x_[1],ekf_.x_[0]);
    hx << r, angle, (ekf_.x_[0]*ekf_.x_[2]+ekf_.x_[1]*ekf_.x_[3])/r;

    VectorXd y = measurement_pack.raw_measurements_-hx;
    if(y[1] > M_PI)
      y[1] -= 2.*M_PI;
    if(y[1] < -M_PI)
      y[1] += 2.*M_PI;

    ekf_.UpdateEKF(y);
  } else
  {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  previous_timestamp_ = measurement_pack.timestamp_,

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
