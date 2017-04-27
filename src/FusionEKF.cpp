#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  // Measurement matrix laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Measurement matrix jacobian radar
  Hj_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float range = measurement_pack.raw_measurements_[0];
      float bearing = measurement_pack.raw_measurements_[1];
      // Transform to Cartesian coordinates
      float px = range * cos(bearing);
      float py = range * sin(bearing);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[0];
    }

    ekf_.x_ << px, py, 1, 1;

    // Initialize state covariance matrix
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = << 1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;

  float noise_ax = 9;
  float noise ay = 9;

  ekf_.Q_(0, 0) = pow(t, 4) / 4 * noise_ax;
  ekf_.Q_(0, 1) = 0;
  ekf_.Q_(0, 2) = pow(t, 3) / 2 * noise_ax;
  ekf_.Q_(0, 3) = 0;
  ekf_.Q_(1, 0) = 0;
  ekf_.Q_(1, 1) = pow(t, 4) / 4 * noise_ay;
  ekf_.Q_(1, 2) = 0;
  ekf_.Q_(1, 3) = pow(t, 3) / 2 * noise_ay;
  ekf_.Q_(2, 0) = pow(t, 3) / 2 * noise_ax;
  ekf_.Q_(2, 1) = 0;
  ekf_.Q_(2, 2) = pow(t, 2) * noise_ax;
  ekf_.Q_(2, 3) = 0;
  ekf_.Q_(3, 0) = 0;
  ekf_.Q_(3, 1) = pow(t, 3) / 2 * noise_ay;
  ekf_.Q_(3, 2) = 0;
  ekf_.Q_(3, 3) = pow(t, 2) * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
