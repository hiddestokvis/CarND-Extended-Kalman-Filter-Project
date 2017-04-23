#include <iostream>
#include <math.h>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Declare RMSE vector and set to (0,0,0,0)
  VectorXd RMSE(4);
  RMSE << 0, 0, 0, 0;

  // Check if estimations size is above 0 and if
  // the estimations vector is the same size as the
  // ground truth vector
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Invalid estimation or ground truth vector size" << endl;
    return RMSE;
  }

  // Sum squared residuals
  VectorXd residual_sum;
  for (int i = 0; i < estimations.size(); i += 1) {
    VectorXd residual = estimations[i] - ground_truth[i];
    VectorXd residual_squared = residual.array() * residual.array();
    residual_sum += residual_squared;
  }

  // Calculate the mean
  VectorXd residual_mean = residual_sum / estimations.size();
  // Calculate the square root
  RMSE = residual_mean.array().sqrt();

  // Return the RMSE;
  return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Define Jacobian Matrix
  MatrixXd Hj(3, 4);

  // State parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Pre compute set of repeatedly used terms
  float c1 = pow(px, 2) + pow(py, 2);
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // Prevent division by zero
  if (fabs(c1) < 0.0001) {
    cout << "Error in Jacobian Matrix - Division by Zero" << endl;
    return Hj;
  }

  // Calculate Jacobian Matrix
  Hj(0, 0) = px / c2;
  Hj(0, 1) = py / c2;
  Hj(0, 2) = 0.0;
  Hj(0, 3) = 0.0;
  Hj(1, 0) = -(py / c1);
  Hj(1, 1) = px / c1;
  Hj(1, 2) = 0.0;
  Hj(1, 3) = 0.0;
  Hj(2, 0) = py * (vx * py - vy * px) / c3;
  Hj(2, 1) = px * (vy * px - vx * py) / c3;
  Hj(2, 2) = px / c2;
  Hj(2, 3) = py / c2;

  // Return Jacobian Matrix
  return Hj;
}
