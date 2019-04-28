#include "kalman_filter.h"
#include <iostream>

using std::cout;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

//  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  const float PI = 3.1415927;
  // we make sure that the angle phi of the radar measurement is normalized between -pi and pi
  VectorXd z_copy(3);
  z_copy << z(0), z(1), z(2);
  while(z_copy(1) > PI || z_copy(1) < -PI){
    if (z_copy(1) > PI)
      z_copy(1) -= 2*PI;
    else if (z(1) < -PI)
      z_copy(1) += 2*PI;
  }
  // Calculate h(x')
	float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
	float phi = atan2(x_(1), x_(0));  // returns value in range [-pi, pi]
	float rho_dot = 0.0;
	if (fabs(rho) > 1e-6) {	// prevent divide by 0
		rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
	}
	VectorXd hx(3);	// h(x')
	hx << rho, phi, rho_dot;

	VectorXd y = z_copy - hx;
  // we make sure that the angle phi of the resulting y vector is normalized between -pi and pi
	while(y(1) > PI || y(1) < -PI){
    if (y(1) > PI)
      y(1) -= 2*PI;
    else if (y(1) < -PI)
      y(1) += 2*PI;
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

//  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
