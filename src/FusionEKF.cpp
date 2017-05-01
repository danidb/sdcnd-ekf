#include "FusionEKF.h"
#include "tools.h"
#include "math.h"
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
  H_radar_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.0;

  // H_radar_ is the jacobian (later)
  H_laser_ << 1, 0, 0, 0,
    0, 1, 0, 0;
  
  // use the default constructor - we assign instance variables later.
  // Don't tell Steve McConnell...
  KalmanFilter ekf_ = KalmanFilter();

  return;
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
    
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // initial  covariance (process, dt = 0, t = 0)
    ekf_.Q_ = MatrixXd(4,4);
    ekf_.Q_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1000, 0, 0, 0,
      0, 1000, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // radius
      float rho = measurement_pack.raw_measurements_[0];
      // angle 
      float phi = measurement_pack.raw_measurements_[1];

      ekf_.x_ <<  rho * cos(phi), rho * sin(phi), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0; 
    }
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Predictionx
   ****************************************************************************/

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) /  1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0,
    0, 0, 0, 1;

  float noise_ax = 9.0;
  float noise_ay = 9.0;
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<
    pow(dt, 4)/4 * noise_ax, 0,                       pow(dt, 3)/3 * noise_ax, 0,    
    0,                       pow(dt, 4)/4 * noise_ay, 0,                       pow(dt, 3)/3 * noise_ay,
    pow(dt, 3)/2 * noise_ax, 0,                        pow(dt, 2)*noise_ax,    0, 
    0,                       pow(dt, 3)/2 * noise_ay,  0,                      pow(dt, 2) * noise_ay;
  
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    Tools tools; 
    H_radar_ = tools.CalculateJacobian(ekf_.x_);
    
    ekf_.H_ = H_radar_;
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {

    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1000, 0, 0, 0,
      0, 1000, 0, 0,
      0, 0, 10, 0,
      0, 0, 0, 10; 
    
    ekf_.Update(measurement_pack.raw_measurements_);    
  }

  // print the output
  cout << "x_ = " << ekf_.x_;
  cout << "P_ = " << ekf_.P_ << endl;
}
