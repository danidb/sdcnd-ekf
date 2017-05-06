#include <iostream>
#include "tools.h"
#include <math.h> // pow, sqrt, arcan
#include <assert.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /** Calculate root mean squared error. 
   * Straight from the lessons.
   */
  assert(estimations.size() > 0); 
  assert(estimations.size() == ground_truth.size()); 

  VectorXd rmse = VectorXd(4);
  rmse <<  0, 0, 0, 0;
  
  // collect squared error, compute the mean
  for(int i=0; i < estimations.size(); ++i){
    
    VectorXd rmse_working = estimations[i] - ground_truth[i]; 
    rmse_working = rmse_working.array() * rmse_working.array();
    rmse += rmse_working;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse; 
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian given a 4D state vector. 
    * This is straight from the lessons. 
  */
 
  // Minimal  input check just to  make sure we're not doing anything crazy
  assert(x_state.size() == 4);
  
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double sqsum   = pow(px, 2) + pow(py, 2);
  double sqsqsum = sqrt(sqsum);

  // tolerance for zero valus 
  if ( fabs(sqsum) < 0.00001 ) {
    sqsum = 0.00001;
  }

  MatrixXd jacobian = MatrixXd(3, 4);
  jacobian << px/sqsqsum, py/sqsqsum, 0, 0,
    -py/sqsum, px/sqsum, 0,  0,
    py*(vx*py - vy*px)/pow(sqsum, 3.0/2.0), px*(vy*px - vx*py)/pow(sqsum, 3.0/2.0), px/sqsqsum,  py/sqsqsum;

  return jacobian;
}


VectorXd Tools::CartesianToPolar(const VectorXd& x) {
  //  a self-explanatory method
  assert(x.size() == 4);
  VectorXd polar_output = VectorXd(3);

  polar_output << sqrt(pow(x[0], 2) + pow(x[1], 2)),
    atan2(x[1], x[0]),
    (x[0]*x[2] + x[1]*x[3])/sqrt(pow(x[0], 2) + pow(x[1],  2));
  return polar_output;  
}


float Tools::WrapAnglePi(const float angle)  {
  // scale module 2pi to  wrap the angle between [-pi, pi]
  float angle_result = angle;
  if ( angle > 0 ) { 
    angle_result = fmod(angle + M_PI, 2*M_PI) - M_PI;
  } else {
    angle_result = fmod(angle - M_PI, 2*M_PI) + M_PI;
  }
  
  return angle_result;
}
   
    
