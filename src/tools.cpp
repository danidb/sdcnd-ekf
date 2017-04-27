#include <iostream>
#include "tools.h"
#include <math.h> // pow, sqrt
#include <assert.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /** Calculate root mean squared error. 
   * Straight from the lessons.
   */
  assert(estimations.size() > 0); 
  assert(estimations.size() == ground_truth.size()); 

  // collect squared error, compute the mean
  for(int i=0; i < estimations.size(); ++i){
    
    VectorXd rmse_working = estimations[i] - ground_truth[i]; 
    rmse_working = rmse_working.array() * rmse_working.array();
    rmse += rmse_working;
  }  
  rmse = rmse.array() / estimations.size();

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
  
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float sqsum   = pow(px, 2) + pow(py, 2);
  float sqsqsum = sqrt(sqsum);

  MatrixXd jacobian = MatrixXd(3, 4);
  jacobian << px/sqsqsum, py/sqsqsum, 0, 0,
    -1*py/sqsum, px/sqsum, 0,  0,
    py*(vx*py - vy*px)/pow(sqsum, 3/2), px*(vy*px + vx*py)/pow(sqsum, 3/2), px/sqsqsum,  py/sqsqsum;

  return jacobian;
}
    

