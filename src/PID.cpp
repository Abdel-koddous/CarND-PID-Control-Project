#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  best_score = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  // p_error holds previous_cte
  d_error = cte - p_error; // Assuming delta_t between two measurements is 1 ... ?
  p_error = cte;
  i_error += cte;

}

void PID::Twiddle(double numberOfSteps) {
  /**
   * Updates PID coef using twiddle algorithm
   */

  // numberOfSteps is the result of the last car run

  switch ( twiddle_progress )
  {
  case /* constant-expression */:
    /* code */
    break;
  
  default:
    break;
  }
  Kp += dp[0];
  // re - run car ... AFTER INCREASING Kp

  if (best_score < numberOfSteps)
  {
    best_score = numberOfSteps;
    dp[0] *= 1.1;
  }
  else
  {
    Kp -= 2*dp[0];
    
    // re - run car ... AFTER DECREASING Kp
    if( best_score < numberOfSteps )
    {
      best_score = numberOfSteps;
      dp[0] *= 1.1;
    }
    else
    {
      Kp += dp[0];
      dp[0] *= 0.9;
    }
    
  }
  
  Kp += 0.05;
  
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double totalError = - ( Kp * p_error + Kd * d_error + Ki * i_error ); 

  return totalError;  // TODO: Add your total error calc here!
}