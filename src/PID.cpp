#include "PID.h"
#include <iostream>

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
  case 0:

    Kp += dp[0];
    twiddle_progress = 1;

    break;

  case 1:

    if (best_score < numberOfSteps)
    {
      best_score = numberOfSteps;
      dp[0] *= 1.1; // increase the amount of change even more - Bump Up
      Kp += dp[0];
      
      twiddle_progress = 1; 
      std::cout << "Found a new best score ... let's bump UP !" << std::endl;
    }
    else
    {
      Kp -= 2*dp[0]; // explore the other side ( negative ) of Coef value - Bump Down
      twiddle_progress = 2;
      std::cout << "Not a best score let's bump down" << std::endl;
    }

    break;

  
  case 2:

    if( best_score < numberOfSteps )
    {
      best_score = numberOfSteps;
      dp[0] *= 1.1;
      Kp += dp[0];
      std::cout << "Found a new best score after bumping DOWN ! " << std::endl; 
      
    }
    else
    {
      Kp += dp[0]; // go back to initial Kp
      dp[0] *= 0.9;
      Kp += dp[0];
      std::cout << "Bumping DOWN didn't give a better score either ... Reducing my intervall" << std::endl;
    }
    
    twiddle_progress = 1;

    break;

  default:
    break;
  }

  // DEBUG
  std::cout << "Parameters after twiddle algorithm " << Kp << 
  " - Best score " << best_score << " - Current score " << numberOfSteps << std::endl;

}

std::vector<double> PID::getPIDCoefs() {
  /**
   * A getter for the PID coefs
   */
  std::vector<double> pidCoefs = {Kp, Ki, Kd};

  return pidCoefs;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double totalError = - ( Kp * p_error + Kd * d_error + Ki * i_error ); 

  return totalError;  // TODO: Add your total error calc here!
}