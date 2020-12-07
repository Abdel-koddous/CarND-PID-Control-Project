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

  p = {Kp, Ki, Kd};
  dp = {0.01, 0, 0.5};

  twiddle_progress = 0;
  pidCoef_index = 0;

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

void PID::Twiddle( double numberOfSteps ) {
  /**
   * Updates PID coef using twiddle algorithm
   */

  // numberOfSteps is the result of the last car run
  //DEBUG
  std::cout << "--- Twiddle Algorithm step running --- Step " << twiddle_progress << " --- " 
  << std::endl;
  
  switch ( twiddle_progress )
  {
    
  case 0:

    std::cout << " --- Fine tuning pid index " << pidCoef_index << " ---" << std::endl;
    p[pidCoef_index] += dp[pidCoef_index];
    twiddle_progress = 1;

    if( best_score < numberOfSteps ) { best_score = numberOfSteps; }
    // reset simulator
    break;

  case 1:

    if (best_score < numberOfSteps)
    {
      std::cout << "Found a new best score ... let's bump UP !" << std::endl;

      best_score = numberOfSteps;
      dp[pidCoef_index] *= 1.1; // increase the amount of change even more - Bump Up
      
      pidCoef_index = ( pidCoef_index + 1 ) % 3; // move to next pid coef
      pidCoef_index = (pidCoef_index == 1) ? 2 : pidCoef_index; // Not tuning Ki

      p[pidCoef_index] += dp[pidCoef_index];

      twiddle_progress = 1;
    }
    else
    {
      std::cout << "Not a best score let's bump down" << std::endl;

      p[pidCoef_index] -= 2*dp[pidCoef_index]; // explore the other side ( negative ) of Coef value - Bump Down
      
      twiddle_progress = 2;
    }

    // reset simulator
    break;
  
  case 2:

    if( best_score < numberOfSteps )
    {
      std::cout << "Found a new best score after bumping DOWN ! " << std::endl; 

      best_score = numberOfSteps;
      dp[pidCoef_index] *= 1.1;
    }
    else
    {
      std::cout << "Bumping DOWN didn't give a better score either ... Reducing my intervall" << std::endl;

      p[pidCoef_index] += dp[pidCoef_index]; // go back to initial Kp
      dp[pidCoef_index] *= 0.9;
    }
    
    pidCoef_index = ( pidCoef_index + 1 ) % 3; // move to next pid coef  
    pidCoef_index = (pidCoef_index == 1) ? 2 : pidCoef_index; // Not tuning Ki
  
    p[pidCoef_index] += dp[pidCoef_index];

    twiddle_progress = 1;
    // no break here to jump to case 0 before resetting simulator

  default:
    break;
  }

  Kp = p[0];
  Ki = p[1];
  Kd = p[2];

  // DEBUG
  std::cout << "Parameters after twiddle algorithm " << Kp << " - " << Ki << " - " << Kd << 
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