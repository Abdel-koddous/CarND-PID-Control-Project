#ifndef PID_H
#define PID_H

#include<vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void Twiddle(double numberOfSteps);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * dp
   */

  std::vector<double> dp = {1, 1, 1};
  
  /**
   *  best error
   */
  double best_score; 

  /**
   *  twiddle progress parameter to track algorithm steps across multiple runs
   */
  int twiddle_progress = 0;
};

#endif  // PID_H