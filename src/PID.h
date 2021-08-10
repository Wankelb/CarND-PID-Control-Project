#ifndef PID_H
#define PID_H

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

  void Twiddle(double error, int idx);

  bool enableTwiddle();

  double getParameter(int i);

 //private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp[3] = {0, 0, 0.10};

  double Kd[3] = {0.0001, 0.0001, 0.0001};

  double Kl[3];

  double best_err = 0.0;
  bool first_time = true;
  bool restart = false;
  double prev_cte = 0.0;
  double tol = 0.000005;
  bool twiddle = false;
};

#endif  // PID_H