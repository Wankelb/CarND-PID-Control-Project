#include "PID.h"
#include<iostream>
#include <math.h>

using std::string;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp[0] = Kp_;
  Kp[1] = Ki_;
  Kp[2] = Kd_;



}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  double prev_p_error = p_error;
  p_error = cte;
  d_error = cte - prev_p_error;
  i_error += cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return -Kp[0] * p_error - Kp[2] * d_error - Kp[1] * i_error;  // TODO: Add your total error calc here!
}

bool PID::enableTwiddle() {
  return twiddle;
}

double PID::getParameter(int i) {
  return Kp[i];
}

void PID::Twiddle(double error, int idx) {
  if (first_time) {
    best_err = error;
    first_time = false;
    return;
  } 

  if (fabs(Kd[idx]) > tol) {
    if (restart) {
      Kl[idx] = Kp[idx];
      Kp[idx] += Kd[idx];
      i_error = 0;
      restart = false;   
    } else {
      if (error < best_err) {
        
        Kd[idx] *= 1.1;

        best_err = error;
        restart = true; 
      } else {
        if (fabs(Kl[idx]) < fabs(Kp[idx])) {
          Kl[idx] = Kp[idx];
          Kp[idx] -= 2.0 * Kd[idx]; 
        } else {
          Kl[idx] = Kp[idx];
          Kp[idx] += Kd[idx];
          Kd[idx] *= 0.9;
          restart = true;
        }

      }
    }

  }
}
