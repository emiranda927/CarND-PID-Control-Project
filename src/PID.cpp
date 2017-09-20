#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double dKp, double dKi, double dKd, int process_step, char param) {

  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  PID::dKp = dKp;
  PID::dKi = dKi;
  PID::dKd = dKd;


  PID::process_step = process_step;
  PID::param = param;

  p_error = i_error = d_error = 0.0;

}

void PID::UpdateError(double cte) {

  if(!is_init){
    prev_cte_ = 0.;
    is_init = true;
  }

  p_error = cte;
  i_error += cte;
  d_error = cte - prev_cte_;
  prev_cte_ = cte;
}

double PID::TotalError() {

  return -Kp*p_error - Ki*i_error - Kd*d_error;
}

