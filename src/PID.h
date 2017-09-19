#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double dKp;
  double dKi;
  double dKd;

  double best_error;
  double error;

  int process_step;
  char param;
  int avg_count = 0;

  bool is_init = false;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double dKp, double dKi, double dKd, int process_step, char param);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  /*
   * private variable for internal calculations
   */
  double prev_cte_;
};

#endif /* PID_H */
