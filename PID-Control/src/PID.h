#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error = 0;
  double i_error = 0;
  double d_error = 0;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double prev_cte = 0;
  double itg_cte = 0;

  int step = 0;
  double total_cte = 0.0;
  double mean_cte = 0.0;
  double max_cte =0.0;
  double min_cte =5;
  double last_steer = 0.0;
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

  void Init(double _Kp, double _Ki, double _Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


};

#endif /* PID_H */
