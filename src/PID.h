#ifndef PID_H
#define PID_H

class PID {
public:
  bool      is_twiddle; 
  float     twiddle_tol;
  int       twiddle_endstep;
  int       twiddle_iter;
  float     twiddle_best_sse;
  float     d_gain[3]; 
  float     gain[3];   // current gain, convention Kp, Ki, Kd
  int       curr_param;
  char      state;
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

  void Twiddle(double SSE);
};

#endif /* PID_H */
