#ifndef PID_H
#define PID_H

class PID {
public:
  bool      is_twiddle; 
  bool      is_twiddle_init;
  float     twiddle_tol;
  int       twiddle_endstep;
  int       twiddle_cnt;
  float     twiddle_best_sse;
  float     d_gain[3]; 
  float     gain[3];    // current gain, convention Kp, Ki, Kd
  int       gain_idx;   // index to d_gain/gain, 0: P, 1: I, 2: D
  int       state;

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
