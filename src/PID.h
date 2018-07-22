#ifndef PID_H
#define PID_H

//#define PRINT_INIT
//#define PRINT_UPDATE_ERROR
#define PRINT_VALUES
#define PRINT_PARAMS_OPTIMIZER
//#define PRINT_PARAMS_OPTIMIZER2
//#define PRINT_PARAMS_OPTIMIZER3
//#define PARAMS_OPTIMIZER
#define PRINT_FINAL_PARAMS
#define PRINT_DEBUG



class PID {
public:
  enum{
  TWIDDLE_UPDATE_PARAMS_STEP1,
  TWIDDLE_UPDATE_PARAMS_STEP2,
  TWIDDLE_RUN,
  TWIDDLE_EVALUATE,
  TWIDDLE_INIT,
  TWIDDLE_END,
  TWIDDLE_STOP
  };

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double previous_cte;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double dKp;
  double dKi;
  double dKd;
  
  int num_samples;
  int maq_Twiddle;
  int maq_Twiddle_previous;
  int paramCount;
  int num_max;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Parameters Optimizer
  */
  void InitParamsOptimizer();

 double Twiddle(double cte,double speed);

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
