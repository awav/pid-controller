#ifndef PID_H
#define PID_H

#include <cmath>
#include <limits>

class PID {
public:
  /*
   * Constructor
   */
  PID();

  /*
   * Construct PID.
   * param kp: proportional coefficient of PID controller
   * param ki: integral coefficient of PID controller
   * param kd: differential coefficient of PID controller
   * param out_min: PID output bottom boundary
   * param out_max: PID output top boundary
   */
  PID(double kp, double ki, double kd, double out_min = NAN,
      double out_max = NAN);

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Update the PID error variables given cross track error.
   * param cte: cross-track error
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

private:
  double bound_minmax(double value) const;

private:
  // const double alpha_i_error_ = 0.9;

  /*
   * Errors:
   * p_term_ - proportional term
   * i_term_ - integral term
   * d_term_ - differential term
   */
  double p_term_ = 0;
  double i_term_ = 0;
  double d_term_ = 0;

  /*
   * Coefficients:
   * kp_ - proportional coefficient
   * ki_ - integral coefficient
   * kd_ - differential coefficient
   */
  double kp_ = 0;
  double ki_ = 0;
  double kd_ = 0;

  /* Min and max boundaries of ouput */
  double min_ = std::numeric_limits<double>::min();
  double max_ = std::numeric_limits<double>::max();

  double pid_last_ = NAN;
  double cte_last_ = NAN;
};

#endif /* PID_H */
