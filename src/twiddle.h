//
// Created by Gang Wang on 6/26/2018.
//

#ifndef T2_P4_TWIDDLE_H
#define T2_P4_TWIDDLE_H

#include <uWS/uWS.h>

#include "PID.h"

#define PARAMS_LEN        3

class Twiddle {
 public:
  Twiddle() {};
  ~Twiddle() {};

  void Init();
  void Process(uWS::WebSocket<uWS::SERVER>& ws, double cte, double throttle);

 private:
  int m_State;
  int m_Trials;
  double m_BestError;
  double m_Error;
  uWS::WebSocket<uWS::SERVER> m_WS;
  PID m_PID;

  double m_p[PARAMS_LEN];
  double m_dp[PARAMS_LEN];

  int m_StateTwiddleGains;
  int m_TwiddleIdx;

  double run(double cte, double throttle);
  void move(double steer, double throttle);
  void reset();
  void twiddle_gains(double cte, double throttle);
};

#endif //T2_P4_TWIDDLE_H
