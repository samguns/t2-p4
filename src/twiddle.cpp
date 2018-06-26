//
// Created by Gang Wang on 6/26/2018.
//
#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "json.hpp"

#include "twiddle.h"

#define TOLERENCE         0.02

#define MAX_TRIALS        200

#define PARAMS_KP_IDX     0
#define PARAMS_KI_IDX     1
#define PARAMS_KD_IDX     2

#define STATE_INIT_GUESS_BEST   0
#define STATE_TWIDDLE_BEST      1
#define STATE_TWIDDLE_FINISHED  2

#define TWIDDLE_STATE_GAIN            0
#define TWIDDLE_STATE_GAIN_ASCEND     1
#define TWIDDLE_STATE_GAIN_DESCEND    2

using json = nlohmann::json;
using namespace std;

void Twiddle::Init() {
  m_p[PARAMS_KP_IDX] = 0.2;
  m_p[PARAMS_KI_IDX] = 0;
  m_p[PARAMS_KD_IDX] = 0.2;
  m_dp[PARAMS_KP_IDX] = 0.1;
  m_dp[PARAMS_KI_IDX] = 0.1;
  m_dp[PARAMS_KD_IDX] = 0.1;
  m_State = STATE_INIT_GUESS_BEST;
  m_Trials = 0;
  m_Error = 0;
  m_BestError = 0;
  m_StateTwiddleGains = TWIDDLE_STATE_GAIN;
  m_TwiddleIdx = 0;

  m_PID.Init(m_p[PARAMS_KP_IDX], m_p[PARAMS_KI_IDX], m_p[PARAMS_KD_IDX]);
}


void Twiddle::Process(uWS::WebSocket<uWS::SERVER>& ws, double cte, double throttle) {
  double err = 0;
  double dp_sum = 0;
  m_WS = ws;

  switch (m_State) {
    case STATE_INIT_GUESS_BEST:
      err = run(cte, throttle);

      if (m_Trials >= 2 * MAX_TRIALS) {
        m_BestError = err / MAX_TRIALS;

        /**
         * Reset PID controller and simulator
         */
        m_PID.Init(m_p[PARAMS_KP_IDX], m_p[PARAMS_KI_IDX], m_p[PARAMS_KD_IDX]);
        reset();

        m_State = STATE_TWIDDLE_BEST;
        m_Trials = 0;
        printf("%s[%d] m_BestError: %f\n", __func__, __LINE__, m_BestError);
      }

      break;

    case STATE_TWIDDLE_BEST:
      dp_sum = m_dp[PARAMS_KP_IDX] + m_dp[PARAMS_KI_IDX] + m_dp[PARAMS_KD_IDX];
      printf("%s[%d] dp_sum: %f\n", __func__, __LINE__, dp_sum);
      if (dp_sum <= TOLERENCE) {
        /**
         * Reset PID controller and simulator
         */
        m_PID.Init(m_p[PARAMS_KP_IDX], m_p[PARAMS_KI_IDX], m_p[PARAMS_KD_IDX]);
        reset();

        m_State = STATE_TWIDDLE_FINISHED;
      } else {
        twiddle_gains(cte, throttle);
      }

      break;

    case STATE_TWIDDLE_FINISHED:
    default:
      run(cte, throttle);
      break;
  }
}


/**
 * Private member function
 */
double Twiddle::run(double cte, double throttle) {
  m_PID.UpdateError(cte);

  double steer = -m_PID.TotalError();
  move(steer, throttle);

  m_Trials++;
  if (m_Trials >= MAX_TRIALS) {
    m_Error += pow(cte, 2);
  }

  return m_Error;
}


void Twiddle::move(double steer, double throttle) {
  json msgJson;
  msgJson["steering_angle"] = steer;
  msgJson["throttle"] = throttle;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  m_WS.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void Twiddle::reset() {
  std::string msg = "42[\"reset\",{}]";
  m_WS.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void Twiddle::twiddle_gains(double cte, double throttle) {
  double err = 0;
  switch (m_StateTwiddleGains) {
    case TWIDDLE_STATE_GAIN:
      m_p[m_TwiddleIdx] += m_dp[m_TwiddleIdx];
      m_PID.Init(m_p[PARAMS_KP_IDX], m_p[PARAMS_KI_IDX], m_p[PARAMS_KD_IDX]);
      reset();
      m_Trials = 0;
      m_StateTwiddleGains = TWIDDLE_STATE_GAIN_ASCEND;
      break;

    case TWIDDLE_STATE_GAIN_ASCEND:
      err = run(cte, throttle);

      if (m_Trials >= 2 * MAX_TRIALS) {
        err /= MAX_TRIALS;

        if (err < m_BestError) {
          m_BestError = err;
          m_dp[m_TwiddleIdx] *= 1.1;

          m_TwiddleIdx++;
          m_TwiddleIdx %= PARAMS_LEN;
          printf("%s[%d] m_BestError: %f\n", __func__, __LINE__, m_BestError);
          printf("%s[%d] p: %f %f %f dp: %f %f %f\n", __func__, __LINE__,
                 m_p[0], m_p[1], m_p[2], m_dp[0], m_dp[1], m_dp[2]);
        } else {
          m_p[m_TwiddleIdx] -= 2 * m_dp[m_TwiddleIdx];
          m_PID.Init(m_p[PARAMS_KP_IDX], m_p[PARAMS_KI_IDX], m_p[PARAMS_KD_IDX]);
          reset();

          m_Trials = 0;
          m_StateTwiddleGains = TWIDDLE_STATE_GAIN_DESCEND;
          break;
        }
      }

      break;

    case TWIDDLE_STATE_GAIN_DESCEND:
      err = run(cte, throttle);

      if (m_Trials >= 2 * MAX_TRIALS) {
        err /= MAX_TRIALS;

        if (err < m_BestError) {
          m_BestError = err;
          m_dp[m_TwiddleIdx] *= 1.1;
        } else {
          m_p[m_TwiddleIdx] += m_dp[m_TwiddleIdx];
          m_dp[m_TwiddleIdx] *= 0.9;
        }
        printf("%s[%d] m_BestError: %f\n", __func__, __LINE__, m_BestError);
        printf("%s[%d] p: %f %f %f dp: %f %f %f\n", __func__, __LINE__,
               m_p[0], m_p[1], m_p[2], m_dp[0], m_dp[1], m_dp[2]);

        m_PID.Init(m_p[PARAMS_KP_IDX], m_p[PARAMS_KI_IDX], m_p[PARAMS_KD_IDX]);
        reset();

        m_Trials = 0;

        m_StateTwiddleGains = TWIDDLE_STATE_GAIN;

        m_TwiddleIdx++;
        m_TwiddleIdx %= PARAMS_LEN;
      }

      break;

    default:
      break;
  }
}
