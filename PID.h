/* PID.h
 * Implementation based on: http://www.mstarlabs.com/apeng/techniques/pidsoftw.html
 * Copyright (C) 2019 Szymon Szantula
 *
 * Distributed under the MIT license.
 * For full terms see the file LICENSE.md.
 */
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>


typedef enum
{
    PID_OUTPUT_LIMITS = 0x01,        ///< enable output limits
    PID_INT_SOFT_ANTI_WINDUP = 0x02, ///< enable soft anti-windup reduction strategy
    PID_INT_RATE_LIMIT = 0x04,       ///< enable integral part limitation
    PID_AVG_FILTER = 0x08,           ///< enable derivative average filter
    PID_ERROR_TOLERANCE = 0x10,      ///<
    PID_BIAS_BALANCE = 0x20,         ///<
    UPDATE_ALL,                      ///< copy all params
    TUNING_ONLY                      ///< ki, kp, kd and scale
};

typedef struct PID_params
{
    uint8_t flags;             ///<
    float ki;                  ///< Integral part constant
    float kp;                  ///< Proportional part constant
    float kd;                  ///< Derivative part constant
    float scale;               ///< tuning scale
    float del_kp;              ///< balance factor
    float anti_windup;         ///<
    float int_rate_limit_high; ///<
    float int_rate_limit_low;  ///<
    float pidout_limit_high;   ///<
    float pidout_limit_low;    ///<
    float error_tolerance;     ///<
} PID_params_t;

typedef struct PID_state
{
    float int_sum;
    float last_error;
    float last_last_error;
    float last_setpoint;
    float pidout;
    PID_state()
    : int_sum(0.0f)
    , last_error(0.0f)
    , last_last_error(0.0f)
    , last_setpoint(0.0f)
    , pidout(0.0f)
    {}

    void reset()
    {
        int_sum = pidout = last_error = last_last_error = 0.0f;
    }
}PID_state_t;


class PIDController 
{
    public:
        PIDController();
        PIDController(const PID_params_t & params);
        ~PIDController();

        void updateParams(const PID_params_t & params, uint8_t update_flag)
        {
            if(update_flag == UPDATE_ALL)
                _params = params;
            else if(update_flag == TUNING_ONLY)
            {
                _params.kp = params.kp; 
                _params.kd = params.kd; 
                _params.ki = params.ki; 
                _params.scale = params.ki; 
            }
        }

        void getParams(PID_params_t & params)
        {
            params = _params;
        }

        void getState(PID_state_t & state)
        {
            state = _state;
        }

        int updateState(const float * curr_setpoint, const float * curr_feedback, float * pidout, uint32_t dt);

        void reset()
        {
            _state.reset();
        }
        
    private:
        PID_params _params;
        PID_state _state;
};
#endif /*__PID_H__ */