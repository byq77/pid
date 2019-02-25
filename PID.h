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

typedef enum PID_flags
{
    PID_INT_SOFT_ANTI_WINDUP = 0x01, ///< enable soft anti-windup reduction strategy
    PID_INT_RATE_LIMIT = 0x02,       ///< enable integral part limitation
    PID_AVG_FILTER = 0x04,           ///< enable derivative average filter
    PID_DERIV_RESP_SMOOTING = 0x08,  ///< applay fix for derivative response
    PID_END_REG_JOB_SUCCESS = 0x10   ///< reserved
} PID_flags_t;

typedef struct PID_params
{
    uint8_t flags;        ///< enable PID functionalities
    float ki;             ///< integral part constant
    float kp;             ///< proportional part constant
    float kd;             ///< derivative part constant
    float bias;           ///< controller output bias
    float anti_windup;    ///< anti-windup compensation factor
    float int_rate_limit; ///< maximum integral update rate
    float out_min;        ///< upper output limit
    float out_max;        ///< lower output limit
    float in_min;         ///< upper input limit
    float in_max;         ///< lower input limit
    float mdr;            ///< reserved
    int mdr_spins;        ///< reserved
    float dt;             ///< update interval [sec]
} PID_params_t;

typedef struct PID_state
{
    float int_sum;
    float last_error;
    float last_last_error;
    int mdr_spins_count;
} PID_state_t;


class PIDController 
{
    public:
        PIDController();
        PIDController(const PID_params_t & params);
        ~PIDController();
        void updateParams(const PID_params_t & params);
        void getParams(PID_params_t & params);
        void getState(PID_state_t & state);
        void reset();
        int updateState(const float * curr_setpoint, const float * curr_feedback, float * pidout);

        
        void updateInterval(float dt);//TODO: implement !affects ki, kd, kp in _params
        void updateTuning(float kp, float kd, float ki); //TODO: implement

    private:
        PID_params_t _params;
        PID_state_t _state;
        float _in_span;
        float _out_span;
};
#endif /*__PID_H__ */