/* PID.cpp
 * Implementation based on: http://www.mstarlabs.com/apeng/techniques/pidsoftw.html
 * Copyright (C) 2019 Szymon Szantula
 *
 * Distributed under the MIT license.
 * For full terms see the file LICENSE.md.
 */
#include "PID.h"

#if 0
static const PID_params_t DEFAULT_PARAMS = {
    .flags = PIDController::PID_INT_RATE_LIMIT | PIDController::PID_CLEAN_INT_SUM | PIDController::PID_INT_SOFT_ANTI_WINDUP,
    .ki = 0,
    .kp = 0.001,
    .kd = 0,
    .bias = 0,
    .anti_windup = 0.05,
    .windup_lim = 2000,
    .int_rate_lim = 100, 
    .out_min = -1,
    .out_max = 1,
    .in_min = -500,
    .in_max = 500, 
    .clean_feedback_lim =
    .clean_rate = 
    .dt = 0.05
};
#endif

PIDController::~PIDController(){}

PIDController::PIDController(const PID_params *params)
{
    _params = *params;
    _kd = _params.kd / _params.dt;
    _ki = _params.ki * _params.dt;
    reset();
}

int PIDController::updateState(float curr_setpoint, float curr_feedback, float * pidout)
{
    // setpoint saturation
    bool setpoint_limit = false, feedback_limit = false;
    if(curr_setpoint >= _params.in_max)
    {
        curr_setpoint = _params.in_max;
        setpoint_limit = true;
    }
    else if (curr_setpoint <= _params.in_min)
    {
        curr_setpoint = _params.in_min;
        setpoint_limit = true;
    }

    // feedback limit check
    if(curr_feedback >= _params.in_max)
    {
        feedback_limit = true;
    }
    else if (curr_feedback <= _params.in_min)
    {
        feedback_limit = true;
    }

    // Compute error
    float error = (curr_setpoint - curr_feedback);

    // Compute proportional and integral part
    float pidout_internal = _params.kp*error;
    
    // Compute integral part
    pidout_internal += _state.int_sum;

    float dchange = error - _state.last_error;
    _state.last_last_error = _state.last_error;
    if (_params.flags & PID_DERIV_RESP_GLITCHES_FIX)
    {
        if (_params.flags & PID_AVG_FILTER)
            dchange = (curr_feedback - _state.last_last_error) / 2;
        else
            dchange = (curr_feedback - _state.last_error);
        _state.last_error = curr_feedback;
    }
    else
    {
        if (_params.flags & PID_AVG_FILTER)
            dchange = (error - _state.last_last_error) / 2;
        _state.last_error = error;
    }

    // Compute derivative part
    pidout_internal += (_kd * dchange);

    // Compute ichange and applay integration rate limit
    float ichange = error;
    if (_params.flags & PID_INT_RATE_LIMIT)
    {
        if (ichange > _params.int_rate_lim)
            ichange = _params.int_rate_lim;
        else if (ichange < -_params.int_rate_lim)
            ichange = -_params.int_rate_lim;
    }

    // Scale output and applay bias
    pidout_internal += _params.bias;

    // output saturation
    pidout_internal = (pidout_internal > _params.out_max ? _params.out_max : (pidout_internal < _params.out_min ? _params.out_min : pidout_internal) );

    // anti windup
    if (_params.flags & PID_INT_SOFT_ANTI_WINDUP)
    {
        if (fabs(_state.int_sum) >= _params.windup_lim )
            _state.int_sum += _ki*(_params.anti_windup * ichange);
        else
            _state.int_sum += _ki*ichange;
    }

    // int sum autoclean
    if (_params.flags & PID_CLEAN_INT_SUM)
    {
        if(curr_setpoint == 0.0f && fabs(curr_feedback) <= _params.clean_feedback_lim)
        {
            float tmp = _ki*_params.clean_rate;
            if(_state.int_sum > tmp)
                _state.int_sum -= tmp;
            else if(_state.int_sum < -tmp)
                _state.int_sum += tmp;
            else 
                _state.int_sum = 0.0;
        }
    }

    *pidout = _state.last_pidout = pidout_internal;
    if(feedback_limit)
        return PID_ERROR_FEEDBACK_LIMITS;
    else if(setpoint_limit)
        return PID_ERROR_SETPOINT_LIMITS;
    else
        return PID_COMPUTATION_SUCCESS;
}

void PIDController::updateParams(const PID_params_t * params)
{
    if(params->ki == 0.0f)
        _state.int_sum = 0;
    else if(_params.ki != 0 )
        _state.int_sum *= (params->ki/_params.ki);  
    _params = *params;
    _kd = _params.kd / _params.dt;
    _ki = _params.ki * _params.dt;
}

void PIDController::getParams(PID_params_t * params)
{
    *params = _params;
}

void PIDController::getState(PID_state_t * state)
{
    *state = _state;
}

void PIDController::reset()
{
    _state.int_sum = 0.0f;
    _state.last_error = 0.0f;
    _state.last_last_error = 0.0f;
    _state.last_pidout = 0.0f;
}

void PIDController::updateInterval(float dt)
{
    // update internal parameters:
    if(dt<=0)
        return;
    _ki *= (dt/_params.dt);
    _kd *= (dt/_params.dt);
    _state.last_error *= (_params.dt/dt);
    _state.last_last_error *= (_params.dt/dt);
    _params.dt = dt;
}

void PIDController::updateTuning(float kp, float kd, float ki)
{
    if( ki == 0.0f)
        _state.int_sum = 0;
    else if( _params.ki != 0)
        _state.int_sum *= (ki/_params.ki);
    _params.kp = kp;
    _params.kd = kd;
    _params.ki = ki;
    _kd = kd / _params.dt;
    _ki = ki * _params.dt;
}