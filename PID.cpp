/* PID.cpp
 * Implementation based on: http://www.mstarlabs.com/apeng/techniques/pidsoftw.html
 * Copyright (C) 2019 Szymon Szantula
 *
 * Distributed under the MIT license.
 * For full terms see the file LICENSE.md.
 */
#include "PID.h"

static const PID_params_t DEFAULT_PARAMS; //TODO: add some default params

PIDController::PIDController()
    : _params(DEFAULT_PARAMS), _state()
{
    reset();
}

PIDController::~PIDController(){}

PIDController::PIDController(const PID_params * params)
: _params(params ? *params : DEFAULT_PARAMS), _state()
{
    reset();
}

int PIDController::updateState(const float * curr_setpoint, const float * curr_feedback, float * pidout)
{
    // Check if inputs are correct
    if(*curr_setpoint > _params.in_max || *curr_setpoint < _params.in_min)
        return PID_ERROR_SETPOINT_LIMITS; 
    if(*curr_feedback > _params.in_max || *curr_feedback < _params.in_min)
        return PID_ERROR_FEEDBACK_LIMITS; // Possible system failure, react immediately!

    // Compute error
    float error = *curr_setpoint - *curr_feedback;
    if(_params.flags & PID_END_REG_JOB_SUCCESS)
    {
        //TODO: implement
        return PID_REGULATION_END;
    }

    // Compute proportional and integral part
    float pidout_internal = _params.kp*error + _params.ki*_state.int_sum;
    
    // Compute dchange (derivative slope)
    float dchange;
    if(_params.flags & PID_DERIV_RESP_GLITCHES_FIX)
    {
        if(_params.flags & PID_AVG_FILTER)
            dchange = (*curr_feedback - _state.last_last_error)/2;
        else
            dchange = (*curr_feedback - _state.last_error);
        _state.last_last_error = _state.last_error; 
        _state.last_error = *curr_feedback;
    }
    else
    {
        if(_params.flags & PID_AVG_FILTER)
            dchange = (error - _state.last_last_error)/2;
        else
            dchange = (error - _state.last_error);
        _state.last_last_error = _state.last_error; 
        _state.last_error = error;
    }

    // Compute derivative part
    pidout_internal += (_params.kd * dchange/_params.dt);

    // Compute ichange and applay integration rate limit
    float ichange = error;
    if (_params.flags & PID_INT_RATE_LIMIT)
    {
        if (ichange > _params.int_rate_limit)
            ichange = _params.int_rate_limit;
        else if (ichange < -_params.int_rate_limit)
            ichange = -_params.int_rate_limit;
    }

    // Scale output and applay bias
    // pidout_internal = _params.bias + (pidout_internal * _out_span) + _params.out_min;
    pidout_internal += _params.bias;

    bool limit_high = (pidout_internal > _params.out_max);
    bool limit_low = (pidout_internal < _params.out_min);
    
    if(limit_high || limit_low)
        pidout_internal = (limit_high ? _params.out_max : _params.out_min);

    if ((_params.flags & PID_INT_SOFT_ANTI_WINDUP) && (limit_high || limit_low))
        _state.int_sum += _params.anti_windup * ichange * _params.dt;
    else
        _state.int_sum += ichange * _params.dt;

    *pidout = pidout_internal;
    return PID_COMPUTATION_SUCCESS;
}

void PIDController::updateParams(const PID_params_t * params)
{
    _params = *params;
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
}

void PIDController::updateInterval(float dt)
{
    // update internal parameters:
    if(dt<=0)
        return;
    _params.kp *= (dt/_params.dt);
    _params.ki *= (dt/_params.dt);
    _params.kd *= (dt/_params.dt);
    _state.last_error *= (dt/_params.dt);
    _state.last_last_error *= (dt/_params.dt);
    _params.dt = dt;
}

void PIDController::updateTuning(float kp, float kd, float ki)
{
    if( ki != 0.0f)
    {
        _state.int_sum *= (ki/_params.ki);
    }
    _params.kp =kp;
    _params.kd =kd;
    _params.ki =ki;
}