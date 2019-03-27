/* PID.cpp
 * Implementation based on: http://www.mstarlabs.com/apeng/techniques/pidsoftw.html
 * Copyright (C) 2019 Szymon Szantula
 *
 * Distributed under the MIT license.
 * For full terms see the file LICENSE.md.
 */
#include "PID.h"

static const PID_params_t DEFAULT_PARAMS = {
    .flags = PIDController::PID_INT_RATE_LIMIT,
    .ki = 0,
    .kp = 0.001,
    .kd = 0,
    .bias = 0,
    .anti_windup = 0,
    .int_rate_limit = 0.1, 
    .out_min = -1,
    .out_max = 1,
    .in_min = -500,
    .in_max = 500, 
    .dt = 0.05
};

PIDController::PIDController()
    : _params(DEFAULT_PARAMS), _state()
{
    _kd = _params.kd / _params.dt;
    _ki = _params.ki * _params.dt;
    reset();
}

PIDController::~PIDController(){}

PIDController::PIDController(const PID_params * params)
{
    if(params != NULL)
    {
        _params = *params;
        _kd = _params.kd / _params.dt;
        _ki = _params.ki * _params.dt;
    }
    reset();
}

int PIDController::updateState(float curr_setpoint, float curr_feedback, float * pidout)
{
    // saturation
    curr_setpoint = ( curr_setpoint > _params.in_max ? _params.in_max : (curr_setpoint < _params.in_min ? _params.in_min : curr_setpoint)); 

    // if(curr_setpoint > _params.in_max || curr_setpoint < _params.in_min)
        // return PID_ERROR_SETPOINT_LIMITS; 
    // if(curr_feedback > _params.in_max || curr_feedback < _params.in_min)
        // do{}while(0); //FIXME: Possible system failure, notify user!

    // Compute error
    float error = (curr_setpoint - curr_feedback);

    // Compute proportional and integral part
    float pidout_internal = _params.kp*error;
    
    // Compute integral part
    if (_params.ki != 0.0f)
        pidout_internal += _ki*_state.int_sum;

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
    if (_params.kd != 0.0F)
        pidout_internal += (_kd * dchange);

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

    if (_params.ki != 0.0f)
    {
        if ((_params.flags & PID_INT_SOFT_ANTI_WINDUP) && (limit_high || limit_low))
            _state.int_sum += (_params.anti_windup * ichange);
        else
            _state.int_sum += ichange;
    }

    if(_params.flags & PID_USE_QUANTUM_OUTPUT)
    {
        int tmp = (_state.last_pidout-pidout_internal)/_params.min_pid_output_step;
        *pidout = _state.last_pidout + tmp * _params.min_pid_output_step;
    }
    else
        *pidout = _state.last_pidout = pidout_internal;
    
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
    _ki = _params.ki / _params.dt;
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