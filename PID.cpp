/* PID.cpp
 * Implementation based on: http://www.mstarlabs.com/apeng/techniques/pidsoftw.html
 * Copyright (C) 2019 Szymon Szantula
 *
 * Distributed under the MIT license.
 * For full terms see the file LICENSE.md.
 */
#include "PID.h"

static const PID_params_t default_params = {
    .flags = 0,
    .ki = 0,
    .kp = 1,
    .kd = 0,
    .scale = 1,
    .del_kp = 1,
    .anti_windup = 0,
    .int_rate_limit_high = 0, 
    .int_rate_limit_low = 0,
    .pidout_limit_high = 0,
    .pidout_limit_low = 0,
    .error_tolerance = 0
};

PIDController::PIDController()
    : _params(default_params), _state()
{}

PIDController::~PIDController(){}

PIDController::PIDController(const PID_params & params)
: _params(params), _state()
{}

int PIDController::updateState(const float * curr_setpoint, const float * curr_feedback, float * pidout, uint32_t dt)
{
    float curr_error = *curr_setpoint - *curr_feedback;
    
    if (curr_error >= 0.0f && (_params.flags & PID_BIAS_BALANCE))
        curr_error *= _params.del_kp;
    else
        curr_error /= _params.del_kp;

    if( (_params.flags & PID_ERROR_TOLERANCE) && (curr_error <= _params.error_tolerance) && (curr_error >= -_params.error_tolerance))
    {
        // _state.reset();
        return 0;
    }

    _state.pidout=0.0f;

    // proportional part
    _state.pidout += (_params.scale * _params.kp*curr_error);

    // integral part
    _state.pidout += (_params.scale * _params.ki*_state.int_sum);

    // derivative part
    if(_state.last_setpoint == *curr_setpoint)
    {
        if(_params.flags & PID_AVG_FILTER)
            _state.pidout += _params.scale * _params.kd * (curr_error-_state.last_last_error)/(2*dt);
        else
            _state.pidout += _params.scale * _params.kd * (curr_error-_state.last_error)/dt;
    }
    else
    {
        _state.last_setpoint = *curr_setpoint;
    }
        _state.last_last_error = _state.last_error; 
        _state.last_error = curr_error;

    float ichange = curr_error;
    if (_params.flags & PID_INT_RATE_LIMIT)
    {
        if (ichange > _params.int_rate_limit_high)
            ichange = _params.int_rate_limit_high;
        else if (ichange < _params.int_rate_limit_low)
            ichange = _params.int_rate_limit_low;
    }


    if (_params.flags & PID_OUTPUT_LIMITS)
    {
        bool limit_high = (_state.pidout >= _params.pidout_limit_high);
        bool limit_low = (_state.pidout <= _params.pidout_limit_low);
        _state.pidout = (limit_high ? _params.pidout_limit_high : (limit_low ? _params.pidout_limit_low : _state.pidout));
        
        if ((_params.flags & PID_INT_SOFT_ANTI_WINDUP) && (limit_high || limit_low))
            _state.int_sum += _params.anti_windup * ichange * dt;
        else
            _state.int_sum += ichange * dt;
    }
    *pidout = _state.pidout;
    return 1;
}