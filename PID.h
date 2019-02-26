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

#define PID_ERROR_SETPOINT_LIMITS   1 ///< setpoint exceeds input limit 
#define PID_ERROR_FEEDBACK_LIMITS   2 ///< feedback exceeds input limit (!!SYSTEM FAILURE!!)   
#define PID_REGULATION_END          3 ///< regulation was stopped 
#define PID_COMPUTATION_SUCCESS     0 ///< computation success - new pid output

/**
 * @brief PID parameters.
 *
 * All pid params.
 */
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

/**
 * @brief Internal pid state.
 */
typedef struct PID_state
{
    float int_sum;         ///< integral part sum
    float last_error;      ///< control error
    float last_last_error; ///< previous control error
    int mdr_spins_count;   ///< reserved
} PID_state_t;

/**
 * @brief Proportional-Integral-derivative controller.
 * 
 * PID controller with extened capabilities and simplified API.
 * It provides:
 *  * Average derivative filter
 *  * Anti-windup software reduction strategy
 *  * Integral rate limiting
 *  * Derivative glitches reduction strategy
 * @sa PID_params
 * @sa PID_state
 * @sa PID_flags
 */
class PIDController 
{
    public:

        /**
         * @brief PID flags.
         *
         * Extend pid functionality:
         * @code{.cpp}
         * PID_params_t params.flags = PID_INT_SOFT_ANTI_WINDUP | PID_INT_RATE_LIMIT;
         * @endcode
         */
        enum PID_flags
        {
            PID_INT_SOFT_ANTI_WINDUP = 0x01,    ///< enable soft anti-windup reduction strategy
            PID_INT_RATE_LIMIT = 0x02,          ///< enable integral part limitation
            PID_AVG_FILTER = 0x04,              ///< enable derivative average filter
            PID_DERIV_RESP_GLITCHES_FIX = 0x08, ///< applay fix for derivative response
            PID_END_REG_JOB_SUCCESS = 0x10      ///< reserved
        };        

        /**
         * @brief Create controller with default params.
         */
        PIDController();

        /**
         * @brief Create controller.
         * @param params pid configuration
         */
        PIDController(const PID_params_t * params);
        
        ~PIDController();
        
        /**
         * @brief Update pid configuration.
         *
         * It updates all pid parameters.
         * @param params pid configuration
         */
        void updateParams(const PID_params_t * params);
        
        /**
         * @brief Get current pid configuration.
         * @param params pid parameters to update 
         */
        void getParams(PID_params_t * params);
        
        /**
         * @brief Get current pid state.
         * 
         * @param state state to update 
         */
        void getState(PID_state_t * state);
        
        /**
         * @brief Reset pid state.
         *
         * Resets error and integral part
         */
        void reset();

        /**
         * @brief Update current pid state.
         *
         * Compute new pid output and update its internal state.
         * @param curr_setpoint current setpoint
         * @param curr_feedback current feedback
         * @param pidout new pid output
         * @return 0 on success, 1 if inputs are incorrect
         */
        int updateState(const float * curr_setpoint, const float * curr_feedback, float * pidout);
        
        /**
         * @brief Update pid computation interval.
         *
         * It also updates other parameters to provide smooth transition.
         * @param dt interval [s]
         */
        void updateInterval(float dt);
        
        /**
         * @brief Update pid tunings.
         *
         * It also update other parameters to provide smooth transition.
         * @param kp proportional gain
         * @param kd derivative gain
         * @param ki integral gain
         */
        void updateTuning(float kp, float kd, float ki);

    private:
        PID_params_t _params;
        PID_state_t _state;
};
#endif /*__PID_H__ */