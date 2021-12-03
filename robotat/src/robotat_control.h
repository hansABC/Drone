/**
 * @file robotat_control.h
 * @author Miguel Zea (mezea@uvg.edu.gt)
 * @brief 
 * @version 0.1
 * @date 2021-08-12
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef ROBOTAT_CONTROL_H_
#define ROBOTAT_CONTROL_H_

 /**
  * Dependencies. 
  */
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <stdarg.h>
#include "robotat_linalg.h"

// ====================================================================================================
// Data structures, enums and type definitions
// ====================================================================================================
// NOTE: these should not be manipulated directly, use the init, setter and getter routines instead.

/**
 * @brief   Discretization specification data type.
 * 
 * Used to specify the numerical integration scheme to use when discretizing LTI systems and controllers.
 */
typedef enum
{
    PURE_DISCRETE,  /**< Sampling period independent. */
    FWD_EULER,      /**< Forward Euler integration. */
    BWD_EULER,      /**< Backward Euler integration. */
    TUSTIN,         /**< Trapezoidal rule. */
    ZOH             /**< Zero-order hold. */
} discretization_spec_t;


/**
 * @brief   PID controller data structure.
 */
typedef struct
{
    float kp;       /**< Proportional gain. */
    float ki;       /**< Integral gain. */
    float kd;       /**< Derivative gain. */
    float e_k_1;    /**< Last error. */
    float u_k_1;    /**< Last controller output. */
    float i_min;    /**< Lower integrator saturation threshold. */
    float i_max;    /**< Upper integrator saturation threshold. */
    float tau;      /**< Time constant of the derivative HPF. */
    float dt;       /**< Sampling period. */
    discretization_spec_t pid_alg;  /**< Specifies the discretization scheme to be used. */
} pid_info_t;


/**
 * @brief   State space LTI system data structure.
 */
typedef struct
{
    matf32_t* state;        /**< State at the current time step (used for simulation/integration). */
    uint16_t state_dim;     /**< Number of state variables. This is redundant but we'll keep it for completeness. */
    uint16_t input_dim;     /**< Number of inputs/actuators/controls. */
    uint16_t output_dim;    /**< Number of outputs/measurements. */
    matf32_t* A;            /**< System matrix. */
    matf32_t* B;            /**< Input/actuator matrix. */
    matf32_t* C;            /**< Output/sensor matrix. */
    matf32_t* D;            /**< Feedforward terms. */
    float dt;               /**< Sampling period (for discrete time systems). */
    bool is_continuous;     /**< System time domain specification. */
} sys_lti_t;


/**
 * @brief   State space nonlinear system data structure.
 */
typedef struct
{
    matf32_t* state;        /**< State at the current time step (used for simulation/integration). */
    uint16_t state_dim;     /**< Number of state variables. This is redundant but we'll keep it for completeness. */
    uint16_t input_dim;     /**< Number of inputs/actuators/controls. */
    uint16_t output_dim;    /**< Number of outputs/measurements. */
    err_status_t (*dynamics)(matf32_t*, const matf32_t*, const matf32_t*);      /** System dynamics. */
    err_status_t (*outputs)(matf32_t*, const matf32_t*, const matf32_t*);       /** System outputs. */
    float dt;               /**< Sampling period (for discrete time systems). */
    bool is_continuous;     /**< System time domain specification. */
} sys_nonlin_t;


/**
 * @brief   Linear time-varying Kalman filter data structure.
 */
typedef struct
{
    sys_lti_t* sys;     /**< LTI system model(has to be discrete time). */
    matf32_t* F;        /**< Coupling matrix for the process noise. */
    matf32_t* Qw;       /**< Process noise covariance matrix. */
    matf32_t* Qv;       /**< Measurement noise covariance matrix. */
    matf32_t* xhat;     /**< State estimate. */
    matf32_t* P;        /**< Estimation covariance matrix. */
} kalman_info_t;


// ====================================================================================================
// Public function prototypes
// ====================================================================================================
// PID Control
// ====================================================================================================
/**
 * @brief   Initializes a PID controller structure.
 * 
 * The controller transfer function (except for the PURE_DISCRETE case) is given by:
 * C(s) = kP + kI/s + kD * tau*s / (s + tau).
 * Depending on the discretization scheme and whether or not the integrator saturates, the function 
 * can ask for additional parameters:
 * 
 * 1) pid_init(pid, kp, ki, kd, PURE_DISCRETE, 0); 
 * 2) pid_init(pid, kp, ki, kd, PURE_DISCRETE, 1, i_min, i_max); // With saturation limits.
 * 3) pid_init(pid, kp, ki, kd, pid_alg, 0, dt, tau); // Needs sampling period and time constant.
 * 4) pid_init(pid, kp, ki, kd, pid_alg, 1, dt, tau, i_min, i_max); // Needs all info. 
 * 
 * @param[in, out]  pid             PID controller data structure.
 * @param[in]       kp              Proportional gain.
 * @param[in]       ki              Integral gain.
 * @param[in]       kd              Derivative gain.
 * @param[in]       pid_alg         PID discretization scheme specification.
 * @param[in]       set_i_limits    Allows to set lower and upper saturation thresholds for the integrator.    
 * @param[in]       dt              Sampling period.
 * @param[in]       tau             Derivative HPF time constant.
 * @param[in]       i_min           Lower saturation threshold.
 * @param[in]       i_max           Upper saturation threshold.
 * 
 * @return  None.
 */
void
pid_init(pid_info_t* const pid, float kp, float ki, float kd, discretization_spec_t pid_alg, bool set_i_limits, ...);


/**
 * @brief   Sets new gains for the PID controller.
 * 
 * @param[in, out]  pid     PID controller data structure.
 * @param[in]       kp      New proportional gain.
 * @param[in]       ki      New integral gain.
 * @param[in]       kd      New derivative gain.
 * 
 * @return  None.
 */
static inline void
pid_set_gains(pid_info_t* const pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}


/**
 * @brief   Updates a previously initialized PID controller.
 * 
 * WARNING: this routine does NOT check whether or not the controller was previously initialized. It also
 * does NOT take measurement compensation into consideration.
 * 
 * @param[in]   pid     PID controller data structure to update.
 * @param[in]   r_k     Reference signal at the current time step.
 * @param[in]   y_k     Measurement (after compensation) at current time step.
 * 
 * @return  Controller output.
 */
float
pid_update(pid_info_t* const pid, float r_k, float y_k);


// ====================================================================================================
// State space representation
// ====================================================================================================
/**
 * @brief   Initializes a state space LTI model.
 * 
 * WARNING: this routine is written to provide somewhat of a MATLAB compatibility but does NOT initialize
 * the complete data structure. The state vector will be initialized until it's needed.
 * 
 * @param[in]       A               System matrix.
 * @param[in]       B               System input/actuator matrix.
 * @param[in]       C               System output/sensor matrix.
 * @param[in]       D               System feedforward terms.
 * @param[in]       sample_time     Sampling period, in case of a discrete time system.
 * @param[in, out]  sys             LTI state space system data structure.
 * 
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
ss(matf32_t* A, matf32_t* B, matf32_t* C, matf32_t* D, float sample_time, sys_lti_t* const sys);


/**
 * @brief   Discretizes a continuous time LTI system model.
 *
 * WARNING: this routine overwrites the original continuous time system. This also does NOT
 * work for discrete time systems.
 *
 * @param[in, out]  sys             Continuous time LTI system data structure.
 * @param[in]       sample_time     Sampling period.
 * @param[in]       method          Discretization scheme.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed. 
 */
err_status_t
c2d(sys_lti_t* const sys, float sample_time, discretization_spec_t method);


// ====================================================================================================
// Linear state space controllers
// ====================================================================================================
/**
 * @brief   Updates/computes the linear state feedback controller u = -K * (x - xss) + uss.
 *
 * @param[in, out]  u       Controller output.
 * @param[in]       K       Gain matrix.
 * @param[in]       x       State vector.
 * @param[in]       xss     Operation point (desired state).
 * @param[in]       uss     Feedforward input to reach the desired state.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
linear_state_feedback(matf32_t* const u, const matf32_t* K, const matf32_t* x, const matf32_t* xss, const matf32_t* uss);


// ====================================================================================================
// Linear time-varying, discrete time Kalman filter
// ====================================================================================================
/**
 * @brief   Initializes a linear, time-varying Kalman filter structure.
 * 
 * @param[in, out]  kf      Kalman filter data structure.
 * @param[in]       sys     LTI system model.
 * @param[in]       F       Coupling matrix of the process noise.
 * @param[in]       Qw      Process noise covariance matrix.
 * @param[in]       Qv      Measurement noise covariance matrix.
 * @param[in]       xhat    State estimate.
 * @param[in]       P       Estimation covariance matrix.
 * 
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 *              MATH_ARGUMENT_ERROR :   LTI system model is not discrete time.
 */
err_status_t
kalman_init(kalman_info_t* const kf, sys_lti_t* const sys, matf32_t* F, matf32_t* Qw, matf32_t* Qv, matf32_t* const xhat, matf32_t* const P);


err_status_t
kalman_predict(kalman_info_t* const kf, const matf32_t* inputs);


err_status_t
kalman_correct(kalman_info_t* const kf, const matf32_t* measurements);


static inline err_status_t
kalman_update(kalman_info_t* const kf, const matf32_t* inputs, const matf32_t* measurements)
{
    kalman_predict(kf, inputs);
    return kalman_correct(kf, measurements);
}


static inline void
kalman_get_estimate(kalman_info_t* const kf, float* const estimate)
{
    memcpy(estimate, kf->xhat->p_data, kf->xhat->num_rows * sizeof(float));
}


// TODO:
// 1. Nonlinear system definition
// 2. Nonlinear system linearization
// 3. Nonlinear system discretization
// 4. Extended Kalman Filter
// 5. Linear time-varying LQR
// 6. Linear MPC


//void
//kalman_predict(kalman_info_t* const kf, float* const inputs);
//err_status_t
//kalman_correct(kalman_info_t* const kf, float* const measurements);

#endif /* ROBOTAT_CONTROL_H_ */
