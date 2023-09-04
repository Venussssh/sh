#ifndef BASICVQF_H
#define BASICVQF_H

#include "LSM6DS3.h"

#define M_PI       3.14159265358979323846   // pi
#define M_SQRT2    1.41421356237309504880   // sqrt(2)
#define EPS FLT_EPSILON                     // 极小值
#define NaN NAN                             // 非实数

// #ifndef VQF_SINGLE_PRECISION
// typedef double vqf_real_t;
// #else
typedef float vqf_real_t;
// #endif

/**
 * @brief Contains the current parameters.
 *
 * See #getParams. To set parameters, pass them to the constructor. The parameters can be changed with
 * #setTauAcc and #setTauMag.
 */
typedef struct 
{
    /* 用于加计低通滤波，Default value: 3.0 s */
    vqf_real_t tauAcc;    

}BasicVQFParams;
/**
 * @brief Contains the current state of filter.
 *
 * See #getState, #getState and #resetState.
 */
typedef struct 
{
    /* 角速度捷联积分四元数 */
    vqf_real_t gyrQuat[4];
    /* 倾斜校正四元数 */
    vqf_real_t accQuat[4];
    /* 航向角偏差 */
    vqf_real_t delta;
    /* 上一次低通滤波后在I坐标系下的加计向量 */
    vqf_real_t lastAccLp[3];
    /* lastAccLp的内部低通滤波器状态 */
    double accLpState[3*2];

}BasicVQFState;
/**
 * @brief Contains the current coefficients (calculated in #setup).
 *
 * See #getCoeffs.
 */
typedef struct 
{
    /* gyro采样时间 */
    vqf_real_t gyrTs;
    /* acc采样时间 */
    vqf_real_t accTs;
    /* 加计低通滤波器的的分子系数，b_0 & b_1 & b_2 */
    double accLpB[3]; 
    /* 加计低通滤波器的的分母系数，a_1 & a_2 */
    double accLpA[2]; 

}BasicVQFCoefficients;
/**
     * Initializes the object with default parameters.
     *
     * In the most common case (using the default parameters and all data being sampled with the same frequency,
     * create the class like this:
     * \rst
     * .. code-block:: c++
     *    BasicVQF(vqf_real_t gyrTs, vqf_real_t accTs=-1.0, vqf_real_t magTs=-1.0);
     *     BasicVQF vqf(0.01); // 0.01 s sampling time, i.e. 100 Hz
     * \endrst
     *
     * @param TauAcc sampling time of LPF
     * @param gyrTs sampling time of the gyroscope measurements in seconds
     * @param accTs sampling time of the accelerometer measurements in seconds (the value of `gyrTs` is used if set to -1)
     *
     */
void BasicVQFParamsInit(vqf_real_t TauAcc, vqf_real_t gyrTs, vqf_real_t accTs);

/**
 * @brief Performs gyroscope update step.
 *
 * It is only necessary to call this function directly if gyroscope, accelerometers and magnetometers have
 * different sampling rates. Otherwise, simply use #update().
 *
 * @param gyr gyroscope measurement in rad/s
 */
void updateGyr(vqf_real_t gyr[3]);
/**
 * @brief Performs accelerometer update step.
 *
 * It is only necessary to call this function directly if gyroscope, accelerometers and magnetometers have
 * different sampling rates. Otherwise, simply use #update().
 *
 * Should be called after #updateGyr and before #updateMag.
 *
 * @param acc accelerometer measurement in m/s²
 */
void updateAcc( vqf_real_t acc[3]);
/**
 * @brief Performs filter update step for one sample (magnetometer-free).
 * @param gyr gyroscope measurement in rad/s
 * @param acc accelerometer measurement in m/s²
 */
void update( vqf_real_t gyr[3],  vqf_real_t acc[3]);


/**
 * @brief Returns the angular velocity strapdown integration quaternion
 * \f$^{\mathcal{S}_i}_{\mathcal{I}_i}\mathbf{q}\f$.
 * @param out output array for the quaternion
 */
void getQuat3D(vqf_real_t out[4]);
/**
 * @brief Returns the 6D (magnetometer-free) orientation quaternion
 * \f$^{\mathcal{S}_i}_{\mathcal{E}_i}\mathbf{q}\f$.
 * @param out output array for the quaternion
 */
void getQuat6D(vqf_real_t out[4]);


/**
 * @brief Sets the time constant for accelerometer low-pass filtering.
 *
 * For more details, see VQFParams.tauAcc.
 *
 * @param tauAcc time constant \f$\tau_\mathrm{acc}\f$ in seconds
 */
void setTauAcc(vqf_real_t tauAcc);
/**
 * @brief Resets the state to the default values at initialization.
 *
 * Resetting the state is equivalent to creating a new instance of this class.
 */
void resetState();


// operations of quats & vectors
/**
 * @brief Performs quaternion multiplication (\f$\mathbf{q}_\mathrm{out} = \mathbf{q}_1 \otimes \mathbf{q}_2\f$).
 */
void quatMultiply( vqf_real_t q1[4],  vqf_real_t q2[4], vqf_real_t out[4]);
/**
 * @brief Sets the output quaternion to the identity quaternion (\f$\mathbf{q}_\mathrm{out} =
 * \begin{bmatrix}1 & 0 & 0 & 0\end{bmatrix}\f$).
 */
void quatSetToIdentity(vqf_real_t out[4]);
/**
 * @brief Rotates a vector with a given quaternion.
 *
 * \f$\begin{bmatrix}0 & \mathbf{v}_\mathrm{out}\end{bmatrix} =
 * \mathbf{q} \otimes \begin{bmatrix}0 & \mathbf{v}\end{bmatrix} \otimes \mathbf{q}^*\f$
 */
void quatRotate( vqf_real_t q[4],  vqf_real_t v[3], vqf_real_t out[3]);
/**
 * @brief Calculates the Euclidean norm of a vector.
 * @param vec pointer to an array of N elements
 * @param N number of elements
 */
 vqf_real_t norm( vqf_real_t vec[], size_t N);
/**
 * @brief Normalizes a vector in-place.
 * @param vec pointer to an array of N elements that will be normalized
 * @param N number of elements
 */
 void normalize(vqf_real_t vec[], size_t N);

// operations of filter
/**
 * @brief Calculates coefficients for a second-order Butterworth low-pass filter.
 *
 * The filter is parametrized via the time constant of the dampened, non-oscillating part of step response and the
 * resulting cutoff frequency is \f$f_\mathrm{c} = \frac{\sqrt{2}}{2\pi\tau}\f$.
 *
 * @param tau time constant \f$\tau\f$ in seconds
 * @param Ts sampling time \f$T_\mathrm{s}\f$ in seconds
 * @param outB output array for numerator coefficients
 * @param outA output array for denominator coefficients (without \f$a_0=1\f$)
 */
void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, double outB[], double outA[]);
/**
 * @brief Calculates the initial filter state for a given steady-state value.
 * @param x0 steady state value
 * @param b numerator coefficients
 * @param a denominator coefficients (without \f$a_0=1\f$)
 * @param out output array for filter state
 */
void filterInitialState(vqf_real_t x0,  double b[3],  double a[2], double out[]);
/**
 * @brief Adjusts the filter state when changing coefficients.
 *
 * This function assumes that the filter is currently in a steady state, i.e. the last input values and the last
 * output values are all equal. Based on this, the filter state is adjusted to new filter coefficients so that the
 * output does not jump.
 *
 * @param last_y last filter output values (array of size N)
 * @param N number of values in vector-valued signal
 * @param b_old previous numerator coefficients
 * @param a_old previous denominator coefficients (without \f$a_0=1\f$)
 * @param b_new new numerator coefficients
 * @param a_new new denominator coefficients (without \f$a_0=1\f$)
 * @param state filter state (array of size N*2, will be modified)
 */
void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N,  double b_old[],
                                               double a_old[],  double b_new[],
                                               double a_new[], double state[]);
/**
 * @brief Performs a filter step for a scalar value.
 * @param x input value
 * @param b numerator coefficients
 * @param a denominator coefficients (without \f$a_0=1\f$)
 * @param state filter state array (will be modified)
 * @return filtered value
 */
vqf_real_t filterStep(vqf_real_t x,  double b[3],  double a[2], double state[2]);
/**
 * @brief Performs filter step for vector-valued signal with averaging-based initialization.
 *
 * During the first \f$\tau\f$ seconds, the filter output is the mean of the previous samples. At \f$t=\tau\f$, the
 * initial conditions for the low-pass filter are calculated based on the current mean value and from then on,
 * regular filtering with the rational transfer function described by the coefficients b and a is performed.
 *
 * @param x input values (array of size N)
 * @param N number of values in vector-valued signal
 * @param tau filter time constant \f$\tau\f$ in seconds (used for initialization)
 * @param Ts sampling time \f$T_\mathrm{s}\f$ in seconds (used for initialization)
 * @param b numerator coefficients
 * @param a denominator coefficients (without \f$a_0=1\f$)
 * @param state filter state (array of size N*2, will be modified)
 * @param out output array for filtered values (size N)
 */
void filterVec(vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, double b[3],
                         double a[2], double state[], vqf_real_t out[]);

/**
 * @brief Calculates coefficients based on parameters and sampling rates.
 */
void setup();

void RunBasicVQF(vqf_real_t Data_Gx, vqf_real_t Data_Gy, vqf_real_t Data_Gz, vqf_real_t Data_XLx, vqf_real_t Data_XLy, vqf_real_t Data_XLz, vqf_real_t *angle);


#endif
