#include "basicvqf.h"
#include <math.h>
#include <float.h>
#include <string.h>

// vqf_real_t square(vqf_real_t x) { return x*x; }

BasicVQFParams params;
BasicVQFState state;
BasicVQFCoefficients coeffs;

void BasicVQFParamsInit(vqf_real_t TauAcc, vqf_real_t gyrTs, vqf_real_t accTs)
{
    // params.tauAcc = 3.0;
    // params->tauMag = 9.0;
    params.tauAcc = TauAcc;

    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    
    setup();
}

void updateGyr(vqf_real_t gyr[3])
{
    // gyroscope prediction step
    vqf_real_t gyrNorm = norm(gyr, 3);
    vqf_real_t angle = gyrNorm * coeffs.gyrTs;  // integral
    if (gyrNorm > EPS) 
    {
        vqf_real_t c = cos(angle/2);
        vqf_real_t s = sin(angle/2)/gyrNorm;
        vqf_real_t gyrStepQuat[4] = {c, s*gyr[0], s*gyr[1], s*gyr[2]};
        quatMultiply(state.gyrQuat, gyrStepQuat, state.gyrQuat);    //last one is output
        normalize(state.gyrQuat, 4);
    }
}

void updateAcc( vqf_real_t acc[3])
{
    // ignore [0 0 0] samples
    if (acc[0] == 0.0 && acc[1] == 0.0 && acc[2] == 0.0) 
    {
        return;
    }

    vqf_real_t accEarth[3];

    // filter acc in inertial frame ，I frame
    quatRotate(state.gyrQuat, acc, accEarth);
    filterVec(accEarth, 3, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.accLpState, state.lastAccLp);

    // transform to 6D earth frame and normalize,E frame
    quatRotate(state.accQuat, state.lastAccLp, accEarth);
    normalize(accEarth, 3);

    // inclination correction
    vqf_real_t accCorrQuat[4];
    vqf_real_t q_w = sqrt((accEarth[2]+1)/2);
    if (q_w > 1e-6) 
    {
        accCorrQuat[0] = q_w;
        accCorrQuat[1] = 0.5*accEarth[1]/q_w;
        accCorrQuat[2] = -0.5*accEarth[0]/q_w;
        accCorrQuat[3] = 0;
    } 
    else 
    {
        // to avoid numeric issues when acc is close to [0 0 -1], i.e. the correction step is close (<= 0.00011°) to 180°:
        accCorrQuat[0] = 0;
        accCorrQuat[1] = 1;
        accCorrQuat[2] = 0;
        accCorrQuat[3] = 0;
    }
    quatMultiply(accCorrQuat, state.accQuat, state.accQuat);
    normalize(state.accQuat, 4);
}

void update( vqf_real_t gyr[3],  vqf_real_t acc[3])
{
    updateGyr(gyr);
    updateAcc(acc);
}

void getQuat3D(vqf_real_t out[4])
{
    memcpy(out,state.gyrQuat,4);
    // std::copy(state.gyrQuat, state.gyrQuat+4, out); //memcopy
}

void getQuat6D(vqf_real_t out[4])
{
    quatMultiply(state.accQuat, state.gyrQuat, out);
}

void setTauAcc(vqf_real_t tauAcc)
{
    if (params.tauAcc == tauAcc) 
    {
        return;
    }
    params.tauAcc = tauAcc;
    double newB[3];
    double newA[3];

    filterCoeffs(params.tauAcc, coeffs.accTs, newB, newA);
    filterAdaptStateForCoeffChange(state.lastAccLp, 3, coeffs.accLpB, coeffs.accLpA, newB, newA, state.accLpState);

    memcpy(coeffs.accLpB,newB,3);
    memcpy(coeffs.accLpA,newA,2);
    // std::copy(newB, newB+3, coeffs.accLpB);
    // std::copy(newA, newA+2, coeffs.accLpA);
}

void resetState()
{
    quatSetToIdentity(state.gyrQuat); /* 初始化四元数为单位数 */
    quatSetToIdentity(state.accQuat);
    state.delta = 0.0;

    memset(state.lastAccLp,0,sizeof(state.lastAccLp));
    for (int8_t i = 0; i < 6; i++) //sizeof(state.accLpState) / sizeof(state.accLpState[0]
    {
        state.accLpState[i] = NAN;
    }
    // memset(state.accLpState,NAN,sizeof(state.accLpState)); /* memset中使用int型，NAN是float型 */
    // std::fill(state.lastAccLp, state.lastAccLp+3, 0);
    // std::fill(state.accLpState, state.accLpState + 3*2, NaN);

}

// operations of quats & vectors
void quatMultiply( vqf_real_t q1[4],  vqf_real_t q2[4], vqf_real_t out[4])
{
    vqf_real_t w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    vqf_real_t x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    vqf_real_t y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    vqf_real_t z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void quatSetToIdentity(vqf_real_t out[4])
{
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}

void quatRotate( vqf_real_t q[4],  vqf_real_t v[3], vqf_real_t out[3])
{
    vqf_real_t x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
    vqf_real_t y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
    vqf_real_t z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}

vqf_real_t norm( vqf_real_t vec[], size_t N)
{
    vqf_real_t s = 0;
    for(size_t i = 0; i < N; i++) 
    {
        s += vec[i]*vec[i];
    }
    return sqrt(s);
}

void normalize(vqf_real_t vec[], size_t N)
{
    vqf_real_t n = norm(vec, N);
    if (n < EPS) 
    {
        return;
    }
    for(size_t i = 0; i < N; i++) 
    {
        vec[i] /= n;
    }
}

// operations of filter
/* second-order LPF */
void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, double outB[], double outA[])
{
    // assert(tau > 0);
    // assert(Ts > 0);
    // second order Butterworth filter
    double fc = (M_SQRT2 / (2.0*M_PI))/(double)tau; // time constant of dampened, non-oscillating part of step response
    double C = tan(M_PI*fc*(double)Ts);
    double D = C*C + sqrt(2)*C + 1;
    double b0 = C*C/D;
    outB[0] = b0;
    outB[1] = 2*b0;
    outB[2] = b0;
    // a0 = 1.0
    outA[0] = 2*(C*C-1)/D; // a1
    outA[1] = (1-sqrt(2)*C+C*C)/D; // a2
}

void filterInitialState(vqf_real_t x0,  double b[3],  double a[2], double out[])
{
    // initial state for steady state (equivalent to scipy.signal.lfilter_zi, obtained by setting y=x=x0 in the filter
    // update equation)
    out[0] = x0*(1 - b[0]);
    out[1] = x0*(b[2] - a[1]);
}

void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N,  double b_old[],
                                               double a_old[],  double b_new[],
                                               double a_new[], double state[])
{
    if (isnan(state[0])) 
    {
        return;
    }
    for (size_t i = 0; i < N; i++) 
    {
        state[0+2*i] = state[0+2*i] + (b_old[0] - b_new[0])*last_y[i];
        state[1+2*i] = state[1+2*i] + (b_old[1] - b_new[1] - a_old[0] + a_new[0])*last_y[i];
    }
}

vqf_real_t filterStep(vqf_real_t x,  double b[3],  double a[2], double state[2])
{
    // difference equations based on scipy.signal.lfilter documentation
    // assumes that a0 == 1.0
    double y = b[0]*x + state[0];
    state[0] = b[1]*x - a[0]*y + state[1];
    state[1] = b[2]*x - a[1]*y;
    return y;
}

void filterVec(vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, double b[3],
                         double a[2], double state[], vqf_real_t out[])
{
    // assert(N>=2);

    // to avoid depending on a single sample, average the first samples (for duration tau)
    // and then use this average to calculate the filter initial state
    if (isnan(state[0])) 
    { // initialization phase
        if (isnan(state[1])) // first sample
        { 
            state[1] = 0; // state[1] is used to store the sample count
            for(size_t i = 0; i < N; i++) 
            {
                state[2+i] = 0; // state[2+i] is used to store the sum
            }
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) 
        {
            state[2+i] += x[i];
            out[i] = state[2+i]/state[1];
        }
        if (state[1]*Ts >= tau) 
        {
            for(size_t i = 0; i < N; i++) 
            {
               filterInitialState(out[i], b, a, state+2*i);
            }
        }
        return;
    }

    for (size_t i = 0; i < N; i++) 
    {
        out[i] = filterStep(x[i], b, a, state+2*i);
    }
}

void setup()
{
    // assert(coeffs.gyrTs > 0);  // 判断采样时间是否大于零

    // 启动低通滤波器
    filterCoeffs(params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA);
    // 重置参数
    resetState();
}

void RunBasicVQF(vqf_real_t Data_Gx, vqf_real_t Data_Gy, vqf_real_t Data_Gz, vqf_real_t Data_XLx, vqf_real_t Data_XLy, vqf_real_t Data_XLz, vqf_real_t *angle)
{
  
    vqf_real_t gyr[3] = {Data_Gx, Data_Gy, Data_Gz};
    vqf_real_t acc[3] = {Data_XLx, Data_XLy, Data_XLz};
    vqf_real_t q[4] = {1, 0, 0, 0};
    vqf_real_t pitch;
    vqf_real_t roll;
    vqf_real_t yaw;
    BasicVQFParamsInit(2.5, 0.002, -1);
    update(gyr, acc);
    getQuat6D(q);

    pitch = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / M_PI;
    roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 180 / M_PI;
    yaw   = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) * 180 / M_PI;
    angle[0] = pitch;
    angle[1] = roll;
    angle[2] = yaw;
}