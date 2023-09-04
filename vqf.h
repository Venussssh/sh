#ifndef __VQF_H
#define __VQF_H

#include <stddef.h>
#include <stdbool.h>

#ifndef VQF_SINGLE_PRECISION
typedef double vqf_real_t;
#else
typedef float vqf_real_t;
#endif


//cpp中math.h
#define M_TWOPI         (M_PI * 2.0)
#define M_3PI_4		2.3561944901923448370E0
#define M_SQRTPI        1.77245385090551602792981
#define M_LN2LO         1.9082149292705877000E-10
#define M_LN2HI         6.9314718036912381649E-1
#define M_SQRT3	1.73205080756887719000
#define M_IVLN10        0.43429448190325182765 /* 1 / log(10) */
#define M_LOG2_E        _M_LN2
#define M_INVLN2        1.4426950408889633870E0  /* 1 / log(2) */
#define MAXFLOAT	3.40282347e+38F
#define M_E		2.7182818284590452354
#define M_LOG2E		1.4426950408889634074
#define M_LOG10E	0.43429448190325182765
#define M_LN2		_M_LN2
#define M_LN10		2.30258509299404568402
#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308
#define M_2_SQRTPI	1.12837916709551257390
#define M_SQRT2		1.41421356237309504880
#define M_SQRT1_2	0.70710678118654752440






typedef struct
{
	vqf_real_t tauAcc;
	vqf_real_t tauMag;
	#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    //运动阶段启用陀螺仪偏差
    bool motionBiasEstEnabled;
    #endif
    //休息阶段启用陀螺仪偏差估计
    bool restBiasEstEnabled;//true
    //磁干扰，舍弃
    bool magDistRejectionEnabled;//no true
    //初始偏差估计的不确定性偏差？
    vqf_real_t biasSigmaInit;
    //偏置估计不确定度
    vqf_real_t biasForgettingTime;
    //陀螺仪最大预期偏差
    vqf_real_t biasClip;
    #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    //运动过程中收敛偏差估计不确定性的标准差
    vqf_real_t biasSigmaMotion;
    //偏差因子
    vqf_real_t biasVerticalForgettingFactor;
    #endif
    //标准差
    vqf_real_t biasSigmaRest;
    //休息检测时间阈值
    vqf_real_t restMinT;
    //用于休息检测的低通滤波器常数
    vqf_real_t restFilterTau;
    //精致检测的角速度阈值
    vqf_real_t restThGyr;
    //静止检测的加速度阈值
    vqf_real_t restThAcc;
    //磁干扰
    vqf_real_t magCurrentTau;
    //调整磁场基准的时间常数
    vqf_real_t magRefTau;//mag
    vqf_real_t magNormTh;//mag
    vqf_real_t magDipTh;//mag
    vqf_real_t magNewTime;//mag
    vqf_real_t magNewFirstTime;//mag
    vqf_real_t magNewMinGyr;//mag
    vqf_real_t magMinUndisturbedTime;
    vqf_real_t magMaxRejectionTime;
    vqf_real_t magRejectionFactor;
}LSM6DS3_VQFTypeDef;



typedef struct
{
    vqf_real_t gyrTs;
    vqf_real_t accTs;
    vqf_real_t magTs;
    double accLpB[3];
    double accLpA[2];
    vqf_real_t kMag;
    vqf_real_t biasP0;
    vqf_real_t biasV;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasMotionW;
    vqf_real_t biasVerticalW;
#endif
    vqf_real_t biasRestW;
    double restGyrLpB[3];
    double restGyrLpA[2];
    double restAccLpB[3];
    double restAccLpA[2];
    vqf_real_t kMagRef;
    double magNormDipLpB[3];
    double magNormDipLpA[2];
}VQFCoefficients;


typedef struct
{
    vqf_real_t gyrQuat[4];
    vqf_real_t accQuat[4];
    vqf_real_t delta;
    bool restDetected;
    bool magDistDetected;//mag
    vqf_real_t lastAccLp[3];
    double accLpState[3*2];
    vqf_real_t lastAccCorrAngularRate;
    vqf_real_t kMagInit;
    vqf_real_t lastMagDisAngle;//mag
    vqf_real_t lastMagCorrAngularRate;//mag
    vqf_real_t bias[3];

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasP[9];
#else
    vqf_real_t biasP;
#endif

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    double motionBiasEstRLpState[9*2];
    double motionBiasEstBiasLpState[2*2];
#endif
    vqf_real_t restLastSquaredDeviations[2];
    vqf_real_t restT;
    vqf_real_t restLastGyrLp[3];
    double restGyrLpState[3*2];
    vqf_real_t restLastAccLp[3];
    double restAccLpState[3*2];
    vqf_real_t magRefNorm;
    vqf_real_t magRefDip;
    vqf_real_t magUndisturbedT;
    vqf_real_t magRejectT;
    vqf_real_t magCandidateNorm;
    vqf_real_t magCandidateDip;
    vqf_real_t magCandidateT;
    vqf_real_t magNormDip[2];
    double magNormDipLpState[2*2];
}VQFState;

void quatSetToIdentity(vqf_real_t out[4]);
vqf_real_t square(vqf_real_t x);
void setMagRef(vqf_real_t norm, vqf_real_t dip);
vqf_real_t getMagRefDip();
vqf_real_t getMagRefNorm();
void getRelativeRestDeviations(vqf_real_t out[2]);
bool getMagDistDetected();
bool getRestDetected();
void setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma);
vqf_real_t getBiasEstimate(vqf_real_t out[3]);
vqf_real_t getDelta();

void setMotionBiasEstEnabled(bool enabled);
void setRestBiasEstEnabled(bool enabled);
void setMagDistRejectionEnabled(bool enabled);
void setTauAcc(vqf_real_t tauAcc);
void setTauMag(vqf_real_t tauMag);
void setRestDetectionThresholds(vqf_real_t thGyr, vqf_real_t thAcc);
double min(vqf_real_t a, vqf_real_t b);
void getQuat6D(vqf_real_t out[4]);
void updateGyr(vqf_real_t gyr[3]);
void updateAcc(vqf_real_t acc[3]);
void std_fill_any(double array[], int ARRAY_SIZE, vqf_real_t any);
void std_fill(double array[], int ARRAY_SIZE);
void std_fill_zero(double array[], int ARRAY_SIZE);
void std_fill_doublezero(double array[], int ARRAY_SIZE);
double std_max(vqf_real_t a, vqf_real_t b);
void std_copy(vqf_real_t Copy_array[], int ARRAY_SIZE, vqf_real_t Result_array[]);

void VQFParams_Init(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);
void setup();
void update(vqf_real_t gyr[3], vqf_real_t acc[3]);
void resetState();
void quatMultiply(vqf_real_t q1[4], vqf_real_t q2[4], vqf_real_t out[4]);
void quatConj(vqf_real_t q[4], vqf_real_t out[4]);
void quatSetToIdentity(vqf_real_t out[4]);
void quatApplyDelta(vqf_real_t q[], vqf_real_t delta, vqf_real_t out[]);
void quatRotate(vqf_real_t q[4], vqf_real_t v[3], vqf_real_t out[3]);
vqf_real_t norm(vqf_real_t vec[], int N);
void normalize(vqf_real_t vec[], int N);
void clip(vqf_real_t vec[], int N, vqf_real_t min, vqf_real_t max);
vqf_real_t gainFromTau(vqf_real_t tau, vqf_real_t Ts);
void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, double outB[], double outA[]);
void filterInitialState(vqf_real_t x0, double b[3], double a[2], double out[]);
vqf_real_t filterStep(vqf_real_t x, double b[3], double a[2], double state[2]);
void getQuat6D(vqf_real_t out[4]);
void matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9]);
void matrix3Multiply(vqf_real_t in1[9], vqf_real_t in2[9], vqf_real_t out[9]);
void matrix3MultiplyTpsFirst(vqf_real_t in1[9], vqf_real_t in2[9], vqf_real_t out[9]);
void matrix3MultiplyTpsSecond(vqf_real_t in1[9], vqf_real_t in2[9], vqf_real_t out[9]);
bool matrix3Inv(vqf_real_t in[9], vqf_real_t out[9]);
void RunVQF(vqf_real_t Data_Gx, vqf_real_t Data_Gy, vqf_real_t Data_Gz, vqf_real_t Data_XLx, vqf_real_t Data_XLy, vqf_real_t Data_XLz, vqf_real_t *angle);


void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, double b_old[],
                                         double a_old[], double b_new[],
                                         double a_new[], double state[]);
void filterVec(vqf_real_t x[], int N, vqf_real_t tau, vqf_real_t Ts, double b[3],
                    double a[2], double state[], vqf_real_t out[]);

#endif 
