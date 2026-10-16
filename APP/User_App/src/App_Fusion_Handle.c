/**
 * @file    App_Fusion_Handle.c
 * @brief   The 7-state attitude filter: the startup reference, and the
 *          per-sample step that consumes one sensor sample.
 * @author  D0cC
 * @date    2026-09-30
 * @version V0.4.0
 *
 * @note    See App_Fusion_Handle.h for the frames, the state, the noise model
 *          and what each public function promises. What is here is the
 *          arithmetic and the order it runs in.
 *
 *          The console side is no longer here: the queue of pending messages
 *          and the lines printed for them moved out unchanged into
 *          App_FusionLog_Handle.c, which this file calls when it has something
 *          to report. Raising a line is a store, which is what lets the step do
 *          it from the tick interrupt without printing there.
 *
 *          Nothing in this file reads a register or a raw count. The sensor
 *          module has already turned the parts into milli-g, milli-degrees per
 *          second and milli-gauss, and the only thing left to decide is which
 *          body axis each of those belongs to -- which is what the alignment
 *          matrices are for, and why they are applied at the one place a sensor
 *          vector enters this module.
 */

#include <math.h>

#include "App_Fusion_Handle.h"
#include "App_FusionLog_Handle.h"
#include "App_Sensor_Handle.h"
#include "main.h"                   /* provides extern volatile uint32_t TimeBase */

/* Internal types. The public struct spells the four quaternion numbers out as
 * Qw..Qz because that is what a consumer prints; in here they are a small
 * struct with the real part first, which is the order every form of the update
 * expects, and small enough that the compiler keeps one in registers. */
typedef struct	// 三维向量
{
    float X, Y, Z;
} Fus_Vec3_t;

typedef struct		// 四元数 (W为实部，X/Y/Z为虚部)
{
    float W, X, Y, Z;
} Fus_Quat_t;


#define FUS_NORM_MIN                    (1.0e-6f)
#define FUS_STATE_DIM                   (6)													 // 状态维数：3维姿态误差 + 3维零偏误差 (ESKF核心)
#define FUS_BLOCK_DIM                   (3)													 // 观测块维数：3轴

#define FUS_OBS_DIM_MAX                 (2 * FUS_BLOCK_DIM)

static const float Fus_AlignAccel[9] = APP_FUSION_ALIGN_ACCEL;
static const float Fus_AlignGyro[9]  = APP_FUSION_ALIGN_GYRO;
static const float Fus_AlignMag[9]   = APP_FUSION_ALIGN_MAG;

static const Fus_Vec3_t Fus_NavUp = { 0.0f, 0.0f, -1.0f };

static App_Fusion_Attitude_t Fus_Att;

static Fus_Quat_t Fus_Q;																													//四元数（当前姿态）
static Fus_Vec3_t Fus_GyroBiasRads;																								//陀螺零偏（当前零偏）
static Fus_Vec3_t Fus_MagRefN;																										// 导航系下的磁场参考向量（启动时标定）

static float    Fus_SumGyro[3];     /* mdps, body frame                        */
static float    Fus_SumAccel[3];    /* mg, body frame                          */
static float    Fus_SumMag[3];      /* mG, body frame                          */
static uint32_t Fus_CountGyro;
static uint32_t Fus_CountAccel;
static uint32_t Fus_CountMag;

static uint32_t Fus_WindowTick;
static uint32_t Fus_WindowStartTick;
static uint8_t  Fus_StallLogged;

static uint32_t Fus_SampleTickUs;

static float Fus_P[FUS_STATE_DIM * FUS_STATE_DIM];																// 6x6 误差协方差矩阵


static uint8_t Fus_CholFailLogged;

static void Fus_ApplyAlign(const float *pMat, const int32_t *pIn,								// 传感器轴系对齐：将传感器原始数据乘以 3x3 对齐矩阵
                           Fus_Vec3_t *pOut);
static float Fus_VecMag(const Fus_Vec3_t *pVec);
static float Fus_ClampUnit(float Value);
static void Fus_QuatNorm(Fus_Quat_t *pQ);																				// 四元数归一化：防止四元数偏离单位球，如果模长过小则重置为单位四元数
static void Fus_QuatFromEuler(float Roll, float Pitch, float Yaw,
                              Fus_Quat_t *pQ);
static void Fus_EulerFromQuat(const Fus_Quat_t *pQ, float *pRoll,
                              float *pPitch, float *pYaw);
static void Fus_QuatVec(const Fus_Quat_t *pQ, const Fus_Vec3_t *pVec,						
                        Fus_Vec3_t *pOut);
static void Fus_QuatMul(const Fus_Quat_t *pA, const Fus_Quat_t *pB,
                        Fus_Quat_t *pOut);
static void Fus_QuatVecT(const Fus_Quat_t *pQ, const Fus_Vec3_t *pVec,					//由四元数旋转向量：将导航系向量旋转到机体系（观测方程预测用）
                         Fus_Vec3_t *pOut);
static void Fus_RotQuat(const Fus_Quat_t *pQ, const Fus_Vec3_t *pVec,						//误差向量转增量四元数，右乘修正姿态 核心公式：dq = [cos(|dθ|/2), (dθ/|dθ|)sin(|dθ|/2)]，然后 Q_new = Q * dq
                        Fus_Quat_t *pOut);
static void Fus_MatMul(const float *pA, const float *pB, float *pC,							//通用矩阵乘法 C = A * B
                       int RowsA, int Shared, int ColsB);
static void Fus_MatMulT(const float *pA, const float *pB, float *pC,						//矩阵乘法（第二因子转置）C = A * B^T
                        int RowsA, int Shared, int RowsB);
static void Fus_BuildPhi(const Fus_Vec3_t *pRate, float Dt, float *pPhi);				//构建 6x6 误差状态转移矩阵 Phi = I + F*dt 状态顺序: [姿态误差(dθx, dθy, dθz), 零偏误差(dbx, dby, dbz)]
static int  Fus_CholFactor(const float *pA, float *pL, int Dim);								//Cholesky 分解：A = L * L^T，要求 A 对称正定
static void Fus_CholSolve(const float *pL, const float *pB, float *pX, int Dim);//利用 Cholesky 因子解线性方程组 A*x = b 分两步：前向代入解 L*y = b，再后向代入解 L^T*x = y
static int  Fus_IsMoving(const Fus_Vec3_t *pGyroMdps);
static void Fus_WindowRestart(uint32_t Now);
static void Fus_ReferenceTake(uint32_t Now);
static void Fus_WindowFeed(const App_Sensor_Sample_t *pSample, uint32_t Now);
static void Fus_Track(const App_Sensor_Sample_t *pSample);
static void Fus_Predict(const App_Sensor_Sample_t *pSample, float Dt);					//使用陀螺仪数据预测姿态和协方差
static void Fus_RowsFill(const Fus_Quat_t *pQ, const Fus_Vec3_t *pMeas,					//填充观测矩阵 H 和残差 Z		
                         const Fus_Vec3_t *pRefN, int Row, float *pH, float *pZ);			
static int  Fus_Update(const float *pH, const float *pR, const float *pZ,				// 应用观测修正，更新状态和协方差
                       int Dim);			
static void Fus_Correct(const App_Sensor_Sample_t *pSample);
static void Fus_Output(void);

/**
 * @name    Fus_ApplyAlign
 * @brief   Turn a sensor vector into the body frame.
 * @param   pMat    The device's 3x3 alignment, row-major, nine floats.
 * @param   pIn     Three raw counts, the sensor's own integer unit.
 * @param   pOut    The same vector in the body frame, in the same unit.
 * @retval  None
 * @note    Row-major with the body X row first, which is the order the matrix
 *          macros in the header are written in. The unit is carried through
 *          untouched: this is a rotation, and the conversion to g or rad/s
 *          happens at the one place each vector is used as a number rather than
 *          as a direction.
 */
static void Fus_ApplyAlign(const float *pMat, const int32_t *pIn,
                           Fus_Vec3_t *pOut)
{
    pOut->X = (pMat[0] * (float)pIn[0]) + (pMat[1] * (float)pIn[1]) +
              (pMat[2] * (float)pIn[2]);
    pOut->Y = (pMat[3] * (float)pIn[0]) + (pMat[4] * (float)pIn[1]) +
              (pMat[5] * (float)pIn[2]);
    pOut->Z = (pMat[6] * (float)pIn[0]) + (pMat[7] * (float)pIn[1]) +
              (pMat[8] * (float)pIn[2]);
}

/**
 * @name    Fus_VecMag
 * @brief   The length of a vector.
 * @param   pVec    The vector.
 * @retval  Its length, in whatever unit the components are in.
 */
static float Fus_VecMag(const Fus_Vec3_t *pVec)
{
    return sqrtf((pVec->X * pVec->X) + (pVec->Y * pVec->Y) +
                 (pVec->Z * pVec->Z));
}

/**
 * @name    Fus_ClampUnit
 * @brief   Hold a value inside the range a sine can occupy.
 * @param   Value   The value.
 * @retval  The value, clipped to [-1, 1].
 * @note    asinf is undefined outside [-1, 1] and returns a NaN rather than an
 *          error, and a ratio of 1.0000001 is what a normalised vector gives
 *          you on a bad day. A NaN here would reach the quaternion and then the
 *          covariance, and it never comes back out on its own.
 */
static float Fus_ClampUnit(float Value)
{
    if (Value > 1.0f)
    {
        return 1.0f;
    }
    if (Value < -1.0f)
    {
        return -1.0f;
    }
    return Value;
}

/**
 * @name    Fus_QuatNorm
 * @brief   Scale a quaternion to unit length, in place.
 * @param   pQ  The quaternion.
 * @retval  None
 * @note    Every operation that touches the quaternion in this module is
 *          followed by this call, which is the rule the theory note sets out.
 *          The accumulation is small but it is not zero, and a quaternion that
 *          drifts off the unit sphere stops being a rotation: the error shows
 *          up as a scale factor on every vector it turns.
 * @note    A quaternion with no length at all is replaced by the identity
 *          rather than divided by zero. That is a measurement that did not
 *          happen, and level is the honest answer to it.
 */
static void Fus_QuatNorm(Fus_Quat_t *pQ)
{
    float Mag = sqrtf((pQ->W * pQ->W) + (pQ->X * pQ->X) + (pQ->Y * pQ->Y) +
                      (pQ->Z * pQ->Z));

    if (Mag > FUS_NORM_MIN)
    {
        pQ->W /= Mag;
        pQ->X /= Mag;
        pQ->Y /= Mag;
        pQ->Z /= Mag;
    }
    else
    {
        pQ->W = 1.0f;
        pQ->X = 0.0f;
        pQ->Y = 0.0f;
        pQ->Z = 0.0f;
    }
}

/**
 * @name    Fus_QuatFromEuler
 * @brief   Build a body-to-nav quaternion from roll, pitch and yaw.
 * @param   Roll    Rotation about body X, radians.
 * @param   Pitch   Rotation about body Y, radians.
 * @param   Yaw     Rotation about body Z, radians.
 * @param   pQ      Receives the quaternion, which is already unit length.
 * @retval  None
 * @note    The order is Z, then Y, then X, which is the order the angles are
 *          read in and the order the nav frame implies: yaw is applied in the
 *          nav frame, roll in the body frame that is left.
 * @note    The result is unit length by construction -- every term is a product
 *          of half-angle sines and cosines -- so a caller that needs it
 *          normalised is asking for a check rather than for a fix. The startup
 *          reference normalises anyway, because it is the one place where the
 *          angles come from a measurement rather than from arithmetic.
 */
static void Fus_QuatFromEuler(float Roll, float Pitch, float Yaw,
                              Fus_Quat_t *pQ)
{
    float cr = cosf(Roll * 0.5f);
    float sr = sinf(Roll * 0.5f);
    float cp = cosf(Pitch * 0.5f);
    float sp = sinf(Pitch * 0.5f);
    float cy = cosf(Yaw * 0.5f);
    float sy = sinf(Yaw * 0.5f);

    pQ->W = (cy * cp * cr) + (sy * sp * sr);
    pQ->X = (cy * cp * sr) - (sy * sp * cr);
    pQ->Y = (cy * sp * cr) + (sy * cp * sr);
    pQ->Z = (sy * cp * cr) - (cy * sp * sr);
}

/**
 * @name    Fus_EulerFromQuat
 * @brief   Read roll, pitch and yaw back out of a body-to-nav quaternion.
 * @param   pQ      The quaternion.
 * @param   pRoll   Receives roll, radians.
 * @param   pPitch  Receives pitch, radians.
 * @param   pYaw    Receives yaw, radians.
 * @retval  None
 * @note    Yaw is undefined when pitch is at the pole, and this form is the one
 *          that degrades gracefully there: at pitch = +-90 degrees roll and yaw
 *          describe the same rotation, so the pair is arbitrary even though
 *          each number is still finite. The quaternion is what carries the real
 *          attitude through that region -- which is the whole reason the state
 *          is a quaternion and the angles are derived from it rather than the
 *          other way round.
 */
static void Fus_EulerFromQuat(const Fus_Quat_t *pQ, float *pRoll,
                              float *pPitch, float *pYaw)
{
    float SinPitch = Fus_ClampUnit(2.0f * ((pQ->W * pQ->Y) - (pQ->Z * pQ->X)));

    *pRoll  = atan2f(2.0f * ((pQ->W * pQ->X) + (pQ->Y * pQ->Z)),
                     1.0f - (2.0f * ((pQ->X * pQ->X) + (pQ->Y * pQ->Y))));
    *pPitch = asinf(SinPitch);
    *pYaw   = atan2f(2.0f * ((pQ->W * pQ->Z) + (pQ->X * pQ->Y)),
                     1.0f - (2.0f * ((pQ->Y * pQ->Y) + (pQ->Z * pQ->Z))));
}

/**
 * @name    Fus_QuatVec
 * @brief   Rotate a vector from the body frame into the nav frame.
 * @param   pQ      Body-to-nav quaternion, unit length.
 * @param   pVec    The vector, in the body frame.
 * @param   pOut    The same vector, in the nav frame.
 * @retval  None
 * @note    The cross-product form rather than the rotation matrix: it is fewer
 *          multiply-adds than building the matrix and it never materialises the
 *          matrix, so there is no nine-element intermediate to get wrong. The
 *          length is preserved, which the startup reference relies on.
 */
static void Fus_QuatVec(const Fus_Quat_t *pQ, const Fus_Vec3_t *pVec,
                        Fus_Vec3_t *pOut)
{
    float tx = 2.0f * ((pQ->Y * pVec->Z) - (pQ->Z * pVec->Y));
    float ty = 2.0f * ((pQ->Z * pVec->X) - (pQ->X * pVec->Z));
    float tz = 2.0f * ((pQ->X * pVec->Y) - (pQ->Y * pVec->X));

    pOut->X = pVec->X + (pQ->W * tx) + ((pQ->Y * tz) - (pQ->Z * ty));
    pOut->Y = pVec->Y + (pQ->W * ty) + ((pQ->Z * tx) - (pQ->X * tz));
    pOut->Z = pVec->Z + (pQ->W * tz) + ((pQ->X * ty) - (pQ->Y * tx));
}

/**
 * @name    Fus_QuatMul
 * @brief   Hamilton product: pOut = pA (x) pB.
 * @param   pA      The left factor.
 * @param   pB      The right factor.
 * @param   pOut    Receives the product.
 * @retval  None
 * @note    The product composes the rotations, with the right factor applied
 *          first -- reading pA (x) pB as "b, then a". The error-state update is
 *          written to that convention rather than against it, so the order
 *          matters at every call site.
 * @note    The four components are written out only once all of them have been
 *          read, so a caller may pass the output over an input.
 */
static void Fus_QuatMul(const Fus_Quat_t *pA, const Fus_Quat_t *pB,
                        Fus_Quat_t *pOut)
{
    float W = (pA->W * pB->W) - (pA->X * pB->X) - (pA->Y * pB->Y) -
              (pA->Z * pB->Z);
    float X = (pA->W * pB->X) + (pA->X * pB->W) + (pA->Y * pB->Z) -
              (pA->Z * pB->Y);
    float Y = (pA->W * pB->Y) - (pA->X * pB->Z) + (pA->Y * pB->W) +
              (pA->Z * pB->X);
    float Z = (pA->W * pB->Z) + (pA->X * pB->Y) - (pA->Y * pB->X) +
              (pA->Z * pB->W);

    pOut->W = W;
    pOut->X = X;
    pOut->Y = Y;
    pOut->Z = Z;
}

/**
 * @name    Fus_QuatVecT
 * @brief   Rotate a vector from the nav frame back into the body frame.
 * @param   pQ      Body-to-nav quaternion, unit length.
 * @param   pVec    The vector, in the nav frame.
 * @param   pOut    The same vector, in the body frame.
 * @retval  None
 * @note    The inverse rotation is the conjugate's rotation, so this hands the
 *          conjugate to the same code instead of repeating the cross-product
 *          form with three signs flipped -- one place for that expression to be
 *          right in, rather than two.
 * @note    Every observation needs this direction. The sensors measure in the
 *          body frame and the reference directions they are compared against
 *          live in the nav frame, so one side of the comparison has to cross
 *          over, and it is cheaper to turn the fixed reference than the
 *          measurement -- the reference is one vector, the measurement is a
 *          fresh one every tick.
 */
static void Fus_QuatVecT(const Fus_Quat_t *pQ, const Fus_Vec3_t *pVec,
                         Fus_Vec3_t *pOut)
{
    Fus_Quat_t Inv;

    Inv.W = pQ->W;
    Inv.X = -pQ->X;
    Inv.Y = -pQ->Y;
    Inv.Z = -pQ->Z;

    Fus_QuatVec(&Inv, pVec, pOut);
}

/**
 * @name    Fus_RotQuat
 * @brief   Move a quaternion by a small rotation: pOut = pQ (x) dq(pVec).
 * @param   pQ      The quaternion being moved.
 * @param   pVec    The rotation vector, body frame, radians.
 * @param   pOut    Receives the result.
 * @retval  None
 * @note    Right-multiplied, because the error state this carries is defined in
 *          the body frame -- see the error-state note in the header. Left and
 *          right are not interchangeable and the difference is not a sign: the
 *          two are related by the attitude being estimated, so an update applied
 *          the wrong way round moves the attitude away from the measurement
 *          instead of towards it, and does it fast enough to look like a
 *          diverging filter rather than like a transposed index.
 * @note    The exact half-angle form rather than the first-order 1 + v/2. It
 *          costs one square root and two transcendentals on a correction that
 *          runs at 100 Hz, and what it buys is that a large correction stays a
 *          rotation: the linearised form shortens the quaternion by |v|^2 / 8,
 *          which the normalisation afterwards then undoes by pulling the whole
 *          attitude towards the identity.
 */
static void Fus_RotQuat(const Fus_Quat_t *pQ, const Fus_Vec3_t *pVec,
                        Fus_Quat_t *pOut)
{
    Fus_Quat_t Delta;
    float Mag = Fus_VecMag(pVec);

    if (Mag < FUS_NORM_MIN)
    {
        Delta.W = 1.0f;
        Delta.X = 0.0f;
        Delta.Y = 0.0f;
        Delta.Z = 0.0f;
    }
    else
    {
        float s = sinf(Mag * 0.5f) / Mag;

        Delta.W = cosf(Mag * 0.5f);
        Delta.X = pVec->X * s;
        Delta.Y = pVec->Y * s;
        Delta.Z = pVec->Z * s;
    }

    Fus_QuatMul(pQ, &Delta, pOut);
}

/**
 * @name    Fus_MatMul
 * @brief   Dense matrix product, pC = pA * pB.
 * @param   pA      The left factor, RowsA by Shared, row-major.
 * @param   pB      The right factor, Shared by ColsB, row-major.
 * @param   pC      Receives RowsA by ColsB. May not alias either input.
 * @param   RowsA   Rows in pA and in the product.
 * @param   Shared  Columns in pA and rows in pB.
 * @param   ColsB   Columns in pB and in the product.
 * @retval  None
 * @note    One general loop rather than a hand-unrolled body per shape. This
 *          file multiplies 6x6 by 6x6, 6x6 by 6x3, 3x6 by 6x3 and 6x3 by 3x6,
 *          and four unrolled variants are four chances to write an index that
 *          is transposed and still compiles. The cost does not matter here: a
 *          6x6 product is 216 multiply-adds on an M7 with a hardware FPU, at
 *          100 Hz.
 */
static void Fus_MatMul(const float *pA, const float *pB, float *pC,
                       int RowsA, int Shared, int ColsB)
{
    int i, j, k;

    for (i = 0; i < RowsA; i++)
    {
        for (j = 0; j < ColsB; j++)
        {
            float Sum = 0.0f;

            for (k = 0; k < Shared; k++)
            {
                Sum += pA[(i * Shared) + k] * pB[(k * ColsB) + j];
            }
            pC[(i * ColsB) + j] = Sum;
        }
    }
}

/**
 * @name    Fus_MatMulT
 * @brief   Matrix product with the second factor transposed: pC = pA * pB'.
 * @param   pA      The left factor, RowsA by Shared, row-major.
 * @param   pB      The factor to transpose, RowsB by Shared, row-major.
 * @param   pC      Receives RowsA by RowsB. May not alias either input.
 * @param   RowsA   Rows in pA and in the product.
 * @param   Shared  Columns in pA and in pB.
 * @param   RowsB   Rows in pB and columns in the product.
 * @retval  None
 * @note    Both places the covariance is propagated take this shape -- Phi P
 *          Phi' and (I-KH) P (I-KH)' -- and so does the observation's inner
 *          product. Taking the transpose inside the loop is what keeps a
 *          transposed copy of a 6x6 from having to exist at all, which is one
 *          less array to declare and one less place to get the direction of the
 *          transpose wrong.
 */
static void Fus_MatMulT(const float *pA, const float *pB, float *pC,
                        int RowsA, int Shared, int RowsB)
{
    int i, j, k;

    for (i = 0; i < RowsA; i++)
    {
        for (j = 0; j < RowsB; j++)
        {
            float Sum = 0.0f;

            for (k = 0; k < Shared; k++)
            {
                Sum += pA[(i * Shared) + k] * pB[(j * Shared) + k];
            }
            pC[(i * RowsB) + j] = Sum;
        }
    }
}

/**
 * @name    Fus_BuildPhi
 * @brief   The error state's discrete transition: pPhi = I + F dt.
 * @param   pRate   The corrected gyro rate, body frame, radians per second.
 * @param   Dt      The step, seconds.
 * @param   pPhi    Receives the 6x6, row-major.
 * @retval  None
 * @note    F, or what F dt is, has two blocks. The attitude error turns with the
 *          body -- dtheta' = -[w]x dtheta -- which is the statement that a
 *          rotation error already expressed in the body frame is carried around
 *          by the rotation the body is doing. Beside it sits -I: believing the
 *          rate is low by dbias is, to first order, believing the attitude is
 *          wrong by dbias per second, so a bias error drives an attitude error
 *          directly.
 * @note    The bottom block row is zero. The bias is a random walk with no
 *          dynamics of its own, which is what Q_BIAS_VAR says too: nothing here
 *          pushes it, it only drifts.
 * @note    First order in dt is enough, and the reason is that F is a rate: F dt
 *          is |w| dt, about 0.01 at four radians per second, so the term dropped
 *          is four orders of magnitude down on the one kept.
 */
static void Fus_BuildPhi(const Fus_Vec3_t *pRate, float Dt, float *pPhi)		//误差状态转移矩阵 Phi = I + F*dt
{
    int i;

    for (i = 0; i < (FUS_STATE_DIM * FUS_STATE_DIM); i++)
    {
        pPhi[i] = 0.0f;
    }

    /* -[w]x dt */			 // 1. 姿态误差传播：dθ' = -[w]x * dθ
    pPhi[1]  =  pRate->Z * Dt;		
    pPhi[2]  = -pRate->Y * Dt;
    pPhi[6]  = -pRate->Z * Dt;
    pPhi[8]  =  pRate->X * Dt;
    pPhi[12] =  pRate->Y * Dt;
    pPhi[13] = -pRate->X * Dt;

    /* -I dt, and the identity itself */			 // 2. 零偏耦合：零偏误差直接积分成姿态误差
    pPhi[3]  = -Dt;
    pPhi[10] = -Dt;
    pPhi[17] = -Dt;
    for (i = 0; i < FUS_STATE_DIM; i++)
    {
        pPhi[(i * FUS_STATE_DIM) + i] += 1.0f;
    }
}

/**
 * @name    Fus_CholFactor
 * @brief   Cholesky factorisation: pA = pL * pL', with pL lower triangular.
 * @param   pA      The matrix, symmetric positive definite, row-major.
 * @param   pL      Receives the factor, Dim by Dim, row-major.
 * @param   Dim     Side of both.
 * @retval  0 on success, -1 when pA is not positive definite.
 * @note    This is the inverse the theory note asks for, and it is the right
 *          choice for a reason beyond speed: the matrix being inverted is a
 *          covariance, which is positive definite by construction, and the
 *          factorisation is the operation that can tell you so. A general
 *          inverse of a covariance that has gone wrong returns a matrix made of
 *          noise and the filter carries on; this returns a failure, and the
 *          caller treats that as a tick with no correction rather than as
 *          something to keep going through.
 */
static int Fus_CholFactor(const float *pA, float *pL, int Dim)	// Cholesky 分解：A = L * L^T，要求 A 对称正定
{
    int i, j, k;

    for (i = 0; i < (Dim * Dim); i++)
    {
        pL[i] = 0.0f;
    }

    for (i = 0; i < Dim; i++)
    {
        for (j = 0; j <= i; j++)
        {
            float Sum = pA[(i * Dim) + j];

            for (k = 0; k < j; k++)
            {
                Sum -= pL[(i * Dim) + k] * pL[(j * Dim) + k];
            }

            if (i == j)
            {
                if (Sum <= 0.0f)
                {
                    return -1;
                }
                pL[(i * Dim) + i] = sqrtf(Sum);
            }
            else
            {
                pL[(i * Dim) + j] = Sum / pL[(j * Dim) + j];
            }
        }
    }

    return 0;
}

/**
 * @name    Fus_CholSolve
 * @brief   Solve pA pX = pB, given pA's Cholesky factor.
 * @param   pL      The factor from Fus_CholFactor().
 * @param   pB      The right-hand side, Dim long.
 * @param   pX      Receives the solution, Dim long.
 * @param   Dim     The system's order.
 * @retval  None
 * @note    Two triangular substitutions, which is all the factor leaves to do:
 *          forward for L y = b, back for L' x = y. pX is written twice along the
 *          way, first as y and then as the answer.
 * @note    Solving rather than forming an inverse is the other half of why this
 *          is the Cholesky path. The filter needs S^-1 multiplying a matrix it
 *          already holds, and it gets that by solving for one row at a time --
 *          so S is never inverted and the expression S^-1 never has to exist.
 */
static void Fus_CholSolve(const float *pL, const float *pB, float *pX, int Dim)
{
    int i, k;

    for (i = 0; i < Dim; i++)
    {
        float Sum = pB[i];

        for (k = 0; k < i; k++)
        {
            Sum -= pL[(i * Dim) + k] * pX[k];
        }
        pX[i] = Sum / pL[(i * Dim) + i];
    }

    for (i = Dim - 1; i >= 0; i--)
    {
        float Sum = pX[i];

        for (k = i + 1; k < Dim; k++)
        {
            Sum -= pL[(k * Dim) + i] * pX[k];
        }
        pX[i] = Sum / pL[(i * Dim) + i];
    }
}

/**
 * @name    Fus_IsMoving
 * @brief   Whether a gyro sample says the part is being handled.
 * @param   pGyroMdps   Aligned gyro, milli-degrees per second.
 * @retval  1 when any axis is past the threshold, 0 otherwise.
 * @note    This is what makes the reference trustworthy rather than lucky. A
 *          mean taken across a bump is a mean of two different bodies, and no
 *          amount of averaging afterwards can tell that from a quiet one.
 */
static int Fus_IsMoving(const Fus_Vec3_t *pGyroMdps)
{
    const float Limit = (float)(APP_FUSION_BIAS_MAX_DPS * 1000);

    if ((pGyroMdps->X > Limit) || (pGyroMdps->X < -Limit) ||
        (pGyroMdps->Y > Limit) || (pGyroMdps->Y < -Limit) ||
        (pGyroMdps->Z > Limit) || (pGyroMdps->Z < -Limit))
    {
        return 1;
    }
    return 0;
}

/**
 * @name    Fus_WindowRestart
 * @brief   Empty the startup window and start it again from now.
 * @param   Now The current TimeBase, milliseconds.
 * @retval  None
 * @note    Deliberately silent. A board that is being turned over while it
 *          boots would otherwise print a line per restart, and the console
 *          budget here is one line per period.
 */
static void Fus_WindowRestart(uint32_t Now)
{
    uint32_t i;

    for (i = 0u; i < 3u; i++)
    {
        Fus_SumGyro[i]  = 0.0f;
        Fus_SumAccel[i] = 0.0f;
        Fus_SumMag[i]   = 0.0f;
    }
    Fus_CountGyro     = 0u;
    Fus_CountAccel    = 0u;
    Fus_CountMag      = 0u;
    Fus_WindowTick    = Now;
    Fus_WindowStartTick = Now;
    Fus_StallLogged   = 0u;
}

/**
 * @name    Fus_ReferenceTake
 * @brief   Close the startup window: turn the three means into the gyro bias,
 *          the attitude to start from, and the field direction that attitude
 *          implies.
 * @param   Now The current TimeBase, milliseconds, for the line that reports
 *              how long the window took.
 * @retval  None
 * @note    The attitude comes from gravity alone, because gravity is the only
 *          direction the module knows before it has an attitude. At rest the
 *          accelerometer measures the specific force holding the body up, which
 *          is one g pointing away from down, so the direction it reads in the
 *          body frame IS the nav frame's up axis expressed in the body -- and
 *          that pair of angles is exactly roll and pitch. Yaw is not observable
 *          this way and is set to zero, which is the definition of the nav
 *          frame rather than a failure to measure it.
 * @note    The field reference is that attitude applied to the measured field.
 *          With nav X at the power-on heading the result is a general vector,
 *          which is the point: the textbook [mh, 0, mv] would be the right
 *          answer only if nav X were magnetic north.
 */
static void Fus_ReferenceTake(uint32_t Now)
{
    Fus_Vec3_t AccelBody;
    Fus_Vec3_t MagBody;
    float AccelMag = 0.0f;
    float MagMag = 0.0f;
    float Roll = 0.0f;
    float Pitch = 0.0f;
    int i;

    /* An empty chain is a part that never answered, and its mean would be a
     * division by zero. Only the gyro is required -- without it there is no
     * filter at all -- but every mean below is guarded anyway, because the
     * window closing is not the same claim as all three chains having
     * contributed. */
    if (Fus_CountGyro > 0u)																																		
    {
        Fus_GyroBiasRads.X = (Fus_SumGyro[0] / (float)Fus_CountGyro) *												//零飘均值后转换角速度
                             FUS_MDPS2RADS;
        Fus_GyroBiasRads.Y = (Fus_SumGyro[1] / (float)Fus_CountGyro) *
                             FUS_MDPS2RADS;
        Fus_GyroBiasRads.Z = (Fus_SumGyro[2] / (float)Fus_CountGyro) *
                             FUS_MDPS2RADS;
    }

    if (Fus_CountAccel > 0u)
    {
        AccelBody.X = Fus_SumAccel[0] / (float)Fus_CountAccel;																//求均值
        AccelBody.Y = Fus_SumAccel[1] / (float)Fus_CountAccel;
        AccelBody.Z = Fus_SumAccel[2] / (float)Fus_CountAccel;
        AccelMag = Fus_VecMag(&AccelBody);																										//加速度模长

        if (AccelMag > FUS_NORM_MIN)
        {
            /* Roll and pitch of the measured up-direction. Flat on the bench
             * these are both zero, and that is the check worth making first:
             * it reads the alignment, the sign convention and this whole
             * derivation in one number. */
            Pitch = asinf(Fus_ClampUnit(AccelBody.X / AccelMag));															//求静止的Pitch
            Roll  = atan2f(-AccelBody.Y, -AccelBody.Z);																				//求静止的Roll
        }
    }

    Fus_QuatFromEuler(Roll, Pitch, 0.0f, &Fus_Q);																							//计算四元数，不包含YAW
    Fus_QuatNorm(&Fus_Q);																																			//四元数归一化

    Fus_Att.Flags = APP_FUSION_HAVE_ATTITUDE;																									//姿态就绪标志位
    Fus_Att.AccelMagMg = (int32_t)AccelMag;																										//存入加速度模长

    if (Fus_CountMag > 0u)
    {
        MagBody.X = Fus_SumMag[0] / (float)Fus_CountMag;																			//计算磁场三轴均值
        MagBody.Y = Fus_SumMag[1] / (float)Fus_CountMag;
        MagBody.Z = Fus_SumMag[2] / (float)Fus_CountMag;
        MagMag = Fus_VecMag(&MagBody);																												//计算磁场模长

        if (MagMag > FUS_NORM_MIN)
        {
            Fus_QuatVec(&Fus_Q, &MagBody, &Fus_MagRefN);																			//换算到导航坐标系上

            /* A rotation preserves length, so the vector that went in
             * normalised comes out normalised. */
            Fus_MagRefN.X /= MagMag;																													//归一化
            Fus_MagRefN.Y /= MagMag;
            Fus_MagRefN.Z /= MagMag;
            Fus_Att.Flags |= APP_FUSION_HAVE_MAG;																							//磁力计就绪状态
        }
    }
    Fus_Att.MagMgaussMag = (int32_t)MagMag;																										//存入磁场模长
	
    /* The angles come out of the quaternion rather than being reported from the
     * Roll and Pitch computed above, so the console cannot disagree with the
     * state the filter is about to start from. From here on the same call keeps
     * them current, which is what leaves one path from the quaternion to the
     * angles instead of two that could drift apart. */
    Fus_Output();																																							//输出欧拉角

    /* The covariance starts at the reference's own uncertainty and nothing else.
     * The attitude is a couple of degrees away from the accelerometer's idea of
     * level, the bias is whatever the window produced -- and on this board that
     * window is worth less than it looks, because a part on flying leads drifts
     * slowly enough to pass the motion test and still not be still. That is the
     * case the bias variance is sized for: generously, so the filter is free to
     * pull the initial guess wherever the observations say it belongs. The
     * off-diagonal blocks start at zero, which claims the two errors are
     * uncorrelated -- true here, because neither has been propagated yet. */
    for (i = 0; i < (FUS_STATE_DIM * FUS_STATE_DIM); i++)																			//清零协方差矩阵
    {
        Fus_P[i] = 0.0f;
    }
    for (i = 0; i < FUS_BLOCK_DIM; i++)																												//姿态误差和零飘误差填入对角线协方差矩阵
    {
        Fus_P[(i * FUS_STATE_DIM) + i] = APP_FUSION_P0_ATT_VAR;
        Fus_P[((i + FUS_BLOCK_DIM) * FUS_STATE_DIM) + i + FUS_BLOCK_DIM] =
            APP_FUSION_P0_BIAS_VAR;
    }

    /* Left for App_Fusion_Log() to print. What survives to that point is the
     * bias and the field reference, both set here and both unchanged after, so
     * only the window's length and its sample count have to be carried. */
    App_FusionLog_Post(FUS_MSG_REFERENCE, Now - Fus_WindowStartTick,
                       Fus_CountGyro);
}

/**
 * @name    Fus_WindowFeed
 * @brief   Offer one sensor sample to the startup window.
 * @param   pSample The sample this tick produced.
 * @param   Now     The current TimeBase, milliseconds.
 * @retval  None
 * @note    A tick that delivered no gyro costs nothing: the window is counted
 *          in samples, not in ticks, so a chain that is running behind stretches
 *          the window instead of diluting it. The magnetometer is the reason
 *          this matters -- it runs continuously at its own 100 Hz against the
 *          poll's, the two are not synchronised, and a tick that finds no
 *          finished measurement has nothing to offer. Counting in samples is
 *          what keeps such a window from being diluted by the ticks that had
 *          nothing, which is the difference between a mean of measurements and
 *          a mean with holes in it.
 */
static void Fus_WindowFeed(const App_Sensor_Sample_t *pSample, uint32_t Now)
{
    int32_t Raw[3];
    Fus_Vec3_t Vec;

    if ((pSample->Flags & APP_SENSOR_HAVE_GYRO) != 0u)														//陀螺仪有数据
    {
        Raw[0] = pSample->GyroMdpsX;
        Raw[1] = pSample->GyroMdpsY;
        Raw[2] = pSample->GyroMdpsZ;
        Fus_ApplyAlign(Fus_AlignGyro, Raw, &Vec);																	//坐标系对齐

        if (Fus_IsMoving(&Vec) != 0)																							//静止采样
        {
            Fus_WindowRestart(Now);
            return;
        }

        Fus_SumGyro[0] += Vec.X;																									//累加误差
        Fus_SumGyro[1] += Vec.Y;
        Fus_SumGyro[2] += Vec.Z;
        Fus_CountGyro++;	
        Fus_WindowTick = Now;
        Fus_StallLogged = 0u;
    }

    if ((pSample->Flags & APP_SENSOR_HAVE_ACCEL) != 0u)														//加计有数据
    {
        Raw[0] = pSample->AccelMgX;
        Raw[1] = pSample->AccelMgY;
        Raw[2] = pSample->AccelMgZ;
        Fus_ApplyAlign(Fus_AlignAccel, Raw, &Vec);																//坐标系对齐

        Fus_SumAccel[0] += Vec.X;																									//累加误差
        Fus_SumAccel[1] += Vec.Y;
        Fus_SumAccel[2] += Vec.Z;
        Fus_CountAccel++;
    }

    if ((pSample->Flags & APP_SENSOR_HAVE_MAG) != 0u)															//磁力计有数据
    {
        Raw[0] = pSample->MagMgaussX;																							
        Raw[1] = pSample->MagMgaussY;
        Raw[2] = pSample->MagMgaussZ;
        Fus_ApplyAlign(Fus_AlignMag, Raw, &Vec);																	//坐标系对齐

        Fus_SumMag[0] += Vec.X;																										//累加误差
        Fus_SumMag[1] += Vec.Y;
        Fus_SumMag[2] += Vec.Z;
        Fus_CountMag++;
    }

    if (Fus_CountGyro >= APP_FUSION_BIAS_SAMPLES)																	//采样100次
    {
        Fus_ReferenceTake(Now);																										//静止零飘、计算四元数、计算协方差
        return;
    }

    /* Reported once per stall rather than once per tick, and it does not stop
     * the window: a sensor that starts answering late gets its reference then
     * instead of never. */
    if (((Now - Fus_WindowTick) >= APP_FUSION_BIAS_STALL_MS) &&
        (Fus_StallLogged == 0u))
    {
        Fus_StallLogged  = 1u;
        App_FusionLog_Post(FUS_MSG_STALL, 0u, Fus_CountGyro);
    }
}

/**
 * @name    Fus_Track
 * @brief   One tick's worth of bookkeeping once the reference exists: how big
 *          the body's acceleration and field vectors currently are.
 * @param   pSample The sample this tick produced.
 * @retval  None
 * @note    The magnitudes are kept current whether or not anything consumes
 *          them, because they are the two numbers that say whether the
 *          observations are still worth correcting on -- an accelerometer that
 *          is not reading one g is a body being pushed, and a field that has
 *          changed size is a field that has changed shape with it.
 */
static void Fus_Track(const App_Sensor_Sample_t *pSample)
{
    int32_t Raw[3];
    Fus_Vec3_t Vec;

    if ((pSample->Flags & APP_SENSOR_HAVE_ACCEL) != 0u)
    {
        Raw[0] = pSample->AccelMgX;
        Raw[1] = pSample->AccelMgY;
        Raw[2] = pSample->AccelMgZ;
        Fus_ApplyAlign(Fus_AlignAccel, Raw, &Vec);
        Fus_Att.AccelMagMg = (int32_t)Fus_VecMag(&Vec);
    }

    if ((pSample->Flags & APP_SENSOR_HAVE_MAG) != 0u)
    {
        Raw[0] = pSample->MagMgaussX;
        Raw[1] = pSample->MagMgaussY;
        Raw[2] = pSample->MagMgaussZ;
        Fus_ApplyAlign(Fus_AlignMag, Raw, &Vec);
        Fus_Att.MagMgaussMag = (int32_t)Fus_VecMag(&Vec);
    }
}

/**
 * @name    Fus_Predict
 * @brief   Advance the attitude on the gyro, and the covariance with it.
 * @param   pSample The sample this tick produced.
 * @param   Dt      The step, seconds, as measured between the samples the filter
 *                  actually consumed.
 * @retval  None
 * @note    Skipped whole when the gyro is not fresh -- state and covariance
 *          together, nothing updated but the timestamp. The alternative is to
 *          integrate the rate the buffer is still holding, which is a rate from
 *          a tick already known to have produced nothing. The next tick's
 *          measured step covers the gap instead, and does it without inventing
 *          an interval of motion.
 * @note    RK2, the midpoint rule, and the midpoint rule is also the cheap way
 *          to do this. The exact solution for a rate held over the step is the
 *          exponential map, which wants a square root and two transcendentals
 *          per step; the midpoint rule wants three quaternion products and no
 *          library call at all, and the two agree to third order -- both expand
 *          to 1 - (|w| dt)^2 / 8, which is the term that keeps the length right
 *          -- leaving the normalisation afterwards to take care of the rest.
 */
static void Fus_Predict(const App_Sensor_Sample_t *pSample, float Dt)
{
    int32_t Raw[3];
    Fus_Vec3_t Rate;
    Fus_Vec3_t Corrected;
    Fus_Quat_t Spin;
    Fus_Quat_t K1;
    Fus_Quat_t Mid;
    float Phi[FUS_STATE_DIM * FUS_STATE_DIM];
    float Tmp[FUS_STATE_DIM * FUS_STATE_DIM];
    int i;

    if ((pSample->Flags & APP_SENSOR_HAVE_GYRO) == 0u)																						//是否有数据
    {
        return;
    }

    Raw[0] = pSample->GyroMdpsX;
    Raw[1] = pSample->GyroMdpsY;
    Raw[2] = pSample->GyroMdpsZ;																																	//取陀螺仪三轴数据
    Fus_ApplyAlign(Fus_AlignGyro, Raw, &Rate);

    Rate.X *= FUS_MDPS2RADS;
    Rate.Y *= FUS_MDPS2RADS;
    Rate.Z *= FUS_MDPS2RADS;																																			//转换角速度

    /* What the body is really turning at, as far as the state currently believes
     * -- which is the whole reason the bias is carried in the state rather than
     * measured once and subtracted. */
    Corrected.X = Rate.X - Fus_GyroBiasRads.X;
    Corrected.Y = Rate.Y - Fus_GyroBiasRads.Y;
    Corrected.Z = Rate.Z - Fus_GyroBiasRads.Z;																										//减去零偏

    Spin.W = 0.0f;
    Spin.X = Corrected.X;
    Spin.Y = Corrected.Y;
    Spin.Z = Corrected.Z;																																				

    /* k1 = 0.5 q (x) w, the midpoint taken along it, then k2 = 0.5 q_mid (x) w
     * and the step is q + dt * k2. */
    Fus_QuatMul(&Fus_Q, &Spin, &K1);																															
    Mid.W = Fus_Q.W + (0.25f * Dt * K1.W);
    Mid.X = Fus_Q.X + (0.25f * Dt * K1.X);
    Mid.Y = Fus_Q.Y + (0.25f * Dt * K1.Y);
    Mid.Z = Fus_Q.Z + (0.25f * Dt * K1.Z);																											

    Fus_QuatMul(&Mid, &Spin, &K1);
    Fus_Q.W += 0.5f * Dt * K1.W;
    Fus_Q.X += 0.5f * Dt * K1.X;
    Fus_Q.Y += 0.5f * Dt * K1.Y;
    Fus_Q.Z += 0.5f * Dt * K1.Z;
    Fus_QuatNorm(&Fus_Q);																																	

    /* P = Phi P Phi' + Q. */
    Fus_BuildPhi(&Corrected, Dt, Phi);																														//构造状态转移矩阵
    Fus_MatMul(Phi, Fus_P, Tmp, FUS_STATE_DIM, FUS_STATE_DIM, FUS_STATE_DIM);
    Fus_MatMulT(Tmp, Phi, Fus_P, FUS_STATE_DIM, FUS_STATE_DIM, FUS_STATE_DIM);

    /* The step's process noise is the density times the step rather than a
     * per-step constant, because the step is measured and it moves. The two
     * blocks are independent and no cross terms are carried: the coupling
     * between them over one step is O(dt^2) on top of terms that are already
     * small. */
    for (i = 0; i < FUS_BLOCK_DIM; i++)																														//更新协方差
    {
        Fus_P[(i * FUS_STATE_DIM) + i] += APP_FUSION_Q_ATT_VAR * Dt;
        Fus_P[((i + FUS_BLOCK_DIM) * FUS_STATE_DIM) + i + FUS_BLOCK_DIM] +=
            APP_FUSION_Q_BIAS_VAR * Dt;
    }
}

/**
 * @name    Fus_RowsFill
 * @brief   Fill one three-row block of the observation: a measured body-frame
 *          direction against the direction the state predicts for it.
 * @param   pQ      The state's quaternion.
 * @param   pMeas   The measurement, body frame, already unit length.
 * @param   pRefN   The reference direction, nav frame, unit length.
 * @param   Row     The first row of the block to write, 0 or FUS_BLOCK_DIM.
 * @param   pH      The Jacobian, FUS_OBS_DIM_MAX by FUS_STATE_DIM, row-major.
 * @param   pZ      Receives this block's three residuals.
 * @retval  None
 * @note    One function for both sensors, because once the reference is
 *          expressed in the nav frame the rows are the same rows: what differs
 *          is which direction goes in and which R comes out, and both of those
 *          are arguments rather than branches. That is also what makes the sign
 *          of the second block the same argument as the first -- a mistake here
 *          would have to move the accelerometer too.
 * @note    The Jacobian is the skew matrix of the predicted direction, which is
 *          the body-frame error convention paying off -- in the nav frame every
 *          row would carry a rotation of the observation as well. The skew
 *          matrix has the predicted vector along its null space, and that is not
 *          a coincidence to be worked around: neither sensor says anything about
 *          rotation about its own reference direction, so the accelerometer's
 *          rows leave yaw alone and the magnetometer is what does not.
 * @note    The bias columns are left as the caller zeroed them. Neither sensor
 *          sees the bias over an instant -- it reaches the measurement through
 *          the attitude, which is what the predict step's -I already carries.
 */
static void Fus_RowsFill(const Fus_Quat_t *pQ, const Fus_Vec3_t *pMeas,
                         const Fus_Vec3_t *pRefN, int Row, float *pH, float *pZ)												//构造观测矩阵和残差
{
    Fus_Vec3_t Pred;

    Fus_QuatVecT(pQ, pRefN, &Pred);																						

    pZ[Row]     = pMeas->X - Pred.X;
    pZ[Row + 1] = pMeas->Y - Pred.Y;
    pZ[Row + 2] = pMeas->Z - Pred.Z;

    pH[(Row * FUS_STATE_DIM) + 1]        = -Pred.Z;
    pH[(Row * FUS_STATE_DIM) + 2]        =  Pred.Y;
    pH[((Row + 1) * FUS_STATE_DIM) + 0]  =  Pred.Z;
    pH[((Row + 1) * FUS_STATE_DIM) + 2]  = -Pred.X;
    pH[((Row + 2) * FUS_STATE_DIM) + 0]  = -Pred.Y;
    pH[((Row + 2) * FUS_STATE_DIM) + 1]  =  Pred.X;
}

/**
 * @name    Fus_Update
 * @brief   Apply one correction over whatever rows the tick produced.
 * @param   pH      The Jacobian, Dim by FUS_STATE_DIM, row-major.
 * @param   pR      The measurement covariance, Dim by Dim, row-major.
 * @param   pZ      The residuals, Dim long.
 * @param   Dim     Rows in the observation: three, or six with the magnetometer.
 * @retval  0 when the state and the covariance moved, -1 when the correction was
 *          skipped because S was not positive definite.
 * @note    The whole correction in one place, with the dimensions as arguments,
 *          which is what keeps three rows and six rows from being two code paths
 *          that can disagree. The mirror that settled the accelerometer's sign
 *          was extended this way before this was written, and it is the reason
 *          the magnetometer block can be added without re-deriving anything: the
 *          structure is the accelerometer's, and only the reference moved.
 * @note    Both blocks go in as ONE update rather than as two in sequence. The
 *          two measurements are of the same body at the same instant, so their
 *          residuals are correlated through the attitude error, and that
 *          correlation is exactly the off-diagonal block of R that a batch
 *          update carries for free and two sequential updates through the same
 *          P would have to be given by hand. Sequential would also let the first
 *          block's correction change the prediction the second is compared
 *          against, which is a different filter.
 * @note    S is H P H' + R, and H P H' on its own is not positive definite when
 *          P is thin in a direction the rows cannot see -- which is exactly the
 *          yaw direction's condition on a tick with no magnetometer row. Adding
 *          R is what fixes that, and it does so by more than a nudge: H P H' is
 *          positive semidefinite, so S >= R in the Loewner order and every
 *          eigenvalue of S sits at or above the smallest one R has -- 4.0e-6,
 *          whatever P has done. That floor is the reason the factorisation below
 *          is expected to succeed rather than hoped to, and it is also why R
 *          must stay a real measurement covariance and not be shrunk to make the
 *          filter "faster" -- it is the only thing standing between a singular
 *          S and a skipped correction.
 * @note    R is used at stride Dim, not at FUS_OBS_DIM_MAX. The caller builds it
 *          that way, and the products below read it at the same stride.
 * @note    The covariance update is the Joseph form, at the cost of two extra
 *          6x6 products. That is not tidiness: the next step factorises P, and a
 *          P that has lost its positive definiteness under the shorter form
 *          would turn an ordinary correction into a failed factorisation.
 * @note    The matrices are local, which is a stack frame of about 2 KB against
 *          the eight kilobytes the startup file reserves. Nothing here is
 *          reentrant, but nothing calls it twice either.
 */
static int Fus_Update(const float *pH, const float *pR, const float *pZ,
                      int Dim)
{
    float V[FUS_STATE_DIM * FUS_OBS_DIM_MAX];      /* 中间矩阵，存 P*H' */
    float W[FUS_STATE_DIM * FUS_OBS_DIM_MAX];      /* 卡尔曼增益 K */
    float S[FUS_OBS_DIM_MAX * FUS_OBS_DIM_MAX];    /* 残差协方差 S */
    float L[FUS_OBS_DIM_MAX * FUS_OBS_DIM_MAX];    /* S 的 Cholesky 因子 */
    float IKH[FUS_STATE_DIM * FUS_STATE_DIM];      /* I - K*H */
    float M[FUS_STATE_DIM * FUS_STATE_DIM];        /* 中间矩阵 */
    float KRK[FUS_STATE_DIM * FUS_STATE_DIM];      /* K*R*K' */
    float Delta[FUS_STATE_DIM];                    /* 状态修正量 dx */
    Fus_Vec3_t DTheta;                             /* 姿态修正旋转向量 */
    Fus_Quat_t QNew;                               /* 修正后的四元数 */
    int i, j;

    /* ========== S = H P H' + R ========== */
    Fus_MatMul(pH, Fus_P, V, Dim, FUS_STATE_DIM, FUS_STATE_DIM);      /* V = H*P */
    Fus_MatMulT(V, pH, S, Dim, FUS_STATE_DIM, Dim);                   /* S = V*H' = H*P*H' */
    for (i = 0; i < (Dim * Dim); i++)
    {
        S[i] += pR[i];                                                /* S += R */
    }

    /* ========== Cholesky 分解 S ========== */
    if (Fus_CholFactor(S, L, Dim) != 0)
    {
        return -1;                                                    /* 不正定，跳过本 tick */
    }

    /* ========== K = P H' S^-1 ========== */
    Fus_MatMulT(Fus_P, pH, V, FUS_STATE_DIM, FUS_STATE_DIM, Dim);     /* V = P*H' */
    for (i = 0; i < FUS_STATE_DIM; i++)
    {
        float Rhs[FUS_OBS_DIM_MAX];
        float Sol[FUS_OBS_DIM_MAX];

        for (j = 0; j < Dim; j++)
        {
            Rhs[j] = V[(i * Dim) + j];                                /* 取 P*H' 的第 i 行 */
        }
        Fus_CholSolve(L, Rhs, Sol, Dim);                              /* 解 S*x = Rhs */
        for (j = 0; j < Dim; j++)
        {
            W[(i * Dim) + j] = Sol[j];                                /* K 的第 i 行 */
        }
    }

    /* ========== dx = K * r ========== */
    for (i = 0; i < FUS_STATE_DIM; i++)
    {
        float Sum = 0.0f;

        for (j = 0; j < Dim; j++)
        {
            Sum += W[(i * Dim) + j] * pZ[j];                          /* K 的第 i 行 点乘 残差 */
        }
        Delta[i] = Sum;
    }

    /* ========== 把修正量注入状态 ========== */
    DTheta.X = Delta[0];                                              /* 姿态修正：绕 X */
    DTheta.Y = Delta[1];                                              /* 姿态修正：绕 Y */
    DTheta.Z = Delta[2];                                              /* 姿态修正：绕 Z */
    Fus_RotQuat(&Fus_Q, &DTheta, &QNew);                              /* 四元数右乘修正量 */
    Fus_QuatNorm(&QNew);                                              /* 归一化 */
    Fus_Q = QNew;

    Fus_GyroBiasRads.X += Delta[3];                                   /* 零偏修正：X */
    Fus_GyroBiasRads.Y += Delta[4];                                   /* 零偏修正：Y */
    Fus_GyroBiasRads.Z += Delta[5];                                   /* 零偏修正：Z */

    /* ========== P = (I-KH) P (I-KH)' + K R K' ========== */
    Fus_MatMul(W, pH, IKH, FUS_STATE_DIM, Dim, FUS_STATE_DIM);        /* IKH = K*H */
    for (i = 0; i < (FUS_STATE_DIM * FUS_STATE_DIM); i++)
    {
        IKH[i] = -IKH[i];                                             /* IKH = -K*H */
    }
    for (i = 0; i < FUS_STATE_DIM; i++)
    {
        IKH[(i * FUS_STATE_DIM) + i] += 1.0f;                         /* IKH = I - K*H */
    }

    Fus_MatMul(IKH, Fus_P, M, FUS_STATE_DIM, FUS_STATE_DIM, FUS_STATE_DIM);    /* M = (I-KH)*P */
    Fus_MatMulT(M, IKH, Fus_P, FUS_STATE_DIM, FUS_STATE_DIM, FUS_STATE_DIM);   /* P = M*(I-KH)' */

    Fus_MatMul(W, pR, V, FUS_STATE_DIM, Dim, Dim);                    /* V = K*R */
    Fus_MatMulT(V, W, KRK, FUS_STATE_DIM, Dim, FUS_STATE_DIM);        /* KRK = V*K' = K*R*K' */
    for (i = 0; i < (FUS_STATE_DIM * FUS_STATE_DIM); i++)
    {
        Fus_P[i] += KRK[i];                                           /* P += K*R*K' */
    }

    return 0;                                                         /* 更新成功 */
}



/**
 * @name    Fus_Correct
 * @brief   Correct the attitude on the accelerometer, and on the magnetometer
 *          when there is a fresh reading of it.
 * @param   pSample The sample this tick produced.
 * @retval  None
 * @note    The blocks are built from whatever this tick actually produced and
 *          the update runs at that size. That is worth having rather than
 *          guarding against: the two parts run at the same nominal rate but they
 *          are not synchronised, so either one can arrive without the other, and
 *          a correction on whichever did is strictly better than none.
 * @note    The measurements are normalised first. What each sensor knows is a
 *          direction -- which way is down, and which way the field points -- and
 *          its length says only whether the body is being pushed or the field
 *          disturbed; comparing the two unnormalised would let a shove move the
 *          attitude. That is the whole of what this module assumes about the
 *          magnetometer's hard iron, and it is why an offset that shifts the
 *          field's direction is not corrected here.
 * @note    The two R values are not the same number in the same units, and the
 *          line between them is the point of the block-diagonal R: the
 *          accelerometer's g^2 and the magnetometer's unitless field direction
 *          have nothing to do with each other and are not averaged anywhere.
 *          Each residual is reported against its own.
 */
static void Fus_Correct(const App_Sensor_Sample_t *pSample)
{
    int32_t Raw[3];
    Fus_Vec3_t Meas;
    Fus_Vec3_t ResAccel;
    Fus_Vec3_t ResMag;
    float MeasMag;
    float H[FUS_OBS_DIM_MAX * FUS_STATE_DIM];
    float R[FUS_OBS_DIM_MAX * FUS_OBS_DIM_MAX];
    float Z[FUS_OBS_DIM_MAX];
    int Dim = 0;
    int i;

    for (i = 0; i < (FUS_OBS_DIM_MAX * FUS_STATE_DIM); i++)
    {
        H[i] = 0.0f;
    }

    if ((pSample->Flags & APP_SENSOR_HAVE_ACCEL) != 0u)
    {
        Raw[0] = pSample->AccelMgX;
        Raw[1] = pSample->AccelMgY;
        Raw[2] = pSample->AccelMgZ;
        Fus_ApplyAlign(Fus_AlignAccel, Raw, &Meas);

        MeasMag = Fus_VecMag(&Meas);
        if (MeasMag > FUS_NORM_MIN)
        {
            Meas.X /= MeasMag;
            Meas.Y /= MeasMag;
            Meas.Z /= MeasMag;

            Fus_RowsFill(&Fus_Q, &Meas, &Fus_NavUp, Dim, H, Z);												//构造加速度计观测矩阵
            Dim += FUS_BLOCK_DIM;
        }
    }

    /* The magnetometer's rows need both halves to exist: a reference direction
     * taken at startup, and a reading from this tick. The flag is the sensor
     * module's, so it is a fresh reading and not the one the buffer is holding. */
    if (((Fus_Att.Flags & APP_FUSION_HAVE_MAG) != 0u) &&
        ((pSample->Flags & APP_SENSOR_HAVE_MAG) != 0u))
    {
        Raw[0] = pSample->MagMgaussX;
        Raw[1] = pSample->MagMgaussY;
        Raw[2] = pSample->MagMgaussZ;
        Fus_ApplyAlign(Fus_AlignMag, Raw, &Meas);

        MeasMag = Fus_VecMag(&Meas);
        if (MeasMag > FUS_NORM_MIN)
        {
            Meas.X /= MeasMag;
            Meas.Y /= MeasMag;
            Meas.Z /= MeasMag;

            Fus_RowsFill(&Fus_Q, &Meas, &Fus_MagRefN, Dim, H, Z);														//构造磁力计观测矩阵
            Dim += FUS_BLOCK_DIM;
        }
    }

    if (Dim == 0)
    {
        return;
    }

    /* R at stride Dim: the accelerometer's block, then the magnetometer's when
     * there is one. Each is a scalar on its own diagonal, because each
     * measurement is a direction, so what it carries is a small tangential error
     * of roughly the same size whichever way it points. The off-diagonal and
     * cross-sensor entries are zero, which claims the two sensors' errors are
     * uncorrelated -- they are different parts reading different quantities, and
     * nothing here has a reason to say otherwise. */
    for (i = 0; i < (Dim * Dim); i++)
    {
        R[i] = 0.0f;
    }
    for (i = 0; i < FUS_BLOCK_DIM; i++)
    {
        R[(i * Dim) + i] = APP_FUSION_R_ACC_VAR;
        if (Dim > FUS_BLOCK_DIM)
        {
            R[((i + FUS_BLOCK_DIM) * Dim) + i + FUS_BLOCK_DIM] =
                APP_FUSION_R_MAG_VAR;
        }
    }

    if (Fus_Update(H, R, Z, Dim) != 0)																											//EKF修正
    {
        if (Fus_CholFailLogged == 0u)
        {
            Fus_CholFailLogged = 1u;
            App_FusionLog_Post(FUS_MSG_CHOLFAIL, 0u, 0u);
        }
        return;
    }

    /* Each disagreement in the units the console can print: the measurements are
     * unit vectors, so a residual's length is the angle between where that
     * sensor says its reference direction is and where the state says it is, in
     * radians. The magnetometer's stays put on the ticks that carried no row. */
    ResAccel.X = Z[0];
    ResAccel.Y = Z[1];
    ResAccel.Z = Z[2];
    Fus_Att.ResidualMilli = (int32_t)(Fus_VecMag(&ResAccel) * 1000.0f);

    if (Dim > FUS_BLOCK_DIM)
    {
        ResMag.X = Z[FUS_BLOCK_DIM];
        ResMag.Y = Z[FUS_BLOCK_DIM + 1];
        ResMag.Z = Z[FUS_BLOCK_DIM + 2];
        Fus_Att.ResidualMagMilli = (int32_t)(Fus_VecMag(&ResMag) * 1000.0f);
    }
}

/**
 * @name    Fus_Output
 * @brief   Publish the quaternion and the angles it implies.
 * @param   None
 * @retval  None
 * @note    The angles are derived from the quaternion every time rather than
 *          carried beside it, so the two cannot drift apart, and the console
 *          line reads what a consumer would get rather than a second opinion.
 */
static void Fus_Output(void)
{
    float Roll = 0.0f;
    float Pitch = 0.0f;
    float Yaw = 0.0f;

    Fus_EulerFromQuat(&Fus_Q, &Roll, &Pitch, &Yaw);

    Fus_Att.Qw = Fus_Q.W;
    Fus_Att.Qx = Fus_Q.X;
    Fus_Att.Qy = Fus_Q.Y;
    Fus_Att.Qz = Fus_Q.Z;
    Fus_Att.RollMdeg  = (int32_t)(Roll * FUS_RAD2DEG * 1000.0f);
    Fus_Att.PitchMdeg = (int32_t)(Pitch * FUS_RAD2DEG * 1000.0f);
    Fus_Att.YawMdeg   = (int32_t)(Yaw * FUS_RAD2DEG * 1000.0f);
}

/**
 * @name    App_Fusion_Init
 * @brief   Reset the filter and open the startup window.
 * @param   None
 * @retval  None
 * @note    Every field is written rather than left to the loader, so a
 *          warm reset that does not clear RAM cannot present a consumer with
 *          half an attitude from the previous run.
 * @note    The covariance is zeroed but not seeded. Its starting value belongs
 *          to the reference, so Fus_ReferenceTake() seeds it; until then nothing
 *          propagates it and a zero matrix is the honest value for a filter that
 *          has not started.
 */
void App_Fusion_Init(void)
{
    int i;

    Fus_Att.Qw = 1.0f;
    Fus_Att.Qx = 0.0f;
    Fus_Att.Qy = 0.0f;
    Fus_Att.Qz = 0.0f;
    Fus_Att.RollMdeg = 0;
    Fus_Att.PitchMdeg = 0;
    Fus_Att.YawMdeg = 0;
    Fus_Att.AccelMagMg = 0;
    Fus_Att.MagMgaussMag = 0;
    Fus_Att.ResidualMilli = 0;
    Fus_Att.ResidualMagMilli = 0;
    Fus_Att.TickMs = TimeBase;
    Fus_Att.Flags = 0u;

    Fus_Q.W = 1.0f;
    Fus_Q.X = 0.0f;
    Fus_Q.Y = 0.0f;
    Fus_Q.Z = 0.0f;
    Fus_GyroBiasRads.X = 0.0f;
    Fus_GyroBiasRads.Y = 0.0f;
    Fus_GyroBiasRads.Z = 0.0f;
    Fus_MagRefN.X = 0.0f;
    Fus_MagRefN.Y = 0.0f;
    Fus_MagRefN.Z = 0.0f;

    for (i = 0; i < (FUS_STATE_DIM * FUS_STATE_DIM); i++)
    {
        Fus_P[i] = 0.0f;
    }
    Fus_CholFailLogged = 0u;

    Fus_WindowRestart(TimeBase);

    /* Zero, because the init order creates this before the tick that stamps the
     * samples: the first sample this filter sees carries that clock's first
     * nonzero value, so it cannot be mistaken for a repeat. */
    Fus_SampleTickUs = 0u;

    /* The console's slow gate is the log module's now, and it starts from the
     * same instant it did when that counter lived here. */
    App_FusionLog_Init();
}

/**
 * @name    App_Fusion_Update
 * @brief   Consume one sensor sample.
 * @param   None
 * @retval  None
 * @note    Prints nothing, and must not: the console is written by
 *          App_Fusion_Log() from the main loop, so that this can run in the
 *          tick interrupt without waiting on a transmitter.
 */
void App_Fusion_Update(void)
{
    const App_Sensor_Sample_t *pSample = App_Sensor_GetSample();
    uint32_t Now = TimeBase;
    uint32_t PrevUs;
    float Dt;

    /* The caller only steps the filter on a tick that produced a sample, so
     * this is a guard rather than the gate it used to be. It stays because the
     * buffer is filled from the interrupt: a repeated timestamp is the one
     * thing that would integrate twice for one reading. */
    if (pSample->TickUs == Fus_SampleTickUs)
    {
        return;
    }

    PrevUs = Fus_SampleTickUs;
    Fus_SampleTickUs = pSample->TickUs;
    Fus_Att.TickMs = pSample->TickMs;

    if ((Fus_Att.Flags & APP_FUSION_HAVE_ATTITUDE) == 0u)			//启动静止采样
    {
        Fus_WindowFeed(pSample, Now);
    }
    else
    {
        /* Measured, and clamped at the top only. A gap longer than the clamp is
         * a main loop that stalled, and the filter has no way to know what the
         * body did during it; integrating the clamp's worth of the rate read
         * after the stall is the least wrong answer available, and it is
         * deliberately not the whole gap. */
        Dt = (float)(pSample->TickUs - PrevUs) * 1.0e-6f;													//计算步长
        if (Dt > APP_FUSION_DT_MAX)
        {
            Dt = APP_FUSION_DT_MAX;
        }

        Fus_Track(pSample);																					//模长
        Fus_Predict(pSample, Dt);																		//预测姿态和协方差
        Fus_Correct(pSample);																				//观测修正
        Fus_Output();																								//输出姿态
    }
}

/**
 * @name    App_Fusion_GetAttitude
 * @brief   The latest attitude.
 * @param   None
 * @retval  Pointer to the module's one attitude buffer.
 */
const App_Fusion_Attitude_t *App_Fusion_GetAttitude(void)
{
    return &Fus_Att;
}

/**
 * @name    App_Fusion_GetGyroBias
 * @brief   The gyro rate bias the filter currently believes.
 * @param   pBiasRads   Receives the bias: three floats, body frame, radians per
 *                      second.
 * @retval  None
 * @note    This is the rate the predict step subtracts from the gyro, so it is
 *          the startup window's mean at first and a state estimate afterwards --
 *          each correction moves it. What the console prints as milli-degrees
 *          per second is this, converted, and the conversion stays at the
 *          caller: the module keeps its units in radians like the rest of the
 *          filter.
 * @note    Written in place from the sampling tick, so a main-loop caller can
 *          be reading it while it changes. It is three floats read back to back
 *          and the worst a live read can do is mix two ticks into one line,
 *          which is the consequence App_Fusion_GetAttitude() documents.
 */
void App_Fusion_GetGyroBias(float *pBiasRads)
{
    pBiasRads[0] = Fus_GyroBiasRads.X;
    pBiasRads[1] = Fus_GyroBiasRads.Y;
    pBiasRads[2] = Fus_GyroBiasRads.Z;
}

/**
 * @name    App_Fusion_GetMagRefN
 * @brief   The field reference the startup attitude implies.
 * @param   pRefN   Receives the reference: three floats, nav frame, unit length.
 * @retval  None
 * @note    Zero until the startup reference is taken, and meaningful only when
 *          APP_FUSION_HAVE_MAG is set in App_Fusion_GetAttitude()->Flags: it is
 *          the measured field rotated into the nav frame by the startup
 *          attitude, and the magnetometer's rows compare against it.
 * @note    Written once, by Fus_ReferenceTake(), and never again -- unlike the
 *          bias, this one does not move, so a caller that reads it late reads
 *          the same vector.
 */
void App_Fusion_GetMagRefN(float *pRefN)
{
    pRefN[0] = Fus_MagRefN.X;
    pRefN[1] = Fus_MagRefN.Y;
    pRefN[2] = Fus_MagRefN.Z;
}

/**
 * @name    App_Fusion_GetGyroCount
 * @brief   How many gyro samples the startup window has taken.
 * @param   None
 * @retval  The count, in samples, of the window that produces the reference.
 * @note    The number the console's pending line reports against
 *          APP_FUSION_BIAS_SAMPLES. It holds its last value once the reference
 *          has been taken, and a restart that finds the part moving empties it,
 *          so it is only a progress figure while HAVE_ATTITUDE is clear.
 */
uint32_t App_Fusion_GetGyroCount(void)
{
    return Fus_CountGyro;
}
