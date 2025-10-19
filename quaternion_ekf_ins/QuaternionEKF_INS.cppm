/**
 * @file QuaternionEKF_INS.cppm
 * @author DuYicheng
 * @date 2025-10-18
 * @brief 姿态解算模块 - 基于四元数扩展卡尔曼滤波的姿态更新算法
 *（Wang Hongxi, https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example）
 */

module;

#include <cstdint>
#include <cmath>

#include "arm_math.h"

#include "kalman_filter.h"
#include "emdevif/fatal_handler.h"

export module rmdev.ins:quaternionEkfIns;
import :base;

import emdevif.errorHandler;
import rmdev.deviceModel.sensor.imu;
import emdevif.timeline;

export namespace rmdev {

template<typename T>
concept QuaternionEkfInsScaleType = std::is_same_v<T, float>;  // 暂时只支持 float 类型

template<QuaternionEkfInsScaleType T>
class Ins<T, InsAlgorithm::QuaternionEKF>
{
public:
    using ScaleType = T;

private:
    struct INS_t {
        ScaleType q[4];              // 四元数估计值

        ScaleType Gyro[3];           // 角速度 数组下标对应取宏 X、Y、Z
        ScaleType Accel[3];          // 加速度
        ScaleType MotionAccel_b[3];  // 机体坐标加速度
        ScaleType MotionAccel_n[3];  // 绝对系加速度

        ScaleType AccelLPF;          // 加速度低通滤波系数

        // 加速度在绝对系的向量表示
        ScaleType xn[3];
        ScaleType yn[3];
        ScaleType zn[3];

        ScaleType atanxz;
        ScaleType atanyz;
    } INS{};

    struct QEKF_INS_t {
        uint8_t Initialized;
        KalmanFilter_t IMU_QuaternionEKF;
        uint8_t ConvergeFlag;
        uint8_t StableFlag;
        uint64_t ErrorCount;
        uint64_t UpdateCount;

        ScaleType q[4];         // 四元数估计值
        ScaleType GyroBias[3];  // 陀螺仪零偏估计值

        ScaleType Gyro[3];
        ScaleType Accel[3];

        ScaleType OrientationCosine[3];

        ScaleType accLPFcoef;
        ScaleType gyro_norm;
        ScaleType accl_norm;
        ScaleType AdaptiveGainScale;

        ScaleType Roll;
        ScaleType Pitch;
        ScaleType Yaw;

        ScaleType YawTotalAngle;

        ScaleType Q1;                      // 四元数更新过程噪声
        ScaleType Q2;                      // 陀螺仪零偏过程噪声
        ScaleType R;                       // 加速度计量测噪声

        ScaleType dt;                      // 姿态更新周期
        ScaleType ChiSquare;               // 卡方检验检测函数
        ScaleType ChiSquareTestThreshold;  // 卡方检验阈值
        ScaleType lambda;                  // 渐消因子

        int16_t YawRoundCount;

        ScaleType YawAngleLast;
    } QEKF_INS{};

    /**
     * @brief 用于修正安装误差的参数,demo中可无视
     *
     */
    struct IMU_Param_t {
        uint8_t flag;

        float scale[3];

        float Yaw;
        float Pitch;
        float Roll;
    } IMU_Param{};

    emdevif::Duration<ScaleType> duration_{};

    static constexpr auto X = 0, Y = 1, Z = 2;

public:
    void insUpdate(rmdev::Imu<ScaleType>& current_imu_data);

    void init();

private:
    float imuParamCorrection_lastYawOffset{};
    float imuParamCorrection_lastPitchOffset{};
    float imuParamCorrection_lastRollOffset{};

    float c_11{}, c_12{}, c_13{}, c_21{}, c_22{}, c_23{}, c_31{}, c_32{}, c_33{};

    /**
     * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
     *
     *
     * @param param IMU参数
     * @param gyro  角速度
     * @param accel 加速度
     */
    void IMU_Param_Correction(IMU_Param_t* param, float gyro[3], float accel[3]);

    /**
     * @brief          Transform 3dvector from BodyFrame to EarthFrame
     * @param[1]       vector in BodyFrame
     * @param[2]       vector in EarthFrame
     * @param[3]       quaternion
     */
    static void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q);

    /**
     * @brief          Transform 3dvector from EarthFrame to BodyFrame
     * @param[1]       vector in EarthFrame
     * @param[2]       vector in BodyFrame
     * @param[3]       quaternion
     */
    static void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q);

private:
    /* clang-format off */
    static constexpr float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                                      0, 1, 0, 0, 0, 0,
                                                      0, 0, 1, 0, 0, 0,
                                                      0, 0, 0, 1, 0, 0,
                                                      0, 0, 0, 0, 1, 0,
                                                      0, 0, 0, 0, 0, 1};

    float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                     0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                     0.1, 0.1, 0.1, 0.1, 0.1, 100};
    /* clang-format on */

    float IMU_QuaternionEKF_K[18]{};
    float IMU_QuaternionEKF_H[18]{};

    /**
     * @brief Quaternion EKF initialization and some reference value
     * @param[in] process_noise1 quaternion process noise    10
     * @param[in] process_noise2 gyro bias process noise     0.001
     * @param[in] measure_noise  accel measure noise         1000000
     * @param[in] lambda         fading coefficient          0.9996
     * @param[in] lpf            lowpass filter coefficient  0
     */
    void IMU_QuaternionEKF_Init(float process_noise1,
                                float process_noise2,
                                float measure_noise,
                                float lambda,
                                float lpf);

    /**
     * @brief Quaternion EKF update
     * @param[in]       gyro x y z in rad/s
     * @param[in]       accel x y z in m/s²
     * @param[in]       update period in s
     */
    void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

private:
    /**
     * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
     *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
     *
     * @param kf
     */
    static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t* kf);

    /**
     * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
     *
     * @param kf
     */
    static void IMU_QuaternionEKF_SetH(KalmanFilter_t* kf);

    /**
     * @brief 利用观测值和先验估计得到最优的后验估计
     *        加入了卡方检验以判断融合加速度的条件是否满足
     *        同时引入发散保护保证恶劣工况下的必要量测更新
     *
     * @param kf
     */
    static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t* kf);

    /**
     * @brief EKF观测环节,其实就是把数据复制一下
     *
     * @param kf kf类型定义
     */
    static void IMU_QuaternionEKF_Observe(KalmanFilter_t* kf);

    static float invSqrt(float x)
    {
        arm_sqrt_f32(x, &x);
        return 1.0f / x;
    }
};

/////////////////////////////////////////////////

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::insUpdate(rmdev::Imu<ScaleType>& current_imu_data)
{
    constexpr float xb[3] = {1, 0, 0};
    constexpr float yb[3] = {0, 1, 0};
    constexpr float zb[3] = {0, 0, 1};

    constexpr float gravity[3] = {0, 0, 9.81f};
    ScaleType dt = duration_.getSecondsDuration();

    INS.Accel[X] = current_imu_data.accel[X];
    INS.Accel[Y] = current_imu_data.accel[Y];
    INS.Accel[Z] = current_imu_data.accel[Z];
    INS.Gyro[X] = current_imu_data.gyro[X];
    INS.Gyro[Y] = current_imu_data.gyro[Y];
    INS.Gyro[Z] = current_imu_data.gyro[Z];

    // demo function,用于修正安装误差,可以不管,本demo暂时没用
    IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
    INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++)  // 同样过一个低通滤波
    {
        INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
                               INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);  // 转换回导航系n

    // 获取最终数据
    current_imu_data.yaw = QEKF_INS.Yaw;
    current_imu_data.pitch = QEKF_INS.Pitch;
    current_imu_data.roll = QEKF_INS.Roll;
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::init()
{
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);

    INS.AccelLPF = 0.0085;
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::IMU_Param_Correction(IMU_Param_t* param, float gyro[3], float accel[3])
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - imuParamCorrection_lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - imuParamCorrection_lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - imuParamCorrection_lastRollOffset) > 0.001f || param->flag) {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++) {
        gyro_temp[i] = gyro[i] * param->scale[i];
    }

    gyro[X] = c_11 * gyro_temp[X] + c_12 * gyro_temp[Y] + c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] + c_22 * gyro_temp[Y] + c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] + c_32 * gyro_temp[Y] + c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++) {
        accel_temp[i] = accel[i];
    }

    accel[X] = c_11 * accel_temp[X] + c_12 * accel_temp[Y] + c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] + c_22 * accel_temp[Y] + c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] + c_32 * accel_temp[Y] + c_33 * accel_temp[Z];

    imuParamCorrection_lastYawOffset = param->Yaw;
    imuParamCorrection_lastPitchOffset = param->Pitch;
    imuParamCorrection_lastRollOffset = param->Roll;
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::IMU_QuaternionEKF_Init(float process_noise1,
                                                                 float process_noise2,
                                                                 float measure_noise,
                                                                 float lambda,
                                                                 float lpf)
{
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 0.1;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    if (lambda > 1) {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    // 初始化矩阵维度信息
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);

    // 姿态初始化
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;
    QEKF_INS.IMU_QuaternionEKF.user_external_arg = this;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = true;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = true;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));

    duration_.update();
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::IMU_QuaternionEKF_Update(float gx,
                                                                   float gy,
                                                                   float gz,
                                                                   float ax,
                                                                   float ay,
                                                                   float az,
                                                                   float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    float halfgxdt, halfgydt, halfgzdt;
    float accelInvNorm;
    if (!QEKF_INS.Initialized) {
        IMU_QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    QEKF_INS.dt = dt;

    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (QEKF_INS.UpdateCount == 0)  // 如果是第一次进入,需要初始化低通滤波
    {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) +
                        ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) +
                        ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) +
                        az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    // set z,单位化重力加速度向量
    accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] +
                           QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    for (uint8_t i = 0; i < 3; i++) {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm;  // 用加速度向量更新量测值
    }

    // get body state
    QEKF_INS.gyro_norm = 1.0f / invSqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] + QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                        QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
    QEKF_INS.accl_norm = 1.0f / accelInvNorm;

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (QEKF_INS.gyro_norm < 0.3f && QEKF_INS.accl_norm > 9.8f - 0.5f && QEKF_INS.accl_norm < 9.8f + 0.5f) {
        QEKF_INS.StableFlag = 1;
    }
    else {
        QEKF_INS.StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

    // 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    // 获取融合后的数据,包括四元数和xy零飘值
    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0;  // 大部分时候z轴通天,无法观测yaw的漂移

    // 利用四元数反解欧拉角
    QEKF_INS.Yaw = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]),
                          2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f) *
                   57.295779513f;
    QEKF_INS.Pitch = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]),
                            2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f) *
                     57.295779513f;
    QEKF_INS.Roll = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) * 57.295779513f;

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.UpdateCount++;  // 初始化低通滤波用,计数测试用
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t* kf)
{
    auto QEKF_INS = static_cast<Ins<T, InsAlgorithm::QuaternionEKF>*>(kf->user_external_arg)->QEKF_INS;

    float q0, q1, q2, q3;
    float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++) {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * QEKF_INS.dt / 2;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    // 限幅,防止发散
    if (kf->P_data[28] > 10000) {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000) {
        kf->P_data[35] = 10000;
    }
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::IMU_QuaternionEKF_SetH(KalmanFilter_t* kf)
{
    float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof(ScaleType) * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t* kf)
{
    auto QEKF_INS = static_cast<Ins<T, InsAlgorithm::QuaternionEKF>*>(kf->user_external_arg)->QEKF_INS;

    float q0, q1, q2, q3;

    kf->MatStatus = KALMAN_FILTER_Matrix_Transpose(&kf->H, &kf->HT);  // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = KALMAN_FILTER_Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);  // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus =
        KALMAN_FILTER_Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);  // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = KALMAN_FILTER_Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);  // S = H P'(k) HT + R
    kf->MatStatus = KALMAN_FILTER_Matrix_Inverse(&kf->S, &kf->temp_matrix1);  // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;  // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; i++) {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 利用加速度计数据修正
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = KALMAN_FILTER_Matrix_Subtract(&kf->z,
                                                  &kf->temp_vector,
                                                  &kf->temp_vector1);  // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus =
        KALMAN_FILTER_Matrix_Multiply(&kf->temp_matrix1,
                                      &kf->temp_vector1,
                                      &kf->temp_matrix);  // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus =
        KALMAN_FILTER_Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector);  // temp_vector = z(k) - h(xhat'(k))'
    kf->temp_matrix.numRows = 1;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = KALMAN_FILTER_Matrix_Multiply(&kf->temp_vector, &kf->temp_vector1, &kf->temp_matrix);

    QEKF_INS.ChiSquare = kf->temp_matrix.pData[0];  // e(k)'*inv(D(k))*e(k) , scalar
    // rk is small,filter converged/converging
    if (QEKF_INS.ChiSquare < 0.5f * QEKF_INS.ChiSquareTestThreshold) {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (QEKF_INS.ChiSquare > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
        if (QEKF_INS.StableFlag) {
            QEKF_INS.ErrorCount++;  // 载体静止时仍无法通过卡方检验
        }
        else {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50) {
            // 滤波器发散
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = false;  // step-5 is cov mat P updating
        }
        else {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(ScaleType) * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof(ScaleType) * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = true;  // part5 is P updating
            return;
        }
    }
    else  // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (QEKF_INS.ChiSquare > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
            QEKF_INS.AdaptiveGainScale =
                (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = false;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = KALMAN_FILTER_Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);  // temp_matrix = P'(k)·HT
    kf->MatStatus = KALMAN_FILTER_Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++) {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f;  // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = KALMAN_FILTER_Matrix_Multiply(&kf->K,
                                                  &kf->temp_vector1,
                                                  &kf->temp_vector);  // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (QEKF_INS.ConvergeFlag) {
        for (uint8_t i = 4; i < 6; i++) {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt) {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt) {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = KALMAN_FILTER_Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

template<QuaternionEkfInsScaleType T>
void Ins<T, InsAlgorithm::QuaternionEKF>::IMU_QuaternionEKF_Observe(KalmanFilter_t* kf)
{
    auto self = static_cast<Ins<T, InsAlgorithm::QuaternionEKF>*>(kf->user_external_arg);

    memcpy(self->IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(self->IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(self->IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

}  // namespace rmdev
