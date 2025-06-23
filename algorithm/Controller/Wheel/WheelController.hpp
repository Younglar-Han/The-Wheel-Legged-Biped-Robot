#ifndef WHEELCONTROLLER_HPP
#define WHEELCONTROLLER_HPP

// TODO: 修改IMU位置
#define CENTER_IMU_L 0.0f //-142.96mm x left11.55mm y -90.6mm z
#define CENTER_IMU_W 0.0f
#define CENTER_IMU_H 0.0f
#define VEL_PROCESS_NOISE 25.0f
#define VEL_MEASURE_NOISE 800.0f
#include <math.h>
#include "RobotEngine.hpp"
#include "StateMachine.hpp"
#include "I6X.hpp"
#include "LK9025.hpp"
#include "LK9025.hpp"
#include "BoardPacket.hpp"
#include "math_first_order_filter.h"
#include "math_ramp_filter.h"
#include "AHRSEstimator.hpp"
#include "LegController.hpp"

// Forward declaration
class WheelController;

class WheelFsm : public StateMachine<WheelController>
{
public:
    WheelFsm(WheelController *_pOwner) : StateMachine<WheelController>(_pOwner) {}
    void HandleInput();

    virtual void Init();
};

class WheelController : public ControllerEntity
{
private:
    LK9025 m_WheelMotor[2];

    AHRSEstimator *m_IMU;

    LegController *m_pLegController;

    RampFilter m_RampFilter_s;
    RampFilter m_RampFilter_yaw;
    FirstOrderFilter m_s_dot_filter;
    FirstOrderFilter m_YawDotFilter;
    FirstOrderFilter m_gyro_filter[3];

    Pid chassisFollowPid;
    Pid sFollowPid;

    float LQR_K[4][10] = {{-5.8674, -9.5209, -1.1063, -1.1793, -27.5900,
                           -3.5515, -11.4722, -1.3030, -27.2686, -3.7579},
                          {-5.8674, -9.5209, 1.1063, 1.1793, -11.4722, -1.3030,
                           -27.5900, -3.5515, -27.2686, -3.7579},
                          {7.8926, 12.3997, -2.2592, -2.4401, 41.1535, 5.2980,
                           3.5938, 0.4496, -72.4427, -6.2298},
                          {7.8926, 12.3997, 2.2592, 2.4401, 3.5938, 0.4496,
                           41.1535, 5.2980, -72.4427, -6.2298}};
    float A_matrix[4][10] = {{0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                              0.0000, 0.0000, 0.0000, 0.0000},
                             {0.0000, 0.0000, 0.0000, 0.0000, -7.1449, 0.0000,
                              -7.1449, 0.0000, 0.0000, 0.0000},
                             {0.0000, 0.0000, 0.0000, 1.0000, 0.0000, 0.0000,
                              0.0000, 0.0000, 0.0000, 0.0000},
                             {0.0000, 0.0000, 0.0000, 0.0000, -2.3860, 0.0000,
                              2.3860, 0.0000, 0.0000, 0.0000}};
    float B_matrix[4][4] = {{0.0000, 0.0000, 0.0000, 0.0000},
                            {4.8928, 4.8928, -1.6755, -1.6755},
                            {0.0000, 0.0000, 0.0000, 0.0000},
                            {-5.1877, 5.1877, -0.3984, 0.3984}};

    float constSet[6][4][10] = {
        {{-0.1810, -0.4909, -0.0637, -0.0666, 0.0549, -0.0901, 0.5920, -0.0233,
          -1.5440, -0.2462},
         {-0.1669, -0.4628, 0.0627, 0.0656, 0.5693, -0.0250, 0.0191, -0.0917,
          -1.5333, -0.2414},
         {0.3406, 0.7639, -0.0642, -0.0685, 1.2075, 0.2612, -0.8244, -0.0292,
          -2.3593, -0.1371},
         {0.2855, 0.6720, 0.0647, 0.0689, -0.8566, -0.0378, 1.1673, 0.2557,
          -2.3727, -0.1469}},
        {{-109.8225, -168.5584, 1.7790, 2.5836, -333.5910, -32.3885, -62.1596,
          -15.6501, -142.8414, -27.3395},
         {65.4478, 99.6943, 9.3071, 10.5071, -23.3665, 9.9930, 108.5746, 6.6843,
          -84.2061, -2.7154},
         {126.1914, 198.4500, -12.3768, -14.0540, 342.3159, 34.0387, 73.5924,
          15.4649, -350.5252, -20.9157},
         {-65.4111, -105.7213, 5.5907, 5.3966, -17.5763, -10.9600, -12.1650,
          6.0597, -178.1566, -24.7211}},
        {{192.8945, 281.5381, -2.4857, -3.0356, 555.8044, 34.9798, 161.6069,
          29.2486, 299.4144, 50.1217},
         {-128.6185, -208.9297, -13.1303, -14.7928, 47.1665, -32.3267,
          -203.7640, -19.4989, 117.7238, -1.0958},
         {-270.1190, -410.9950, 18.5193, 20.8343, -681.2187, -57.0009,
          -225.8117, -34.2365, 345.4355, 6.0391},
         {139.2021, 228.3492, -8.3633, -8.3014, -17.0715, 26.2056, 75.2466,
          1.0548, 262.8653, 40.2089}},
        {{64.4188, 96.6407, -10.4334, -11.7743, 114.0157, 6.7348, -20.0343,
          9.0882, -104.2961, -5.6219},
         {-111.4552, -172.8492, -0.5960, -1.2662, -57.7801, -16.4596, -326.2048,
          -32.2826, -163.3503, -30.4580},
         {-58.7895, -93.6823, -6.1005, -5.8423, 6.2142, 8.9519, -22.2392,
          -10.1855, -206.4190, -26.2188},
         {135.3646, 214.7205, 12.8580, 14.4785, 70.7554, 16.6175, 362.4969,
          37.2022, -378.2071, -21.9672}},
        {{-128.1970, -205.5838, 15.6540, 17.7645, -216.5707, -21.0200, 41.8975,
          -30.3138, 162.5906, 5.1993},
         {199.4128, 296.5342, -0.2414, 0.1176, 153.7803, 31.4803, 538.7108,
          36.2460, 345.3348, 57.4455},
         {128.6579, 207.6287, 9.5517, 9.2557, 40.2643, -4.1228, 1.6306, 25.2371,
          326.1393, 43.8434},
         {-296.3497, -456.1795, -19.4361, -21.6845, -232.2466, -38.0415,
          -731.9982, -65.4211, 405.8746, 7.7599}},
        {{18.6643, 58.8761, 4.5678, 4.0412, 9.8503, 23.2912, -117.2192, 1.1306,
          131.4872, 22.6815},
         {14.3868, 51.0868, -4.5866, -4.3146, -118.7551, 0.4757, 6.1832,
          20.0058, 131.9757, 22.4137},
         {-14.6104, -43.9439, 6.7901, 7.0058, -76.4230, -26.5468, 76.1012,
          -6.5183, 304.2903, 26.1675},
         {-7.8897, -34.4854, -6.9665, -7.0305, 95.9200, -4.9442, -66.3838,
          -24.1103, 305.0535, 26.4332}}};
    float AconstSet[6][4][10] = {
        {{-0.1810, -0.4909, -0.0637, -0.0666, 0.0549, -0.0901, 0.5920, -0.0233,
          -1.5440, -0.2462},
         {-0.1669, -0.4628, 0.0627, 0.0656, 0.5693, -0.0250, 0.0191, -0.0917,
          -1.5333, -0.2414},
         {0.3406, 0.7639, -0.0642, -0.0685, 1.2075, 0.2612, -0.8244, -0.0292,
          -2.3593, -0.1371},
         {0.2855, 0.6720, 0.0647, 0.0689, -0.8566, -0.0378, 1.1673, 0.2557,
          -2.3727, -0.1469}},
        {{-109.8225, -168.5584, 1.7790, 2.5836, -333.5910, -32.3885, -62.1596,
          -15.6501, -142.8414, -27.3395},
         {65.4478, 99.6943, 9.3071, 10.5071, -23.3665, 9.9930, 108.5746, 6.6843,
          -84.2061, -2.7154},
         {126.1914, 198.4500, -12.3768, -14.0540, 342.3159, 34.0387, 73.5924,
          15.4649, -350.5252, -20.9157},
         {-65.4111, -105.7213, 5.5907, 5.3966, -17.5763, -10.9600, -12.1650,
          6.0597, -178.1566, -24.7211}},
        {{192.8945, 281.5381, -2.4857, -3.0356, 555.8044, 34.9798, 161.6069,
          29.2486, 299.4144, 50.1217},
         {-128.6185, -208.9297, -13.1303, -14.7928, 47.1665, -32.3267,
          -203.7640, -19.4989, 117.7238, -1.0958},
         {-270.1190, -410.9950, 18.5193, 20.8343, -681.2187, -57.0009,
          -225.8117, -34.2365, 345.4355, 6.0391},
         {139.2021, 228.3492, -8.3633, -8.3014, -17.0715, 26.2056, 75.2466,
          1.0548, 262.8653, 40.2089}},
        {{64.4188, 96.6407, -10.4334, -11.7743, 114.0157, 6.7348, -20.0343,
          9.0882, -104.2961, -5.6219},
         {-111.4552, -172.8492, -0.5960, -1.2662, -57.7801, -16.4596, -326.2048,
          -32.2826, -163.3503, -30.4580},
         {-58.7895, -93.6823, -6.1005, -5.8423, 6.2142, 8.9519, -22.2392,
          -10.1855, -206.4190, -26.2188},
         {135.3646, 214.7205, 12.8580, 14.4785, 70.7554, 16.6175, 362.4969,
          37.2022, -378.2071, -21.9672}},
        {{-128.1970, -205.5838, 15.6540, 17.7645, -216.5707, -21.0200, 41.8975,
          -30.3138, 162.5906, 5.1993},
         {199.4128, 296.5342, -0.2414, 0.1176, 153.7803, 31.4803, 538.7108,
          36.2460, 345.3348, 57.4455},
         {128.6579, 207.6287, 9.5517, 9.2557, 40.2643, -4.1228, 1.6306, 25.2371,
          326.1393, 43.8434},
         {-296.3497, -456.1795, -19.4361, -21.6845, -232.2466, -38.0415,
          -731.9982, -65.4211, 405.8746, 7.7599}},
        {{18.6643, 58.8761, 4.5678, 4.0412, 9.8503, 23.2912, -117.2192, 1.1306,
          131.4872, 22.6815},
         {14.3868, 51.0868, -4.5866, -4.3146, -118.7551, 0.4757, 6.1832,
          20.0058, 131.9757, 22.4137},
         {-14.6104, -43.9439, 6.7901, 7.0058, -76.4230, -26.5468, 76.1012,
          -6.5183, 304.2903, 26.1675},
         {-7.8897, -34.4854, -6.9665, -7.0305, 95.9200, -4.9442, -66.3838,
          -24.1103, 305.0535, 26.4332}}};
    float BconstSet[6][4][4] = {{{-0.1810, -0.4909, -0.0637, -0.0666},
                                 {-0.1669, -0.4628, 0.0627, 0.0656},
                                 {0.3406, 0.7639, -0.0642, -0.0685},
                                 {0.2855, 0.6720, 0.0647, 0.0689}},
                                {{-109.8225, -168.5584, 1.7790, 2.5836},
                                 {65.4478, 99.6943, 9.3071, 10.5071},
                                 {126.1914, 198.4500, -12.3768, -14.0540},
                                 {-65.4111, -105.7213, 5.5907, 5.3966}},
                                {{192.8945, 281.5381, -2.4857, -3.0356},
                                 {-128.6185, -208.9297, -13.1303, -14.7928},
                                 {-270.1190, -410.9950, 18.5193, 20.8343},
                                 {139.2021, 228.3492, -8.3633, -8.3014}},
                                {{64.4188, 96.6407, -10.4334, -11.7743},
                                 {-111.4552, -172.8492, -0.5960, -1.2662},
                                 {-58.7895, -93.6823, -6.1005, -5.8423},
                                 {135.3646, 214.7205, 12.8580, 14.4785}},
                                {{-128.1970, -205.5838, 15.6540, 17.7645},
                                 {199.4128, 296.5342, -0.2414, 0.1176},
                                 {128.6579, 207.6287, 9.5517, 9.2557},
                                 {-296.3497, -456.1795, -19.4361, -21.6845}},
                                {{18.6643, 58.8761, 4.5678, 4.0412},
                                 {14.3868, 51.0868, -4.5866, -4.3146},
                                 {-14.6104, -43.9439, 6.7901, 7.0058},
                                 {-7.8897, -34.4854, -6.9665, -7.0305}}};

    float s;
    float target_s;
    float s_dot;
    float last_s_dot;
    float target_s_dot;
    float real_target_s_dot;
    float last_target_s_dot;
    float s_dot_l;
    float s_dot_r;
    float yaw;
    float target_yaw;
    float yaw_dot;
    float target_yaw_dot;
    float calc_yaw_dot;
    float theta_l;
    float theta_dot_l;
    float theta_r;
    float theta_dot_r;
    float target_theta_b;
    float cos_target_theta_b;
    float theta_b;
    float theta_dot_b;

    float length_l;
    float length_r;

    float body_spd;
    float body_wheel_spd;

    float body_acc[3];
    float origin_acc[3];
    float current_gyro[3];
    float last_gyro[3];
    float delta_gyro[3];

    float pred_s;
    float pred_s_dot;
    float pred_yaw;
    float pred_yaw_dot;
    float pred_s_dot_l;
    float pred_s_dot_r;

    float T_l_adapt;
    float T_r_adapt;

    float delta_yaw;
    float real_delta_yaw;

    float torque_l;
    float torque_r;

    float torque_leg_l;
    float torque_leg_r;

    float kp_s;
    float kd_s;
    float kp_yaw;
    float kd_yaw;
    float kp_theta;
    float kd_theta;

    bool state_vmc;
    bool state_relax;
    bool state_spd;

    bool move_flag;

    float m_ChassisPower;
    float m_cap_energy;

    float acc_imu_i;
    float s_dot_acc;
    float filtered_s_dot;
    float original_s_dot;

    float extra_acc_x_yaw;
    float extra_acc_x_pitch;
    float extra_acc_z_pitch;
    float acc_x;
    float acc_z;
    float actual_acc;
    float last_acc;
    float u;
    float k;
    float vel_prior;
    float vel_predict;
    float vel_measure;
    float vel_cov;
    float vel;
    float vel_est;

    float theta_coe;
    float yaw_coe;
    float s_coe;

    uint32_t buzzer_tick;
    uint32_t wheel1_update_period;
    uint32_t wheel2_update_period;

    uint16_t projectile_num;
    uint16_t last_projectile_num;
    uint16_t bullet_num;

    // bool power_record_flag;
    // uint8_t power_record_counter;
    // uint32_t power_record_last_tick;
    // float debug_chassis_power_limit;

public:
    enum WheelMotorType
    {
        WMT_Left = 0,
        WMT_Right,

        WMT_LENGTH
    };

    WheelFsm wheelFsm;

    const static float LK9025StandardSpeedKp;
    const static float LK9025StandardSpeedKi;
    const static float LK9025StandardSpeedKd;
    const static float LK9025StandardSpeedMaxout;
    const static float LK9025StandardSpeedIMaxout;
    const static float LK9025MaxSpeed;

    WheelController();

    virtual void Init();
    virtual void Update();

    void UpdateStateVariables();
    void SpeedEstimate();
    void CalcLQR();
    void PredictOutput();
    void CalcWheelTorque();
    void SetWheelTorque();

    // void PowerControlRec();
    // void PowerControlCalc();

    float SdotFilter(float _s_dot);
    float YawFilter(float _yaw);
    void ResetCurrentState();
    void RelaxWheel();
    void MotivateWheel();
    void WheelSpdMode();
    void SetWheelSpd(float _s_dot);

    float GetTorqueLegL() { return torque_leg_l; }
    float GetTorqueLegR() { return torque_leg_r; }
    bool GetMotorOffline(int _motor) { return m_WheelMotor[_motor].sensorFeedBack.IsTimeout(); }

    void SetDeltaYaw(float _deltaYaw) { real_delta_yaw = _deltaYaw; }
    void SetSdot(float _s_dot) { real_target_s_dot = _s_dot; }
    void SetYawDot(float _yaw_dot) { target_yaw_dot = _yaw_dot; }
    void SetPitch(float _pitch) { target_theta_b = _pitch; }
    void SetCosPitch(float _cosPitch) { cos_target_theta_b = _cosPitch; }
    void SetRelaxMode(bool _relax) { state_relax = _relax; }
    void SetVmcMode(bool _vmc) { state_vmc = _vmc; }
    void SetSpdMode(bool _spd) { state_spd = _spd; }

    float GetSdot() { return s_dot; }
    float GetBodySpd() { return body_spd; }
    uint16_t GetProjectileNum() { return bullet_num; }

    LK9025 *GetWheelMotor(WheelMotorType _motor) { return &m_WheelMotor[_motor]; }
};

#endif
