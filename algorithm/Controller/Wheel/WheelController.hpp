#ifndef WHEELCONTROLLER_HPP
#define WHEELCONTROLLER_HPP

#define FLT_EPSILON 1.192e-07F
#define CENTER_IMU_L -0.143f //-142.96mm x left11.55mm y -90.6mm z
#define CENTER_IMU_W -0.01155f
#define CENTER_IMU_H -0.0906f
#define VEL_PROCESS_NOISE 25.0f
#define VEL_MEASURE_NOISE 800.0f
#include <math.h>
#include "RobotEngine.hpp"
#include "StateMachine.hpp"
#include "Dr16.hpp"
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

    float LQR_K[4][10] = {{-4.7349, -7.2901, -0.8846, -0.9632, -23.0257, -2.7163, -9.8029, -0.9789, -35.6561, -3.9404},
                          {-4.7349, -7.2901, 0.8846, 0.9632, -9.8029, -0.9789, -23.0257, -2.7163, -35.6561, -3.9404},
                          {5.5066, 8.3254, -2.6210, -3.0022, 30.8949, 3.5806, -1.7216, 0.2243, -122.2499, -10.1125},
                          {5.5066, 8.3254, 2.6210, 3.0022, -1.7216, 0.2243, 30.8949, 3.5806, -122.2499, -10.1125}};
    float A_matrix[4][10] = {{0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000},
                             {0.0000, 0.0000, 0.0000, 0.0000, -22.1779, 0.0000, -22.1779, 0.0000, 0.0000, 0.0000},
                             {0.0000, 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000},
                             {0.0000, 0.0000, 0.0000, 0.0000, -3.2908, 0.0000, 3.2908, 0.0000, 0.0000, 0.0000}};
    float B_matrix[4][4] = {{0.0000, 0.0000, 0.0000, 0.0000},
                            {4.7641, 4.7641, -1.6751, -1.6751},
                            {0.0000, 0.0000, 0.0000, 0.0000},
                            {-3.0238, 3.0238, -0.2229, 0.2229}};

    float constSet[6][4][10] = {{{-0.1226, -0.2153, -0.0568, -0.0626, -0.4734, -0.0474, -0.1049, 0.0105, -2.8318, -0.2965},
                                 {-0.1217, -0.2127, 0.0571, 0.0630, -0.1063, 0.0103, -0.4824, -0.0481, -2.8380, -0.2960},
                                 {0.4070, 0.6636, -0.0745, -0.0806, 1.3462, 0.1792, 0.2955, 0.0237, -3.4243, -0.2536},
                                 {0.4024, 0.6556, 0.0739, 0.0796, 0.2849, 0.0229, 1.3502, 0.1799, -3.3452, -0.2483}},
                                {{-29.8261, -47.8401, 0.0558, 0.6211, -104.4928, -13.4244, -13.9805, 0.0354, -74.8185, -12.8911},
                                 {-5.2632, -7.7141, 7.6638, 9.0107, -43.1958, -5.5523, -37.6929, -4.0985, -213.0603, -19.0443},
                                 {17.3082, 29.8795, -11.1202, -12.9952, 101.1504, 11.5737, -0.6359, -0.0623, -742.5405, -62.3054},
                                 {26.5883, 37.3832, 7.7763, 8.4291, -6.7080, 2.6701, 121.3663, 15.3795, -162.5854, -12.3174}},
                                {{34.0563, 59.4078, 2.7997, 2.4634, 1.9614, 0.9647, 34.0039, 1.9408, 158.1307, 24.5606},
                                 {22.3770, 34.6551, -10.4441, -12.3779, 55.5468, 8.3112, 62.0350, 6.0073, 317.2777, 30.4166},
                                 {-46.0380, -77.9150, 15.6279, 18.4270, -144.2526, -16.6065, -13.4023, -1.0907, 960.5863, 75.5898},
                                 {-49.3403, -74.4239, -10.3396, -11.0140, 38.5920, -0.3383, -181.8218, -22.6321, 216.3907, 15.1855}},
                                {{-7.0827, -10.8000, -8.4486, -9.9018, -43.5911, -4.6119, -45.0506, -5.4812, -248.8937, -22.8153},
                                 {-31.6576, -51.0034, 0.7156, 0.2493, -15.7960, 0.1175, -109.8730, -13.8932, -110.3211, -16.6811},
                                 {32.5180, 46.9089, -8.7974, -9.5380, 139.3515, 17.6179, -2.5097, 3.0561, -208.4249, -15.6164},
                                 {23.4845, 39.8343, 12.1715, 14.1505, 4.1035, 0.3684, 118.9395, 13.7736, -792.2569, -65.8612}},
                                {{27.4963, 43.1353, 12.1889, 14.3761, 78.2188, 7.5352, 59.2829, 8.1663, 398.6081, 38.9483},
                                 {37.1435, 64.9633, -4.5327, -4.4052, 38.6171, 1.7818, 10.6986, 1.5849, 235.7381, 32.8848},
                                 {-62.0836, -94.8936, 12.6311, 13.4896, -223.2610, -27.8881, 31.2471, -0.9345, 311.5092, 22.0768},
                                 {-60.4888, -101.2192, -17.9709, -21.0186, -25.9496, -2.3100, -182.7099, -21.2244, 1078.5088, 83.9333}},
                                {{4.4515, 7.0544, 6.5930, 7.4915, 74.7150, 10.7361, -46.2191, -10.1098, 259.6010, 25.2296},
                                 {6.7120, 10.5575, -6.5506, -7.4674, -47.3261, -10.1391, 80.6799, 11.5373, 262.2935, 25.5349},
                                 {-16.7940, -22.6203, 5.6963, 5.2710, -91.2900, -14.9804, -76.3120, -8.7034, 426.8838, 37.9264},
                                 {-15.9670, -21.3293, -5.7653, -5.3361, -72.8706, -8.2085, -93.7532, -15.5184, 418.0264, 37.4002}}};
    float AconstSet[6][4][10] = {{{-0.1226, -0.2153, -0.0568, -0.0626, -0.4734, -0.0474, -0.1049, 0.0105, -2.8318, -0.2965},
                                  {-0.1217, -0.2127, 0.0571, 0.0630, -0.1063, 0.0103, -0.4824, -0.0481, -2.8380, -0.2960},
                                  {0.4070, 0.6636, -0.0745, -0.0806, 1.3462, 0.1792, 0.2955, 0.0237, -3.4243, -0.2536},
                                  {0.4024, 0.6556, 0.0739, 0.0796, 0.2849, 0.0229, 1.3502, 0.1799, -3.3452, -0.2483}},
                                 {{-29.8261, -47.8401, 0.0558, 0.6211, -104.4928, -13.4244, -13.9805, 0.0354, -74.8185, -12.8911},
                                  {-5.2632, -7.7141, 7.6638, 9.0107, -43.1958, -5.5523, -37.6929, -4.0985, -213.0603, -19.0443},
                                  {17.3082, 29.8795, -11.1202, -12.9952, 101.1504, 11.5737, -0.6359, -0.0623, -742.5405, -62.3054},
                                  {26.5883, 37.3832, 7.7763, 8.4291, -6.7080, 2.6701, 121.3663, 15.3795, -162.5854, -12.3174}},
                                 {{34.0563, 59.4078, 2.7997, 2.4634, 1.9614, 0.9647, 34.0039, 1.9408, 158.1307, 24.5606},
                                  {22.3770, 34.6551, -10.4441, -12.3779, 55.5468, 8.3112, 62.0350, 6.0073, 317.2777, 30.4166},
                                  {-46.0380, -77.9150, 15.6279, 18.4270, -144.2526, -16.6065, -13.4023, -1.0907, 960.5863, 75.5898},
                                  {-49.3403, -74.4239, -10.3396, -11.0140, 38.5920, -0.3383, -181.8218, -22.6321, 216.3907, 15.1855}},
                                 {{-7.0827, -10.8000, -8.4486, -9.9018, -43.5911, -4.6119, -45.0506, -5.4812, -248.8937, -22.8153},
                                  {-31.6576, -51.0034, 0.7156, 0.2493, -15.7960, 0.1175, -109.8730, -13.8932, -110.3211, -16.6811},
                                  {32.5180, 46.9089, -8.7974, -9.5380, 139.3515, 17.6179, -2.5097, 3.0561, -208.4249, -15.6164},
                                  {23.4845, 39.8343, 12.1715, 14.1505, 4.1035, 0.3684, 118.9395, 13.7736, -792.2569, -65.8612}},
                                 {{27.4963, 43.1353, 12.1889, 14.3761, 78.2188, 7.5352, 59.2829, 8.1663, 398.6081, 38.9483},
                                  {37.1435, 64.9633, -4.5327, -4.4052, 38.6171, 1.7818, 10.6986, 1.5849, 235.7381, 32.8848},
                                  {-62.0836, -94.8936, 12.6311, 13.4896, -223.2610, -27.8881, 31.2471, -0.9345, 311.5092, 22.0768},
                                  {-60.4888, -101.2192, -17.9709, -21.0186, -25.9496, -2.3100, -182.7099, -21.2244, 1078.5088, 83.9333}},
                                 {{4.4515, 7.0544, 6.5930, 7.4915, 74.7150, 10.7361, -46.2191, -10.1098, 259.6010, 25.2296},
                                  {6.7120, 10.5575, -6.5506, -7.4674, -47.3261, -10.1391, 80.6799, 11.5373, 262.2935, 25.5349},
                                  {-16.7940, -22.6203, 5.6963, 5.2710, -91.2900, -14.9804, -76.3120, -8.7034, 426.8838, 37.9264},
                                  {-15.9670, -21.3293, -5.7653, -5.3361, -72.8706, -8.2085, -93.7532, -15.5184, 418.0264, 37.4002}}};
    float BconstSet[6][4][4] = {{{-0.1226, -0.2153, -0.0568, -0.0626},
                                 {-0.1217, -0.2127, 0.0571, 0.0630},
                                 {0.4070, 0.6636, -0.0745, -0.0806},
                                 {0.4024, 0.6556, 0.0739, 0.0796}},
                                {{-29.8261, -47.8401, 0.0558, 0.6211},
                                 {-5.2632, -7.7141, 7.6638, 9.0107},
                                 {17.3082, 29.8795, -11.1202, -12.9952},
                                 {26.5883, 37.3832, 7.7763, 8.4291}},
                                {{34.0563, 59.4078, 2.7997, 2.4634},
                                 {22.3770, 34.6551, -10.4441, -12.3779},
                                 {-46.0380, -77.9150, 15.6279, 18.4270},
                                 {-49.3403, -74.4239, -10.3396, -11.0140}},
                                {{-7.0827, -10.8000, -8.4486, -9.9018},
                                 {-31.6576, -51.0034, 0.7156, 0.2493},
                                 {32.5180, 46.9089, -8.7974, -9.5380},
                                 {23.4845, 39.8343, 12.1715, 14.1505}},
                                {{27.4963, 43.1353, 12.1889, 14.3761},
                                 {37.1435, 64.9633, -4.5327, -4.4052},
                                 {-62.0836, -94.8936, 12.6311, 13.4896},
                                 {-60.4888, -101.2192, -17.9709, -21.0186}},
                                {{4.4515, 7.0544, 6.5930, 7.4915},
                                 {6.7120, 10.5575, -6.5506, -7.4674},
                                 {-16.7940, -22.6203, 5.6963, 5.2710},
                                 {-15.9670, -21.3293, -5.7653, -5.3361}}};

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
