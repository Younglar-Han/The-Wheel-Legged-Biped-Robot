#ifndef WHEELCONTROLLER_HPP
#define WHEELCONTROLLER_HPP

// TODO: 修改IMU位置
#define CENTER_IMU_L 0.12f //-142.96mm x left11.55mm y -90.6mm z
#define CENTER_IMU_W 0.0f
#define CENTER_IMU_H -0.03f
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

float LQR_K[4][10] = {{-2.5376,-4.1477,-0.7150,-0.7818,-13.0353,-1.6123,-5.7977,-0.6321,-13.1250,-1.9802},
{-2.5376,-4.1477,0.7150,0.7818,-5.7977,-0.6321,-13.0353,-1.6123,-13.1250,-1.9802},
{4.2195,6.6594,-1.5633,-1.7548,23.5230,2.9104,0.9828,0.1867,-35.2153,-3.3920},
{4.2195,6.6594,1.5633,1.7548,0.9828,0.1867,23.5230,2.9104,-35.2153,-3.3920}};
float A_matrix[4][10] = {{0.0000,1.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000,-7.1449,0.0000,-7.1449,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,1.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000,-2.3860,0.0000,2.3860,0.0000,0.0000,0.0000}};
float B_matrix[4][4] = {{0.0000,0.0000,0.0000,0.0000},
{4.8928,4.8928,-1.6755,-1.6755},
{0.0000,0.0000,0.0000,0.0000},
{-5.1877,5.1877,-0.3984,0.3984}};

float constSet[6][4][10] = {{{-0.0791,-0.2167,-0.0426,-0.0463,0.0391,-0.0399,0.2440,-0.0112,-0.7160,-0.1265},
{-0.0721,-0.2031,0.0422,0.0459,0.2366,-0.0118,0.0231,-0.0404,-0.7103,-0.1241},
{0.1761,0.4012,-0.0389,-0.0420,0.5867,0.1383,-0.4577,-0.0223,-1.1262,-0.0743},
{0.1466,0.3518,0.0391,0.0421,-0.4781,-0.0275,0.5638,0.1350,-1.1379,-0.0800}},
{{-46.6432,-72.4487,0.4870,0.6570,-157.6258,-14.7215,-27.5618,-6.9848,-71.2987,-13.7694},
{27.2613,42.1904,5.4731,6.0873,-16.0021,4.0925,51.8874,3.2815,-38.2314,-2.1355},
{68.2930,107.0363,-8.1589,-9.2645,198.9060,18.6532,37.0932,7.9410,-166.6543,-11.6384},
{-35.4151,-56.6274,4.2213,4.5840,-11.6076,-5.1303,-11.0448,2.7727,-88.4919,-13.0381}},
{{81.8395,120.4384,-1.1101,-1.0530,258.7638,14.9278,71.3077,12.8417,149.6604,25.4031},
{-55.7660,-92.1185,-7.4070,-8.0822,21.6870,-15.3726,-97.4133,-9.3637,54.5288,0.7909},
{-143.3542,-217.9516,12.5853,14.1301,-387.4513,-29.5438,-117.5694,-17.6761,147.0312,2.5251},
{74.0824,121.3932,-6.3448,-7.0850,-8.7754,11.7116,46.7540,1.2223,133.2521,21.6586}},
{{27.0574,41.2493,-6.1620,-6.8701,55.0530,3.3798,-14.2471,3.7158,-47.5569,-3.6163},
{-47.1493,-73.9857,0.2283,0.1485,-25.4616,-7.3263,-153.5885,-14.6056,-80.8502,-15.3521},
{-32.1141,-50.5541,-4.5411,-4.8781,-2.3540,4.2751,-14.4721,-4.8599,-101.7052,-13.8506},
{72.9332,115.3456,8.4706,9.5551,35.3065,8.4369,208.5731,20.3131,-179.3615,-12.1910}},
{{-56.1182,-91.4817,8.9450,9.8749,-105.0104,-10.2295,18.6173,-14.4620,75.3112,4.0115},
{84.1531,126.1080,-0.5202,-0.7399,67.5406,13.7274,249.6259,15.3449,171.1043,29.0906},
{68.3692,110.2510,7.0533,7.6923,29.9927,-1.5560,1.4196,11.3644,162.9162,23.5965},
{-156.1181,-240.2812,-13.2338,-14.7643,-119.8147,-19.3112,-411.3965,-33.8442,174.5542,3.4525}},
{{9.7769,28.7451,4.0208,4.2062,3.7683,10.4949,-43.2109,2.0412,58.9697,11.5567},
{8.0627,25.6292,-4.0329,-4.3040,-43.8806,1.9112,1.7023,9.0137,59.1341,11.4580},
{-8.4563,-25.0999,3.8465,3.7847,-32.2052,-13.1071,36.3581,-4.5067,150.3896,14.3511},
{-6.2060,-21.9826,-3.8776,-3.7405,45.4709,-4.0075,-28.2336,-12.0823,150.6869,14.3935}}};
float AconstSet[6][4][10] = {{{-0.0791,-0.2167,-0.0426,-0.0463,0.0391,-0.0399,0.2440,-0.0112,-0.7160,-0.1265},
{-0.0721,-0.2031,0.0422,0.0459,0.2366,-0.0118,0.0231,-0.0404,-0.7103,-0.1241},
{0.1761,0.4012,-0.0389,-0.0420,0.5867,0.1383,-0.4577,-0.0223,-1.1262,-0.0743},
{0.1466,0.3518,0.0391,0.0421,-0.4781,-0.0275,0.5638,0.1350,-1.1379,-0.0800}},
{{-46.6432,-72.4487,0.4870,0.6570,-157.6258,-14.7215,-27.5618,-6.9848,-71.2987,-13.7694},
{27.2613,42.1904,5.4731,6.0873,-16.0021,4.0925,51.8874,3.2815,-38.2314,-2.1355},
{68.2930,107.0363,-8.1589,-9.2645,198.9060,18.6532,37.0932,7.9410,-166.6543,-11.6384},
{-35.4151,-56.6274,4.2213,4.5840,-11.6076,-5.1303,-11.0448,2.7727,-88.4919,-13.0381}},
{{81.8395,120.4384,-1.1101,-1.0530,258.7638,14.9278,71.3077,12.8417,149.6604,25.4031},
{-55.7660,-92.1185,-7.4070,-8.0822,21.6870,-15.3726,-97.4133,-9.3637,54.5288,0.7909},
{-143.3542,-217.9516,12.5853,14.1301,-387.4513,-29.5438,-117.5694,-17.6761,147.0312,2.5251},
{74.0824,121.3932,-6.3448,-7.0850,-8.7754,11.7116,46.7540,1.2223,133.2521,21.6586}},
{{27.0574,41.2493,-6.1620,-6.8701,55.0530,3.3798,-14.2471,3.7158,-47.5569,-3.6163},
{-47.1493,-73.9857,0.2283,0.1485,-25.4616,-7.3263,-153.5885,-14.6056,-80.8502,-15.3521},
{-32.1141,-50.5541,-4.5411,-4.8781,-2.3540,4.2751,-14.4721,-4.8599,-101.7052,-13.8506},
{72.9332,115.3456,8.4706,9.5551,35.3065,8.4369,208.5731,20.3131,-179.3615,-12.1910}},
{{-56.1182,-91.4817,8.9450,9.8749,-105.0104,-10.2295,18.6173,-14.4620,75.3112,4.0115},
{84.1531,126.1080,-0.5202,-0.7399,67.5406,13.7274,249.6259,15.3449,171.1043,29.0906},
{68.3692,110.2510,7.0533,7.6923,29.9927,-1.5560,1.4196,11.3644,162.9162,23.5965},
{-156.1181,-240.2812,-13.2338,-14.7643,-119.8147,-19.3112,-411.3965,-33.8442,174.5542,3.4525}},
{{9.7769,28.7451,4.0208,4.2062,3.7683,10.4949,-43.2109,2.0412,58.9697,11.5567},
{8.0627,25.6292,-4.0329,-4.3040,-43.8806,1.9112,1.7023,9.0137,59.1341,11.4580},
{-8.4563,-25.0999,3.8465,3.7847,-32.2052,-13.1071,36.3581,-4.5067,150.3896,14.3511},
{-6.2060,-21.9826,-3.8776,-3.7405,45.4709,-4.0075,-28.2336,-12.0823,150.6869,14.3935}}};
float BconstSet[6][4][4] = {{{-0.0791,-0.2167,-0.0426,-0.0463},
{-0.0721,-0.2031,0.0422,0.0459},
{0.1761,0.4012,-0.0389,-0.0420},
{0.1466,0.3518,0.0391,0.0421}},
{{-46.6432,-72.4487,0.4870,0.6570},
{27.2613,42.1904,5.4731,6.0873},
{68.2930,107.0363,-8.1589,-9.2645},
{-35.4151,-56.6274,4.2213,4.5840}},
{{81.8395,120.4384,-1.1101,-1.0530},
{-55.7660,-92.1185,-7.4070,-8.0822},
{-143.3542,-217.9516,12.5853,14.1301},
{74.0824,121.3932,-6.3448,-7.0850}},
{{27.0574,41.2493,-6.1620,-6.8701},
{-47.1493,-73.9857,0.2283,0.1485},
{-32.1141,-50.5541,-4.5411,-4.8781},
{72.9332,115.3456,8.4706,9.5551}},
{{-56.1182,-91.4817,8.9450,9.8749},
{84.1531,126.1080,-0.5202,-0.7399},
{68.3692,110.2510,7.0533,7.6923},
{-156.1181,-240.2812,-13.2338,-14.7643}},
{{9.7769,28.7451,4.0208,4.2062},
{8.0627,25.6292,-4.0329,-4.3040},
{-8.4563,-25.0999,3.8465,3.7847},
{-6.2060,-21.9826,-3.8776,-3.7405}}};


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
