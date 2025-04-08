#ifndef LEGCONTROLLER_HPP
#define LEGCONTROLLER_HPP

#include <math.h>
#include "RobotEngine.hpp"
#include "StateMachine.hpp"
#include "Dr16.hpp"
#include "HT04.hpp"
#include "BoardPacket.hpp"
#include "math_first_order_filter.h"
#include "AHRSEstimator.hpp"

// Forward declaration
class LegController;

class LegFsm : public StateMachine<LegController>
{
private:
    bool hasInited;
    bool hasInitRequest;
    bool relaxState;
    bool initState;

public:
    LegFsm(LegController *_pOwner) : StateMachine<LegController>(_pOwner) {}
    void HandleInput();

    void SetInited(bool _inited) { hasInited = _inited; }

    virtual void Init();
};

class LegController : public ControllerEntity
{
private:
    HT04 m_LegMotor[4];

    AHRSEstimator *m_IMU;

    Pid legLengthPid;
    Pid leftLegLengthPid;
    Pid rightLegLengthPid;
    Pid rollPid;

    FirstOrderFilter thetaDDotL;
    FirstOrderFilter thetaDDotR;
    FirstOrderFilter lengthDDotL;
    FirstOrderFilter lengthDDotR;

    FirstOrderFilter thetaDotL;
    FirstOrderFilter thetaDotR;
    FirstOrderFilter lengthDotL;
    FirstOrderFilter lengthDotR;

    FirstOrderFilter lengthDFilter;

    FirstOrderFilter distanceFilter;

    float F_L;
    float F_R;
    float T_L;
    float T_R;

    float length_l;
    float length_r;
    float theta_l;
    float theta_r;
    float theta1_l;
    float theta1_r;
    float theta2_l;
    float theta2_r;
    float phi0_l;
    float phi0_r;
    float theta_l_dot;
    float theta_r_dot;
    float last_theta_l;
    float last_theta_r;
    float last_theta_l_dot;
    float last_theta_r_dot;
    float length_l_dot;
    float length_r_dot;
    float last_length_l;
    float last_length_r;
    float last_length_l_dot;
    float last_length_r_dot;
    float FN_L;
    float FN_R;
    float zm;
    float theta_l_ddot;
    float theta_r_ddot;
    float length_l_ddot;
    float length_r_ddot;

    float debug_theta_l_dot_filtered;
    float debug_theta_r_dot_filtered;
    float debug_length_l_dot_filtered;
    float debug_length_r_dot_filtered;

    float debug_length_p_result;
    float debug_length_i_result;
    float debug_length_d_result;
    float debug_length_result;

    float length_d_result_filtered;

    uint32_t buzzer_tick; // 我用在vmc缓启动了，别删  -Rededge 0204
    uint32_t jump_time;
    uint32_t jump_finish;

    uint32_t init_tick;

    uint8_t jump_state;

    float Jacob_L[2][2] = {0};
    float Jacob_R[2][2] = {0};

    float inv_Jacob_L[2][2] = {0};
    float inv_Jacob_R[2][2] = {0};

    float posSet;

    float torque_lf;
    float torque_lr;
    float torque_rf;
    float torque_rr;

    float calc_F_l;
    float calc_F_r;
    float calc_T_l;
    float calc_T_r;

    float P_L;
    float P_R;

    bool l_leave_ground;
    bool r_leave_ground;

    bool relax;
    bool jumping;
    bool c_down;
    bool force_relax;

    bool last_init;
    bool has_inited;
    bool has_init_request;
    bool lose_judge;

    bool power_on;
    bool last_power_on;

    float target_height;
    float height_bias;
    float target_roll;
    float cos_target_roll;

    float compensation_l;
    float compensation_r;

    float s_dot;

    uint32_t lose_control_tick;
    uint32_t power_on_tick;

    uint16_t distance;
    uint16_t distance_filtered;
    uint32_t distance_counter;

    float spdfdb[4];

public:
    enum LegMotorType
    {
        LMT_LeftFront = 0,
        LMT_LeftRear,
        LMT_RightRear,
        LMT_RightFront,

        LMT_LENGTH
    };

    LegFsm legFsm;

    const static float HT04StandardSpeedKp;
    const static float HT04StandardSpeedKi;
    const static float HT04StandardSpeedKd;
    const static float HT04StandardSpeedMaxout;
    const static float HT04StandardSpeedIMaxout;
    const static float HT04MaxSpeed;

    const static float HT04StandardPosKp;
    const static float HT04StandardPosKi;
    const static float HT04StandardPosKd;
    const static float HT04StandardPosMaxout;
    const static float HT04StandardPosIMaxout;

    const float static HEIGHT_KP;
    const float static HEIGHT_KI;
    const float static HEIGHT_KD;
    const float static HEIGHT_MAXOUT;
    const float static HEIGHT_IMAXOUT;
    const float static HEIGHT_INIT_KP;
    const float static HEIGHT_INIT_KI;
    const float static HEIGHT_INIT_KD;
    const float static HEIGHT_INIT_MAXOUT;
    const float static HEIGHT_INIT_IMAXOUT;
    const float static ROLL_KP;
    const float static ROLL_KI;
    const float static ROLL_KD;
    const float static ROLL_MAXOUT;
    const float static ROLL_IMAXOUT;
    const float static JUMP_KP;
    const float static JUMP_KI;
    const float static JUMP_KD;
    const float static JUMP_MAXOUT;
    const float static JUMP_IMAXOUT;
    const float static LEG_LENGTH_1;
    const float static LEG_LENGTH_2;
    const float static LEG_LENGTH_3;
    const float static LEG_MID_HEIGHT;
    const float static LEG_MAX_HEIGHT;
    const float static LEG_MIN_HEIGHT;
    const float static G;

    LegController();

    virtual void Init();
    virtual void Update();

    void UpdateStateVariables();
    void CalcJacob();
    void LeaveGround();
    void CalcF();
    void CalcMotorTorque();
    void SetMotorTorque();

    void RelaxLeg();
    void MotivateLeg();
    void LegPosMode();
    void LegSpdMode();
    void LegCurMode();
    void ResetCurrentState();
    void ClearFilters();
    void JudgeLoseControl();

    void SetPos(float pos);
    void SetSpd(float spd);
    void SetCoSpd(float spd);
    void SetCur(float cur);
    void SetZeroPos();

    void SetHeight(float height) { target_height = height; }
    void SetRoll(float roll) { target_roll = roll; }
    void SetCosRoll(float cos_roll) { cos_target_roll = cos_roll; }

    void SetTorque(float tl, float tr)
    {
        T_L = tl;
        T_R = tr;
    }

    void SetCompensation(float _compensation_l, float _compensation_r)
    {
        compensation_l = _compensation_l;
        compensation_r = _compensation_r;
    }

    void SetRelax(bool _relax) { relax = _relax; }
    void SetJump(bool _jump) { jumping = _jump; }
    void SetJumpTime(uint32_t _jump_time) { jump_time = _jump_time; }
    void SetCDown(bool _c_down) { c_down = _c_down; }
    void SetForceRelax(bool _force_relax) { force_relax = _force_relax; }
    void SetLengthPID(float _kp, float _ki, float _kd, float _maxout, float _imaxout)
    {
        legLengthPid.SetParams(_kp, _ki, _kd, _maxout, _imaxout);
        leftLegLengthPid.SetParams(_kp, _ki, _kd, _maxout, _imaxout);
        rightLegLengthPid.SetParams(_kp, _ki, _kd, _maxout, _imaxout);
    }
    void SetInitLengthPID()
    {
        SetLengthPID(HEIGHT_INIT_KP, HEIGHT_INIT_KI, HEIGHT_INIT_KD, HEIGHT_INIT_MAXOUT, HEIGHT_INIT_IMAXOUT);
    }
    void SetStandardLengthPID()
    {
        SetLengthPID(HEIGHT_KP, HEIGHT_KI, HEIGHT_KD, HEIGHT_MAXOUT, HEIGHT_IMAXOUT);
    }
    void SetLoseJudge(bool _judge){ lose_judge = _judge; }
    void SetSDot(float _s_dot) { s_dot = _s_dot; }

    void InitFinished() { init_tick = Time::GetTick(); }

    bool IsLeftLeaveGround() { return l_leave_ground; }
    bool IsRightLeaveGround() { return r_leave_ground; }

    float GetLengthL() { return length_l; }
    float GetLengthR() { return length_r; }
    float GetThetaL() { return theta_l; }
    float GetThetaR() { return theta_r; }
    float GetThetaLDot() { return theta_l_dot; }
    float GetThetaRDot() { return theta_r_dot; }
    bool GetMotorOffline(int _motor) { return m_LegMotor[_motor].sensorFeedBack.IsTimeout(); }

    bool HasInitRequest() { return has_init_request; }

    HT04 *GetLegMotor(LegMotorType _motor) { return &m_LegMotor[_motor]; }
};

#endif
