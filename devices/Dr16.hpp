#ifndef DR16_HPP
#define DR16_HPP

#include "bsp_rc.h"

class Dr16
{
private:
    RC_raw_t *pRcRaw;

    /* Left Horizontal, Left Vertical, Right Horizontal, Right Vertical, normalized */
    float m_LHAxis;
    float m_LVAxis;
    float m_RHAxis;
    float m_RVAxis;
    float m_JoyStickDeadzone;

    uint16_t m_KbdState;
    uint16_t m_PrevKbdState;
    uint16_t m_KbdFall;
    uint16_t m_KbdRise;
    uint8_t m_MouseLState;
    uint8_t m_PrevMouseLState;
    uint8_t m_MouseRState;
    uint8_t m_PrevMouseRState;
    uint32_t m_LastUpdateTick;
    uint32_t m_TimeoutTick;

    char m_SwState[2];
    char m_PrevSwState[2];

    Dr16();

    void UpdateNormalizedAxis(void);
    void UpdateKbdEvents(void);
    float NormalizeAxis(int16_t ch);

public:
    enum PcKeyStateType
    {
        KEY_DOWN = 0,
        KEY_FALL,
        KEY_RISE,
    };

    enum PcKeyType
    {
        PC_KEY_W = 0,
        PC_KEY_S,
        PC_KEY_A,
        PC_KEY_D,
        PC_KEY_SHIFT,
        PC_KEY_CTRL,
        PC_KEY_Q,
        PC_KEY_E,
        PC_KEY_R,
        PC_KEY_F,
        PC_KEY_G,
        PC_KEY_Z,
        PC_KEY_X,
        PC_KEY_C,
        PC_KEY_V,
        PC_KEY_B,
        PC_M_L,
        PC_M_R,
    };

    enum RcSwStatusType
    {
        RC_SW_UP = 1,
        RC_SW_MID = 3,
        RC_SW_DOWN = 2,
        RC_SW_U2M = 4,
        RC_SW_M2D = 5,
        RC_SW_D2M = 6,
        RC_SW_M2U = 7,
    };

    enum RcSwType
    {
        RC_SW_L = 1,
        RC_SW_R = 0,
    };

    void Init();
    void Enable();
    void Disable();
    void Restart(uint16_t dma_buf_num);
    RC_raw_t *GetRcRaw();

    virtual void Update();

    float GetLHAxis() const { return m_LHAxis; }
    float GetLVAxis() const { return m_LVAxis; }
    float GetRHAxis() const { return m_RHAxis; }
    float GetRVAxis() const { return m_RVAxis; }
    int16_t GetMouseX() const { return pRcRaw->mouse.x; }
    int16_t GetMouseY() const { return pRcRaw->mouse.y; }
    int16_t GetMouseZ() const { return pRcRaw->mouse.z; }
    float GetJoyStickDeadZone() const { return m_JoyStickDeadzone; }
    void SetJoyStickDeadZone(float deadzone) { m_JoyStickDeadzone = deadzone; }
    uint32_t GetLastUpdateTick() const { return m_LastUpdateTick; }
    void SetTimeout(uint32_t _timeout) { m_TimeoutTick = _timeout; }
    uint32_t GetTimeout() const { return m_TimeoutTick; }

    bool QuerySwState(RcSwType target, RcSwStatusType queryType);
    bool QueryPcKeyState(PcKeyType target, PcKeyStateType queryType);
    bool IsTimeout();

    static Dr16 *Instance()
    {
        static Dr16 instance;
        return &instance;
    }
};

#endif
