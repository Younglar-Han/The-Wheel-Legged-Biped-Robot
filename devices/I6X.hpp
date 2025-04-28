#ifndef I6X_HPP
#define I6X_HPP

#include "bsp_rc.h"

class I6X
{
private:
    RC_raw_t *pRcRaw;

    /* Left Horizontal, Left Vertical, Right Horizontal, Right Vertical, normalized */
    float m_LHAxis;
    float m_LVAxis;
    float m_RHAxis;
    float m_RVAxis;
    float m_JoyStickDeadzone;

    uint32_t m_LastUpdateTick;
    uint32_t m_TimeoutTick;

    char m_SwState[4];
    char m_PrevSwState[4];

    I6X();

    void UpdateNormalizedAxis(void);
    void UpdateRcEvents(void);
    float NormalizeAxis(int16_t ch);

public:

    enum RcSwStatusType
    {
        RC_SW_UP = 1,
        RC_SW_MID = 2,
        RC_SW_DOWN = 3,
        RC_SW_U2M = 4,
        RC_SW_M2D = 5,
        RC_SW_D2M = 6,
        RC_SW_M2U = 7,
        RC_SW_U2D = 8,
        RC_SW_D2U = 9,
    };

    enum RcSwType
    {
        RC_SW_L1 = 0,
        RC_SW_L2,
        RC_SW_R1,
        RC_SW_R2,
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
    float GetJoyStickDeadZone() const { return m_JoyStickDeadzone; }
    void SetJoyStickDeadZone(float deadzone) { m_JoyStickDeadzone = deadzone; }
    uint32_t GetLastUpdateTick() const { return m_LastUpdateTick; }
    void SetTimeout(uint32_t _timeout) { m_TimeoutTick = _timeout; }
    uint32_t GetTimeout() const { return m_TimeoutTick; }

    bool QuerySwState(RcSwType target, RcSwStatusType queryType);
    bool IsTimeout();

    static I6X *Instance()
    {
        static I6X instance;
        return &instance;
    }
};

#endif
