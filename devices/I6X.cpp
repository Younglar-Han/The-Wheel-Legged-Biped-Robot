#include "I6X.hpp"
#include "Time.hpp"
#include "Math.hpp"

I6X::I6X():m_LHAxis(0.0f),
             m_LVAxis(0.0f),
             m_RHAxis(0.0f),
             m_RVAxis(0.0f),
             m_JoyStickDeadzone(0.05f),
             m_LastUpdateTick(0),
             m_TimeoutTick(100)
{
}

void I6X::Enable()
{
    bsp_rc_enable();
}

void I6X::Disable()
{
    bsp_rc_disable();
}

void I6X::Restart(uint16_t dma_buf_num)
{
    bsp_rc_restart(dma_buf_num);
}

void I6X::Init()
{
    bsp_remote_control_init();
    pRcRaw = get_remote_control_raw();
    I6X::Instance()->Enable();
}

void I6X::Update()
{
    if(bsp_read_rc_update_flag() == 0x00)
    {
        return;
    }

    m_LastUpdateTick = Time::GetTick();

    this->UpdateNormalizedAxis();
    this->UpdateRcEvents();
}

RC_raw_t* I6X::GetRcRaw()
{
    return pRcRaw;
}

void I6X::UpdateNormalizedAxis()
{
    m_LHAxis = NormalizeAxis(pRcRaw->rc.ch[2]);
    m_LVAxis = NormalizeAxis(pRcRaw->rc.ch[3]);
    m_RHAxis = NormalizeAxis(pRcRaw->rc.ch[0]);
    m_RVAxis = NormalizeAxis(pRcRaw->rc.ch[1]);
}

void I6X::UpdateRcEvents()
{
    m_PrevSwState[RC_SW_L1] = m_SwState[RC_SW_L1];
    m_SwState[RC_SW_L1] = pRcRaw->rc.s[RC_SW_L1];

    m_PrevSwState[RC_SW_L2] = m_SwState[RC_SW_L2];
    m_SwState[RC_SW_L2] = pRcRaw->rc.s[RC_SW_L2];

    m_PrevSwState[RC_SW_R1] = m_SwState[RC_SW_R1];
    m_SwState[RC_SW_R1] = pRcRaw->rc.s[RC_SW_R1];

    m_PrevSwState[RC_SW_R2] = m_SwState[RC_SW_R2];
    m_SwState[RC_SW_R2] = pRcRaw->rc.s[RC_SW_R2];
}

float I6X::NormalizeAxis(int16_t ch)
{
    float normalizedAxis = Math::FloatConstrain((float)ch / (float)RC_CH_OFFSET_MAX, -1.0f, 1.0f);
    if(normalizedAxis < m_JoyStickDeadzone && normalizedAxis > -m_JoyStickDeadzone){
        normalizedAxis = 0.0f;
    }
    return normalizedAxis;
}

bool I6X::QuerySwState(RcSwType target, RcSwStatusType queryType)
{
    switch (queryType)
    {
    case RC_SW_U2M:
        return ((m_PrevSwState[target] == RC_SW_UP) && (m_SwState[target] == RC_SW_MID));

    case RC_SW_M2D:
        return ((m_PrevSwState[target] == RC_SW_MID) && (m_SwState[target] == RC_SW_DOWN));

    case RC_SW_D2M:
        return ((m_PrevSwState[target] == RC_SW_DOWN) && (m_SwState[target] == RC_SW_MID));

    case RC_SW_M2U:
        return ((m_PrevSwState[target] == RC_SW_MID) && (m_SwState[target] == RC_SW_UP));
    
    case RC_SW_U2D:
        return ((m_PrevSwState[target] == RC_SW_UP) && (m_SwState[target] == RC_SW_DOWN));

    case RC_SW_D2U:
        return ((m_PrevSwState[target] == RC_SW_DOWN) && (m_SwState[target] == RC_SW_UP));

    default:
        return (m_SwState[target] == queryType);
    }
}

bool I6X::IsTimeout()
{
    return Time::GetTick() > m_TimeoutTick + m_LastUpdateTick;
}
