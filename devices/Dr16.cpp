#include "Dr16.hpp"
#include "Time.hpp"

Dr16::Dr16():m_LHAxis(0.0f),
             m_LVAxis(0.0f),
             m_RHAxis(0.0f),
             m_RVAxis(0.0f),
             m_JoyStickDeadzone(0.05f),
             m_KbdState(0),
             m_PrevKbdState(0),
             m_KbdFall(0),
             m_KbdRise(0),
             m_MouseLState(0),
             m_PrevMouseLState(0),
             m_MouseRState(0),
             m_PrevMouseRState(0),
             m_LastUpdateTick(0),
             m_TimeoutTick(100)
{
}

void Dr16::Enable()
{
    bsp_rc_enable();
}

void Dr16::Disable()
{
    bsp_rc_disable();
}

void Dr16::Restart(uint16_t dma_buf_num)
{
    bsp_rc_restart(dma_buf_num);
}

void Dr16::Init()
{
    bsp_remote_control_init();
    pRcRaw = get_remote_control_raw();
    Dr16::Instance()->Enable();
}

void Dr16::Update()
{
    if(bsp_read_rc_update_flag() == 0x00)
    {
        return;
    }

    m_LastUpdateTick = Time::GetTick();

    this->UpdateNormalizedAxis();
    this->UpdateKbdEvents();
}

RC_raw_t* Dr16::GetRcRaw()
{
    return pRcRaw;
}

void Dr16::UpdateNormalizedAxis()
{
    m_LHAxis = NormalizeAxis(pRcRaw->rc.ch[2]);
    m_LVAxis = NormalizeAxis(pRcRaw->rc.ch[3]);
    m_RHAxis = NormalizeAxis(pRcRaw->rc.ch[0]);
    m_RVAxis = NormalizeAxis(pRcRaw->rc.ch[1]);
}

void Dr16::UpdateKbdEvents()
{
    m_PrevKbdState = m_KbdState;
    m_KbdState = pRcRaw->key.v;

    m_PrevMouseLState = m_MouseLState;
    m_MouseLState = pRcRaw->mouse.press_l;

    m_PrevMouseRState = m_MouseRState;
    m_MouseRState = pRcRaw->mouse.press_r;

    m_PrevSwState[RC_SW_L] = m_SwState[RC_SW_L];
    m_SwState[RC_SW_L] = pRcRaw->rc.s[RC_SW_L];

    m_PrevSwState[RC_SW_R] = m_SwState[RC_SW_R];
    m_SwState[RC_SW_R] = pRcRaw->rc.s[RC_SW_R];

    uint16_t kbdChanges = m_KbdState ^ m_PrevKbdState;

    m_KbdFall = kbdChanges & m_KbdState;
    m_KbdRise = kbdChanges & (~m_KbdState);
}

float Dr16::NormalizeAxis(int16_t ch)
{
    float normalizedAxis = (float)ch / (float)RC_CH_OFFSET_MAX;
    if(normalizedAxis < m_JoyStickDeadzone && normalizedAxis > -m_JoyStickDeadzone){
        normalizedAxis = 0.0f;
    }
    return normalizedAxis;
}

bool Dr16::QuerySwState(RcSwType target, RcSwStatusType queryType)
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
    
    default:
        return (m_SwState[target] == queryType);
    }
}

bool Dr16::QueryPcKeyState(PcKeyType target, PcKeyStateType queryType)
{
    switch (target)
    {
    case PC_M_L:
            if(KEY_DOWN == queryType){
                return (pRcRaw->mouse.press_l != 0);
            }
            else if(KEY_FALL == queryType){
                return (((m_PrevMouseLState ^ m_MouseLState) & m_MouseLState) != 0);
            }
            else{
                return (((m_PrevMouseLState ^ m_MouseLState) & (~m_MouseLState)) != 0);
            }

    case PC_M_R:
            if(KEY_DOWN == queryType){
                return (pRcRaw->mouse.press_r != 0);
            }
            else if(KEY_FALL == queryType){
                return (((m_PrevMouseRState ^ m_MouseRState) & m_MouseRState) != 0);
            }
            else{
                return (((m_PrevMouseRState ^ m_MouseRState) & (~m_MouseRState)) != 0);
            }
    
    default:
            uint16_t query_key = 0;
            if(KEY_DOWN == queryType){
                query_key = m_KbdState;
            }
            else if(KEY_FALL == queryType){
                query_key = m_KbdFall;
            }
            else if(KEY_RISE == queryType){
                query_key = m_KbdRise;
            }
    
            return ((query_key & (uint16_t(1 << target))) != 0);
    }

    
}

bool Dr16::IsTimeout()
{
    return Time::GetTick() > m_TimeoutTick + m_LastUpdateTick;
}
