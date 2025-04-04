#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include "State.hpp"

template<typename controller_type>
class StateMachine
{
protected:
    controller_type* m_pOwner;
    State<controller_type>* m_pCurrentState;
    State<controller_type>* m_pPreviousState;
    State<controller_type>* m_pGlobalState;

public:
    StateMachine(controller_type* m_pOwner) : m_pOwner(m_pOwner),
                                              m_pCurrentState(nullptr),
                                              m_pPreviousState(nullptr),
                                              m_pGlobalState(nullptr)
    {}

    void SetCurrentState(State<controller_type>* _state){ m_pCurrentState = _state; }
    void SetGlobalState(State<controller_type>* _state){ m_pGlobalState = _state; }
    void SetPreviousState(State<controller_type>* _state){ m_pPreviousState = _state; }

    void Update()
    {
        if(m_pGlobalState)
        {
            m_pGlobalState->Execute(m_pOwner);
        }

        if(m_pCurrentState)
        {
            m_pCurrentState->Execute(m_pOwner);
        }
    }

    void ChangeState(State<controller_type>* _state)
    {
        if(!_state)
        {
            return;
        }

        if(_state == m_pCurrentState)
        {
            return;
        }

        m_pPreviousState = m_pCurrentState;

        if(m_pPreviousState)
        {
            m_pPreviousState->Exit(m_pOwner);
        }

        m_pCurrentState = _state;

        if(m_pCurrentState)
        {
            m_pCurrentState->Enter(m_pOwner);
        }
    }

    void RevertToPreviousState(){ ChangeState(m_pPreviousState); }

    State<controller_type>* GetCurrentState() const{ return m_pCurrentState; }
    State<controller_type>* GetPreviousState() const{ return m_pPreviousState; }
    State<controller_type>* GetGlobalState() const{ return m_pGlobalState; }

    virtual void Init() = 0;
};

#endif
