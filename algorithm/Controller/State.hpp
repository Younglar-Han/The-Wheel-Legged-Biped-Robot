#ifndef STATE_HPP
#define STATE_HPP

template<typename controller_type>
class State
{
public:
    virtual void Init(controller_type* pOwner) = 0;
    virtual void Enter(controller_type* pOwner) = 0;
    virtual void Execute(controller_type* pOwner) = 0;
    virtual void Exit(controller_type* pOwner) = 0;
};

#endif
