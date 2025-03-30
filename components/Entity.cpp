#include "RobotEngine.hpp"

Entity::Entity(EntityType _type, EntityClassType _classType, RobotEngine* _pRobotEngine) : 
    m_pOwner{_pRobotEngine},
    m_EntityType{_type},
    m_LastUpdateTick{0},
    m_TicksToUpdate{0},
    m_DefaultTicksToUpdate{1},
    m_ClassType{_classType},
    m_IsActive{false},
    m_NeedUpdate{false}
{
    m_pOwner->AddEntity(this, m_EntityType);
}

void Entity::Init()
{

}

void Entity::Update()
{
    if(!m_IsActive)
    {
        return;
    }

    if(m_NeedUpdate)
    {
        return;
    }

    if(m_TicksToUpdate > 0)
    {
        --m_TicksToUpdate;
        m_LastUpdateTick = m_pOwner->GetTick();
    }

    if(0 == m_TicksToUpdate)
    {
        m_NeedUpdate = true;
    }
}

void Entity::ClearUpdateFlag()
{
    m_TicksToUpdate = m_DefaultTicksToUpdate;
    m_NeedUpdate = false;
}

void Entity::Suspend(uint32_t _ticks)
{
    m_TicksToUpdate = _ticks;
    m_NeedUpdate = false;
}

SensorEntity::SensorEntity(EntityClassType _classType) :
    Entity(ET_SENSOR, _classType, RobotEngine::Instance()),
    m_CanTimeout(false),
    m_LastUpdateTick(0),
    m_TimeoutTick(100)
{}

bool SensorEntity::IsTimeout()
{
    if(!m_CanTimeout)
    {
        return true;
    }

    return Time::GetTick() > m_LastUpdateTick + m_TimeoutTick;
}

ControllerEntity::ControllerEntity(EntityClassType _classType) :
    Entity(ET_CONTROLLER, _classType, RobotEngine::Instance())
{}

ActuatorEntity::ActuatorEntity(EntityClassType _classType) :
    Entity(ET_ACTUATOR, _classType, RobotEngine::Instance())
{}
