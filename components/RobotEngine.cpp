#include "RobotEngine.hpp"

RobotEngine::RobotEngine(RobotEngine* _instance, uint32_t _msPerTick) : m_EntityList{nullptr},
                                                m_EntityCount{0},
                                                m_IsRunning{false}
{
    if(!IsInstantiated)
    {
        instance =_instance;
    }

    IsInstantiated = false;
}

bool RobotEngine::IsInstantiated = false;
RobotEngine* RobotEngine::instance = nullptr;

void RobotEngine::AddEntity(Entity* _newEntity, EntityType _type)
{
    if(m_IsRunning)
    {
        return;
    }

    if(m_EntityCount[_type] >= MAX_ENTITIES)
    {
        _newEntity->SetId(0xFF);
        return;
    }

    uint8_t _id = m_EntityCount[_type];
    _newEntity->SetId(_id);
    m_EntityList[_type][_id] = _newEntity;

    ++m_EntityCount[_type];
}

Entity* RobotEngine::GetEntity(EntityClassType _classType)
{
    for(int _type = ET_SENSOR; _type < ET_LENGTH; ++_type)
    {
        for(int i = 0; i < m_EntityCount[_type]; ++i)
        {
            if(m_EntityList[_type][i]->GetClassType() == _classType)
            {
                return m_EntityList[_type][i];
            }
        }
    }
    return nullptr;
}

// Not tested yet.
void RobotEngine::GetEntity(EntityClassType _classType, uint32_t _maxLen, Entity** entityFound, uint32_t& numberFound)
{
    numberFound = 0;
    if(numberFound >= _maxLen)
    {
        return;
    }

    for(int _type = ET_SENSOR; _type < ET_LENGTH; ++_type)
    {
        for(int i = 0; i < m_EntityCount[_type]; ++i)
        {
            if(m_EntityList[_type][i]->GetClassType() == _classType)
            {
                entityFound[numberFound] = m_EntityList[_type][i];
                ++numberFound;
                if(numberFound >= _maxLen)
                {
                    return;
                }
            }
        }
    }
}

void RobotEngine::Init()
{
    for(int _type = ET_SENSOR; _type < ET_LENGTH; ++_type)
    {
        for(int i = 0; i < m_EntityCount[_type]; ++i)
        {
            m_EntityList[_type][i]->Init();
            m_EntityList[_type][i]->SetActive(true);
        }
    }

    m_IsRunning = true;
}

void RobotEngine::Tick()
{
    for(int _type = ET_SENSOR; _type < ET_LENGTH; ++_type)
    {
        for(int i = 0; i < m_EntityCount[_type]; ++i)
        {
            Entity* _entity = m_EntityList[_type][i];
            if(!_entity->IsActive())
            {
                continue;
            }

            _entity->Entity::Update();

            // Checking for SennsorEntity timeout
            if(_entity->GetEntityType() == ET_SENSOR)
            {
                SensorEntity& _sensor = *(SensorEntity*)(_entity);
                if(_sensor.CanTimeout())
                {
                    if(_sensor.HasNewData())
                    {
                        _sensor.ClearUpdateFlag();
                    }
                }
            }

            if(_entity->NeedUpdate())
            {
                _entity->Update();
                _entity->ClearUpdateFlag();
            }
        }
    }
}
