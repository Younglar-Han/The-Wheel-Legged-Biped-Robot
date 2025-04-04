#ifndef OBSERVER_HPP
#define OBSERVER_HPP

#include "stm32f4xx.h"

#define MAX_OBSERVER 64

class Observer
{
public:
    virtual void OnNotify(void* _param) = 0;
};

class Notifier
{
private:
    Observer* m_pObserver[MAX_OBSERVER];
    uint8_t m_ObserverNum;
protected:
    void NotifyAll(void* _param)
    {
        for (int i = 0; i < m_ObserverNum; ++i)
        {
            if(m_pObserver[i] != nullptr)
            {
                m_pObserver[i]->OnNotify(_param);
            }
        }
    }
public:
    Notifier():m_pObserver{nullptr},m_ObserverNum(0){}

    void AddObserver(Observer* _observer)
    {
        if(m_ObserverNum >= MAX_OBSERVER || _observer == nullptr)
        {
            return;
        }
        for (int i = 0; i < m_ObserverNum; ++i)
        {
            if(m_pObserver[i] == _observer)
            {
                return;
            }
        }

        m_pObserver[m_ObserverNum] = _observer;
        ++m_ObserverNum;        
    }

    void RemoveObserver(Observer* _observer )
    {
        if(m_ObserverNum == 0 || _observer == nullptr)
        {
            return;
        }

        for (int i = 0; i < m_ObserverNum; ++i)
        {
            if(m_pObserver[i] == _observer)
            {
                --m_ObserverNum;
                m_pObserver[i] = m_pObserver[m_ObserverNum];
                m_pObserver[m_ObserverNum] = nullptr;
                return;
            }
        }
    }
};


#endif
