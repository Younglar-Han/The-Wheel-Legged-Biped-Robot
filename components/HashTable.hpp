#ifndef HASHTABLE_HPP
#define HASHTABLE_HPP

#include "stm32f4xx.h"

#define HASH_TABLE_LENGTH 129

template<typename T>
class HashTable
{
private:
    enum HashState
    {
        NIL = 0,
        OCUPIED,
        DELETED,
    };

    struct HashItem
    {
        uint32_t key;
        T item;
        HashState state;
    };

    HashItem m_Table[HASH_TABLE_LENGTH];
    uint8_t m_Size;

    int16_t SearchKey(uint32_t _key)
    {
        uint8_t _hash = _key % HASH_TABLE_LENGTH;

        if(_key == m_Table[_hash].key)
        {
            return _hash;
        }

        uint8_t _start = _hash;
        
        do{
            _hash = (_hash +1) % HASH_TABLE_LENGTH;
            if(_key == m_Table[_hash].key)
            {
                return _hash;
            }
        }while(m_Table[_hash].state != NIL && _hash != _start);

        return -1;
    }
public:
    HashTable()
    {
        for(int i = 0; i < HASH_TABLE_LENGTH; ++i)
        {
            m_Table[i].state = NIL;
        }
        m_Size = 0;
    }

    int Insert(uint32_t _key, T _item)
    {
        if(m_Size >= HASH_TABLE_LENGTH)
        {
            return -1;
        }

        uint8_t _hash = _key % HASH_TABLE_LENGTH;
        while(m_Table[_hash].state != NIL)
        {
            _hash = (_hash +1) % HASH_TABLE_LENGTH;
        }

        m_Table[_hash].state = OCUPIED;
        m_Table[_hash].item = _item;
        m_Table[_hash].key = _key;
        ++m_Size;

        return 0;        
    }

    T* Search(uint32_t _key)
    {
        int16_t _hash = SearchKey(_key);
        if(_hash == -1)
        {
            return nullptr;
        }

        return &m_Table[(uint8_t)_hash].item;
    }

    void Delete(uint32_t _key)
    {
        int16_t _hash = SearchKey(_key);
        if(_hash == -1)
        {
            return ;
        }

        m_Table[(uint8_t)_hash].state = DELETED;
    }

    void Modify(uint32_t _key, T _item)
    {
        int16_t _index = SearchKey(_key);
        if(-1 == _index)
        {
            Insert(_key, _item);
            return;
        }

        m_Table[_index].item = _item;
    }
};

#endif
