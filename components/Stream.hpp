#ifndef STREAM_HPP
#define STREAM_HPP

#include "stm32f4xx.h"
#include <string.h>

// LITTLE ENDIAN FORMAT !!!! This helper is VERT UN-Professional
class ArrayStreamHelper
{
public:
    static void Write(uint8_t* &_pBuffer, uint8_t _data, bool _movePointer = true)
    {
        memcpy(_pBuffer, &_data, sizeof(uint8_t));

        if(_movePointer)
        {
            _pBuffer += sizeof(uint8_t);
        }
    }

    static void Write(uint8_t* &_pBuffer, uint16_t _data, bool _movePointer = true)
    {
        memcpy(_pBuffer, &_data, sizeof(uint16_t));

        if(_movePointer)
        {
            _pBuffer += sizeof(uint16_t);
        }
    }

    static void Write(uint8_t* &_pBuffer, uint32_t _data, bool _movePointer = true)
    {
        memcpy(_pBuffer, &_data, sizeof(uint32_t));

        if(_movePointer)
        {
            _pBuffer += sizeof(uint32_t);
        }
    }

    static void Write(uint8_t* &_pBuffer, uint64_t _data, bool _movePointer = true)
    {
        memcpy(_pBuffer, &_data, sizeof(uint64_t));

        if(_movePointer)
        {
            _pBuffer += sizeof(uint64_t);
        }
    }

    static void Write(uint8_t* &_pBuffer, float _data, bool _movePointer = true)
    {
        memcpy(_pBuffer, &_data, sizeof(float));

        if(_movePointer)
        {
            _pBuffer += sizeof(float);
        }
    }

    static uint8_t ReadUint8(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        uint8_t _data = 0;
        memcpy(&_data, _pBuffer, sizeof(uint8_t));
        if(_movePointer)
        {
            _pBuffer += sizeof(uint8_t);
        }
        return _data;
    }

    static uint16_t ReadUint16(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        uint16_t _data = 0;
        memcpy(&_data, _pBuffer, sizeof(uint16_t));
        if(_movePointer)
        {
            _pBuffer += sizeof(uint16_t);
        }
        return _data;
    }

    static uint32_t ReadUint32(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        uint32_t _data = 0;
        memcpy(&_data, _pBuffer, sizeof(uint32_t));
        if(_movePointer)
        {
            _pBuffer += sizeof(uint32_t);
        }
        return _data;
    }

    static uint64_t ReadUint64(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        uint64_t _data = 0;
        memcpy(&_data, _pBuffer, sizeof(uint64_t));
        if(_movePointer)
        {
            _pBuffer += sizeof(uint64_t);
        }
        return _data;
    }

    static int8_t ReadInt8(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        int8_t _data = 0;
        memcpy(&_data, _pBuffer, sizeof(int8_t));
        if(_movePointer)
        {
            _pBuffer += sizeof(int8_t);
        }
        return _data;
    }

    static int16_t ReadInt16(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        int16_t _data = 0;
        memcpy(&_data, _pBuffer, sizeof(int16_t));
        if(_movePointer)
        {
            _pBuffer += sizeof(int16_t);
        }
        return _data;
    }

    static int32_t ReadInt32(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        int32_t _data = 0;
        memcpy(&_data, _pBuffer, sizeof(int32_t));
        if(_movePointer)
        {
            _pBuffer += sizeof(int32_t);
        }
        return _data;
    }

    static int64_t ReadInt64(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        int64_t _data = 0;
        memcpy(&_data, _pBuffer, sizeof(int64_t));
        if(_movePointer)
        {
            _pBuffer += sizeof(int64_t);
        }
        return _data;
    }

    static float ReadFloat(uint8_t* &_pBuffer, bool _movePointer = true)
    {
        float _data = 0;
        memcpy(&_data, _pBuffer, sizeof(float));
        if(_movePointer)
        {
            _pBuffer += sizeof(float);
        }
        return _data;
    }
};

#endif
