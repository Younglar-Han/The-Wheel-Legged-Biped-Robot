#include "MemoryStream.hpp"

OutputMemoryStream::OutputMemoryStream(uint8_t* _buffer):
    m_BitHead(0),
    m_Buffer(_buffer)
{}

void OutputMemoryStream::WriteBits(uint8_t _inData, uint32_t _inBitCount)
{
    uint32_t _nextBitHead = m_BitHead + _inBitCount;

    uint32_t _byteOffset = m_BitHead >> 3;
    uint32_t _bitOffset = m_BitHead & 0x07;

    uint8_t _currentMask = ~(0xFF << _bitOffset);

    m_Buffer[_byteOffset] = (m_Buffer[_byteOffset] & _currentMask) | (_inData << _bitOffset);

    uint32_t _bitsFreeThisByte = 8 - _bitOffset;
    if(_bitsFreeThisByte < _inBitCount)
    {
        m_Buffer[_byteOffset + 1] = _inData >> _bitsFreeThisByte;
    }

    m_BitHead = _nextBitHead;
}

void OutputMemoryStream::WriteBits(uint8_t* _pInData, uint32_t _inBitCount)
{
    uint8_t* _srcByte = _pInData;

    while (_inBitCount > 8)
    {
        WriteBits(*_srcByte, 8);
        ++_srcByte;
        _inBitCount -= 8;
    }

    if(_inBitCount > 0)
    {
        WriteBits(*_srcByte, _inBitCount);
    }
}

void OutputMemoryStream::SerializeBytes(uint8_t* _pData, uint8_t _len)
{
    uint32_t _bitLen = 8 * _len;

    WriteBits(_pData, _bitLen);
}

void OutputMemoryStream::SerializeS8(int8_t _data)
{
    WriteBits((uint8_t*)&_data, sizeof(int8_t) * 8);
}

void OutputMemoryStream::SerializeU8(uint8_t _data)
{
    WriteBits((uint8_t*)&_data, sizeof(uint8_t) * 8);
}

void OutputMemoryStream::SerializeS16(int16_t _data)
{
    WriteBits((uint8_t*)&_data, sizeof(int16_t) * 8);
}

void OutputMemoryStream::SerializeU16(uint16_t _data)
{
    WriteBits((uint8_t*)&_data, sizeof(uint16_t) * 8);
}

void OutputMemoryStream::SerializeS32(int32_t _data)
{
    WriteBits((uint8_t*)&_data, sizeof(int32_t) * 8);
}

void OutputMemoryStream::SerializeU32(uint32_t _data)
{
    WriteBits((uint8_t*)&_data, sizeof(uint32_t) * 8);
}

void OutputMemoryStream::SerializeFloat(float _data)
{
    WriteBits((uint8_t*)&_data, sizeof(float) * 8);
}

void OutputMemoryStream::SerializeFloat(float _data, float _inMin, float _inPrecision)
{
    uint16_t _fixed = Math::ConvertToFixed(_data, _inMin, _inPrecision);
    WriteBits((uint8_t*)&_fixed, sizeof(uint16_t) * 8);
}

void OutputMemoryStream::SerializeBool(bool _data)
{
    uint8_t _mapped = _data ? 1 : 0;

    WriteBits(_mapped, 1);
}

InputMemoryStream::InputMemoryStream(uint8_t* _buffer):
    m_BitHead(0),
    m_Buffer(_buffer)
{}

void InputMemoryStream::ReadBits(uint8_t &_outData, uint32_t _outBitCount)
{
    uint32_t _nextBitHead = m_BitHead + _outBitCount;

    uint32_t _byteOffset = m_BitHead >> 3;
    uint32_t _bitOffset = m_BitHead & 0x07;

    uint8_t _currentMask = ~(0xFF << _bitOffset);
    _outData = m_Buffer[_byteOffset] >> _bitOffset | (m_Buffer[_byteOffset + 1] & _currentMask);

    uint8_t _outMask = ~(0xFF << _outBitCount);
    _outData &= _outMask;

    m_BitHead = _nextBitHead;
}

void InputMemoryStream::ReadBits(uint8_t* &_pOutData, uint32_t _outBitCount)
{
    uint8_t* _srcByte = _pOutData;
    while (_outBitCount > 8)
    {
        ReadBits(*_srcByte, 8);
        ++_srcByte;
        _outBitCount -= 8;
    }

    if(_outBitCount > 0)
    {
        ReadBits(*_srcByte, _outBitCount);
    }
}

void InputMemoryStream::SerializeBytes(uint8_t* _pData, uint8_t _len)
{
    uint32_t _bitLen = 8 * _len;

    ReadBits(_pData, _bitLen);
}

void InputMemoryStream::SerializeS8(int8_t &_data)
{
    uint8_t _buffer = _data;
    ReadBits(_buffer, 8);
    _data = _buffer;
}

void InputMemoryStream::SerializeU8(uint8_t &_data)
{
    ReadBits(_data, 8);
}

void InputMemoryStream::SerializeS16(int16_t &_data)
{
    uint8_t _buffer[sizeof(int16_t)];
    SerializeBytes(_buffer, sizeof(int16_t));

    _data = *(int16_t*)_buffer;
}

void InputMemoryStream::SerializeU16(uint16_t &_data)
{
    uint8_t _buffer[sizeof(uint16_t)];
    SerializeBytes(_buffer, sizeof(uint16_t));

    _data = *(uint16_t*)_buffer;
}

void InputMemoryStream::SerializeS32(int32_t &_data)
{
    uint8_t _buffer[sizeof(int32_t)];
    SerializeBytes(_buffer, sizeof(int32_t));

    _data = *(int32_t*)_buffer;
}

void InputMemoryStream::SerializeU32(uint32_t &_data)
{
    uint8_t _buffer[sizeof(uint32_t)];
    SerializeBytes(_buffer, sizeof(uint32_t));

    _data = *(uint32_t*)_buffer;
}

void InputMemoryStream::SerializeFloat(float &_data)
{
    uint8_t _buffer[sizeof(float)];
    SerializeBytes(_buffer, sizeof(float));

    _data = *(float*)_buffer;
}

void InputMemoryStream::SerializeFloat(float &_data, float _inMin, float _inPrecision)
{
    uint16_t _fixed;
    SerializeU16(_fixed);
    _data = Math::ConvertFromFixed(_fixed, _inMin, _inPrecision);
}

void InputMemoryStream::SerializeBool(bool &_data)
{
    uint8_t _buffer;
    ReadBits(_buffer, 1);

    _data = (_buffer == 1);
}

