#ifndef MEMORYSTREAM_HPP
#define MEMORYSTREAM_HPP

#include "stm32f4xx.h"
#include "Math.hpp"

class OutputMemoryStream
{
private:
    uint32_t m_BitHead;
    uint8_t* m_Buffer;
public:
    OutputMemoryStream(uint8_t* _buffer);
    void WriteBits(uint8_t _inData, uint32_t _inBitCount);
    void WriteBits(uint8_t* _pInData, uint32_t _inBitCount);
    uint32_t GetBitLength() const { return m_BitHead; }
    uint32_t GetByteLength() const { return (m_BitHead + 7) >> 3; }
    void Reset(){ m_BitHead = 0; }

    void SerializeBytes(uint8_t* _pData, uint8_t _len);
    void SerializeS8(int8_t _data);
    void SerializeU8(uint8_t _data);
    void SerializeS16(int16_t _data);
    void SerializeU16(uint16_t _data);
    void SerializeS32(int32_t _data);
    void SerializeU32(uint32_t _data);
    void SerializeFloat(float _data);
    void SerializeFloat(float _data, float _inMin, float _inPrecision);
    void SerializeBool(bool _data);
    bool IsInput() const { return false; }
};

class InputMemoryStream
{
private:
    uint32_t m_BitHead;
    uint8_t* m_Buffer;
public:
    InputMemoryStream(uint8_t* _buffer);
    void ReadBits(uint8_t &_outData, uint32_t _outBitCount);
    void ReadBits(uint8_t* &_pOutData, uint32_t _outBitCount);

    void Reset(){ m_BitHead = 0; }

    void SerializeBytes(uint8_t* _pData, uint8_t _len);
    void SerializeS8(int8_t &_data);
    void SerializeU8(uint8_t &_data);
    void SerializeS16(int16_t &_data);
    void SerializeU16(uint16_t &_data);
    void SerializeS32(int32_t &_data);
    void SerializeU32(uint32_t &_data);
    void SerializeFloat(float &_data);
    void SerializeFloat(float &_data, float _inMin, float _inPrecision);
    void SerializeBool(bool &_data);
    bool IsInput() const { return true; }
};

#endif
