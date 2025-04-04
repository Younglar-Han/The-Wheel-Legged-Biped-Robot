#include "Packet.hpp"
#include "Time.hpp"

#include "Stream.hpp"

#include <functional>

#include "Buzzer.hpp"

Packet::Packet() : m_ProtocolId(0xFF),
                   m_DataLen(0),
                   m_InputStream(m_Buffer),
                   m_OutputStream(m_Buffer),
                   m_CallbackLen(0),
                   m_LastReceiveTick(0),
                   m_TimeoutTick(100),
                   m_hasInited(false)
{
}

void Packet::MoveBuffer(const uint8_t* pData, const uint8_t _len)
{
    memcpy(m_Buffer, pData, _len);
}

void Packet::UpdateTick(uint32_t _tick)
{
    m_LastReceiveTick = _tick;
}

StreamChannel::StreamChannel() : m_Buffer{0},
                                 m_PacketBuffer{0},
                                 m_HeaderBuffer{0},
                                 m_BufferFront(0),
                                 m_BufferRear(0),
                                 m_BufferLen(0),
                                 m_PacketLen(0),
                                 m_HeaderLen(0),
                                 m_ExpectedLen(0),
                                 m_Seq(0),
                                 m_CheckingHeader(false)
{
}

void StreamChannel::Enqueue(uint8_t* _pData, uint32_t _len)
{
    for(int i = 0; i < _len; ++i)
    {
        m_Buffer[m_BufferRear] = _pData[i];
        m_BufferRear = (m_BufferRear + 1) % STREAM_CHANNEL_BUFFER_LEN;
    }
}

void StreamChannel::PacketHandlerRegistration(uint32_t _protocolId, Packet* _handler)
{
    m_PacketHandlerTable.Insert(_protocolId, _handler);
}

void StreamChannel::Update()
{
    uint16_t _rear = m_BufferRear;

    while (_rear != m_BufferFront)
    {
        uint8_t _data = m_Buffer[m_BufferFront];
        if(m_PacketLen < STREAM_CHANNEL_BUFFER_LEN)
        {
            m_PacketBuffer[m_PacketLen] = _data;
            ++m_PacketLen;

            if(m_PacketLen == m_ExpectedLen)
            {
                if(Crc::VerifyCrc16CheckSum(m_PacketBuffer, m_PacketLen))
                {
                    HandlePacket();
                }
                m_PacketLen = 0;
            }
        }

        if(m_CheckingHeader)
        {
            m_HeaderBuffer[m_HeaderLen] = _data;
            ++m_HeaderLen;

            if(m_HeaderLen == 5)
            {
                m_CheckingHeader = false;

                if(Crc::VerifyCrc8CheckSum(m_HeaderBuffer, m_HeaderLen))
                {
                    uint8_t* _ptr = m_HeaderBuffer + 1;
                    m_ExpectedLen = ArrayStreamHelper::ReadUint8(_ptr, false) + 8;

                    memcpy(m_PacketBuffer, m_HeaderBuffer, 5);
                    m_PacketLen = 5;
                }
                m_HeaderLen = 0;
            }
        }

        if(_data == 0xA5)
        {
            m_CheckingHeader = true;
            m_HeaderLen = 0;
            m_HeaderBuffer[m_HeaderLen] = _data;
            ++m_HeaderLen;
        }

        m_BufferFront = (m_BufferFront + 1) % STREAM_CHANNEL_BUFFER_LEN;
    }
    
}

void StreamChannel::HandlePacket()
{
    uint8_t _protocolId = m_PacketBuffer[5];
    Packet** _packet = m_PacketHandlerTable.Search(_protocolId);
    if(_packet!=nullptr)
    {
        uint8_t _len = m_PacketBuffer[1];
        (*_packet)->MoveBuffer(m_PacketBuffer, _len + 8);
        (*_packet)->UpdateTick(Time::GetTick());
        (*_packet)->OnPacketReceived();
    }
}

PacketManager::PacketManager(uint32_t _channelNum):
    m_ChannelNum(_channelNum),
    m_pChannel(nullptr)
{
}

void PacketManager::Update()
{
    for(int i = 0; i < m_ChannelNum; ++i)
    {
        m_pChannel[i].Update();
    }
}

void PacketManager::Enqueue(uint32_t _channel, uint8_t* _pData, uint32_t _len)
{
    if(_channel >= m_ChannelNum)
    {
        return;
    }

    m_pChannel[_channel].Enqueue(_pData, _len);
}

void PacketManager::PacketHandlerRegistration(uint32_t _channel, uint32_t _protocolId, Packet* _handler)
{
    if(_channel >= m_ChannelNum || !(_handler->HasInited()))
    {
        return;
    }

    m_pChannel[_channel].PacketHandlerRegistration(_protocolId, _handler);
}

void PacketManager::SendPacket(uint8_t* _pBuffer, uint32_t _len)
{
    uint32_t expectedBufferLen = m_SendBufferLength + _len;
    if(expectedBufferLen > SEND_BUFFER_LEN)
    {
        return;
    }
    memcpy(m_SendBuffer + m_SendBufferLength, _pBuffer, _len);
    m_SendBufferLength = expectedBufferLen;
}

void PacketManager::FlushSendBuffer()
{
    if(m_SendBufferLength == 0)
    {
        return;
    }

    if(m_SendBufferLength >= SEND_BUFFER_LEN)
    {
        m_SendBufferLength = 0;
        Buzzer::Instance()->On();
    }

    if(FlushSendBufferLow())
    {
        m_SendBufferLength = 0;
    }
}

void PacketManager::Init()
{
    m_SendBufferLength = 0;
}
