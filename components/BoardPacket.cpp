#include "BoardPacket.hpp"

#include "CanMsgDispatcher.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void BoardPacket::Registration()
{
    uint32_t _channelId = 0;
    switch (m_ChannelCanId)
    {
    case CIT_Channel0:
        _channelId = 0;
        break;
    case CIT_Channel1:
        _channelId = 1;
        break;

    default:
        break;
    }
    BoardPacketManager::Instance()->PacketHandlerRegistration(_channelId, m_ProtocolId, this);
}

void BoardPacket::SendPacket()
{
    BoardPacketManager::Instance()->SendPacket(m_Buffer, m_DataLen + 8);
}

void TestBoardPacket::OnPacketReceived()
{
    // SerializePacket(m_InputStream);
    // NotifyAll((void*)&m_testData);
}

void TestBoardPacket::SendPacket()
{
    SerializePacket(m_OutputStream);
    BoardPacket::SendPacket();
}

template <typename Stream>
void TestBoardPacket::SerializePacket(Stream &stream)
{
    m_DataLen = 24;

    SerializeHeader(stream);

    stream.SerializeU16(m_testData);
    stream.SerializeFloat(m_Vxfdb, -20.0f, 0.01f);
    stream.SerializeFloat(m_Vyfdb, -20.0f, 0.01f);
    stream.SerializeU8(color);
    stream.SerializeU8(shooterId);
    stream.SerializeU8(robotLevel);
    stream.SerializeFloat(bulletSpeed, -1.0f, 0.005f);
    stream.SerializeFloat(cap_energy, -1.0f, 0.1f);
    stream.SerializeFloat(chassis_power, -1.0f, 0.01f);
    stream.SerializeU8(shoot_freq);
    stream.SerializeU16(bullet_remain_num);
    stream.SerializeU16(stage_remain_time);
    stream.SerializeU16(max_heat);
    stream.SerializeU16(cooling_rate);

    SerializeCrc16(stream);
}

BoardPacketManager::BoardPacketManager() : PacketManager(BOARD_CHANNEL_NUM), m_pCan(CAN2)
{
    m_pChannel = m_StreamChannel;
}

void BoardPacketManager::Init()
{
    CanMsgDispatcher::Instance()->RegisterHandler(m_pCan, CIT_Channel0, this);
    CanMsgDispatcher::Instance()->RegisterHandler(m_pCan, CIT_Channel1, this);
    PacketManager::Init();

    m_testPacket.Init(CIT_Channel0, 0x99);
}

void BoardPacketManager::Update()
{
    PacketManager::Update();
}

bool BoardPacketManager::FlushSendBufferLow()
{
    CAN_HandleTypeDef *m_pCanHandle = (m_pCan == CAN1) ? &hcan1 : &hcan2;
    CanManager::Instance()->CanTransmit(m_pCanHandle, CIT_Channel0, m_SendBuffer, m_SendBufferLength);
    return true;
}

void BoardPacketManager::HandleNewCanRxMsg(CanRxMsg *_msg)
{
    if (_msg->Header.StdId == CIT_Channel0)
    {
        Enqueue(0, _msg->Data, _msg->Header.DLC);
        return;
    }
    Enqueue(1, _msg->Data, _msg->Header.DLC);
}
